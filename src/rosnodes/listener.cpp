#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8MultiArray.h"
#include "prototype/SecureMsg.h"
#include "prototype/GuiMsgOut.h"
#include <iostream>
#include <vector>
#include <iomanip>
#include <sstream>

#include <openssl/rand.h>
#include <openssl/evp.h>
#include <openssl/hmac.h>
#include <openssl/crypto.h>

namespace {

	class Listener {
		int init_argc;
		char **init_argv;
		ros::Subscriber listener;
		ros::Publisher relay_Listener;
		
		public:
				explicit Listener(int argc, char **argv);
				~Listener();

				bool init();
				void start();

		private:
			void chatterCallback(const prototype::SecureMsg &msg);
			uint8_t *hmac_key;
			// hmacs & cipher
			int hmacs_;
			int cipher_;
			// hmacs
			const EVP_MD *evp_md_algorithm_;
			EVP_MD_CTX *md_ctx_;
			EVP_PKEY *md_pkey_;
			uint32_t md_len_;
			// cipher
			const EVP_CIPHER *evp_cipher_;
			EVP_CIPHER_CTX *cipher_ctx_;
			uint8_t *cipher_key_;
			//
			bool mdGenerate(const uint8_t *msg, uint32_t msg_len, boost::shared_array<uint8_t>& md, uint32_t md_offset=0);
			bool mdValidate(const uint8_t *msg, uint32_t msg_len, const uint8_t* md);
			bool aes256Decrypt(const uint8_t *encryption_data, uint32_t encryption_data_size, boost::shared_array<uint8_t> &buffer, uint32_t &size, uint32_t offset);
			bool evp_cipher(const uint8_t *input, int input_len, uint8_t *output, uint32_t &output_len, const uint8_t *iv, bool encrypt);
	};	
	
	Listener::Listener(int argc, char **argv)
	{
		init_argc = argc;
		init_argv = argv;
		hmac_key = (uint8_t*)"01234567890123456789012345678901";
	}
	
	Listener::~Listener()
	{
		if (ros::isStarted())
		{
			ros::shutdown();
			ros::waitForShutdown();
		}
	}
	
	
	bool Listener::aes256Decrypt(const uint8_t *encryption_data, uint32_t encryption_data_size, boost::shared_array<uint8_t> &buffer, uint32_t &size, uint32_t offset)
	{
		uint32_t cipher_block_size = EVP_CIPHER_block_size(evp_cipher_);
		uint32_t iv_size = EVP_CIPHER_iv_length(evp_cipher_);

		if(encryption_data_size <= iv_size)
		{
			return false;
		}

		const uint8_t *iv = encryption_data;
		const uint8_t *ciphertext = iv + iv_size;

		uint32_t ciphertext_size = encryption_data_size - iv_size;

		if (!buffer)
		{
			// account for padding
			uint32_t max_plaintext_size = (ciphertext_size / cipher_block_size)
				  * cipher_block_size + cipher_block_size;
			uint32_t buffer_size = offset + max_plaintext_size;
			buffer = boost::shared_array<uint8_t>(new uint8_t[buffer_size]);
		}

		uint32_t decrypt_len = 0;
		if (!evp_cipher(ciphertext, ciphertext_size, buffer.get() + offset,
			  decrypt_len, iv, false))
		{
			return false;
		}

		size = decrypt_len;

		return true;
	}
	
	bool Listener::evp_cipher(const uint8_t *input, int input_len,
    uint8_t *output, uint32_t &output_len, const uint8_t *iv, bool encrypt)
	{
		int key_length = EVP_CIPHER_key_length(evp_cipher_);
		int iv_length = EVP_CIPHER_iv_length(evp_cipher_);
		int do_encrypt = encrypt ? 1 : 0;

		if (1
			  != (EVP_CipherInit_ex(cipher_ctx_, evp_cipher_, NULL, NULL, NULL,
			      do_encrypt)))
		{
			return false;
		}

		ROS_ASSERT(EVP_CIPHER_CTX_key_length(cipher_ctx_) == key_length);
		ROS_ASSERT(EVP_CIPHER_CTX_iv_length(cipher_ctx_) == iv_length);

		int tmp_len = 0, current_len = 0;

		if (
		(1 != EVP_CipherInit_ex(cipher_ctx_, evp_cipher_, NULL, cipher_key_, iv,
			          do_encrypt))
		|| (1 != EVP_CipherUpdate(cipher_ctx_, output, &tmp_len, input, input_len)))
		{
			return false;
		}

		current_len = tmp_len;

		if (1 != EVP_CipherFinal_ex(cipher_ctx_, output + current_len, &tmp_len))
		{
			return false;
		}

		current_len += tmp_len;
		output_len = current_len;

		return true;
	}
		
	
	bool Listener::mdGenerate(const uint8_t *msg, uint32_t msg_len,
    boost::shared_array<uint8_t>& md, uint32_t md_offset)
	{
		if (!md)
			md = boost::shared_array<uint8_t>(new uint8_t[md_offset + md_len_]);

		size_t md_len;

		if (1 != EVP_DigestInit_ex(md_ctx_, evp_md_algorithm_, NULL)
			  || 1
			      != EVP_DigestSignInit(md_ctx_, nullptr, evp_md_algorithm_, nullptr,
			          md_pkey_) || 1 != EVP_DigestSignUpdate(md_ctx_, msg, msg_len)
			  || 1 != EVP_DigestSignFinal(md_ctx_, md.get() + md_offset, &md_len))
		{
			//ERR_print_errors_cb(errorHandler, nullptr);
			return false;
		}

		ROS_ASSERT(md_len == md_len_);

		return true;
	}
	
	bool Listener::mdValidate(const uint8_t *msg, uint32_t msg_len,
    const uint8_t* md)
	{
		boost::shared_array<uint8_t> md_tmp = boost::shared_array<uint8_t>(
			  new uint8_t[md_len_]);

		if (!mdGenerate(msg, msg_len, md_tmp))
		{
			return false;
		}

		if (0 != CRYPTO_memcmp(md_tmp.get(), md, md_len_))
		{
			ROS_DEBUG_NAMED("superdebug", "HMAC Verify: failed integrity check");
			return false;
		}

		return true;
	}
	
	void Listener::chatterCallback(const prototype::SecureMsg &msg)
	{
		std::string msg_ = std::string(msg.data.begin(),msg.data.end());
		ROS_INFO("[Trusted Listener] heard [%s]\n", msg_.c_str());
	
		if(msg.hmacs != hmacs_)
		{
			if(msg.hmacs == 0)
			{
				evp_md_algorithm_ = EVP_md_null();
			}
			else
			{
				evp_md_algorithm_ = EVP_sha256();
			}
			md_len_ = EVP_MD_size(evp_md_algorithm_);
		}
	
		if(msg.encryption != cipher_)
		{
			if(msg.encryption == 0)
			{
				evp_cipher_ = EVP_enc_null();
			}
			else
			{
				evp_cipher_ = EVP_aes_256_cbc();
			}
		}	
		
		if(msg.hmac.size() != md_len_)
		{
			ROS_WARN("Trusted Listener: dropping [%s]\n", msg_.c_str());	
			return;
		}
		
		
		prototype::GuiMsgOut new_msg;
		new_msg.hmac = msg.hmac;
		new_msg.hmacs = msg.hmacs;
		new_msg.encryption = msg.encryption;
		
		if(!mdValidate(msg.data.data(), msg.data.size(), msg.hmac.data()))
		{
			ROS_WARN("Trusted Listener: dropping [%s]\n", msg_.c_str());
	  	return;
		}
		
		boost::shared_array<uint8_t> out_buffer_dcph = nullptr;
		uint32_t out_written_dcph = -1;
		
		if(!aes256Decrypt(msg.data.data(), msg.data.size(), out_buffer_dcph, out_written_dcph, 0))
		{
			ROS_WARN("Trusted Listener: dropping [%s]\n", msg_.c_str());
			return;
		}
		new_msg.msg = std::vector<uint8_t>(out_buffer_dcph.get(), out_buffer_dcph.get()+out_written_dcph);
		
		relay_Listener.publish(new_msg);
	}
	
	bool Listener::init()
	{

		ros::init(init_argc, init_argv, "listener");
		if (!ros::master::check())
		{
			std::cerr << "ros down\n";
			return false;
		}

		hmacs_ = -1;
		cipher_ = -1;

		// 32 bits
		uint8_t *md_key = (uint8_t*)"01234567890123456789012345678901";

		// hmacs
		md_pkey_ = EVP_PKEY_new_mac_key(EVP_PKEY_HMAC, NULL, md_key, 32);
		md_ctx_ = EVP_MD_CTX_new();
		
		// cypher
		cipher_ctx_ = EVP_CIPHER_CTX_new();
		cipher_key_ = (uint8_t*)"01234567890123456789012345678901";

		//must call ros::start before ros::NodeHandle, otherwise, when NodeHandle  goes out of scope the node is automatically shutdown (as per documentation)
		ros::start();
		ros::NodeHandle nh;
		ros::start();
		listener = nh.subscribe("prototype/chatter", 1000, &Listener::chatterCallback, this);
		relay_Listener = nh.advertise<prototype::GuiMsgOut>("prototype/chatter/output", 1000);
		
		return true;
	}
	
	void Listener::start()
	{
		ros::Time::init();
		ros::Rate loop_rate(100);
		while (ros::ok())
		{
			ros::spinOnce();
	    loop_rate.sleep();
	  }
	}
}//namespace


int main(int argc, char **argv)
{

	Listener listener(argc, argv);
	if(listener.init())
		listener.start();
  return 0;
}
