#include "ros/ros.h"
#include "std_msgs/String.h"
#include "prototype/SecureMsg.h"
#include "prototype/GuiMsgIn.h"
#include <sstream>
#include <vector>

#include <openssl/rand.h>
#include <openssl/evp.h>
#include <openssl/hmac.h>

namespace {

	class Talker {
		int init_argc;
		char **init_argv;
		ros::Subscriber relay_listener;
		ros::Publisher talker;

		
		public:
				explicit Talker(int argc, char **argv);
				~Talker();

				bool init();
				void start();

		private:
			int hmacs_;
			int cipher_;
			uint8_t *dummy_hmac_key_;
			void chatterCallback(const prototype::GuiMsgIn &msg);
			// hmacs
			const EVP_MD *evp_md_algorithm_;
			EVP_MD_CTX *md_ctx_;
			EVP_PKEY *md_pkey_;
			uint32_t md_len_;
			// cypher
			const EVP_CIPHER *evp_cipher_;
			EVP_CIPHER_CTX *cipher_ctx_;
			uint8_t *cipher_key_;

			bool mdGenerate(const uint8_t *msg, uint32_t msg_len, boost::shared_array<uint8_t>& md, uint32_t md_offset=0);
			bool aes256Encrypt(const uint8_t *plaintext, uint32_t plaintext_size, boost::shared_array<uint8_t> &buffer, uint32_t &size, uint32_t offset);
			bool aes256Decrypt(const uint8_t *encryption_data, uint32_t encryption_data_size, boost::shared_array<uint8_t> &buffer, uint32_t &size, uint32_t offset);
			bool evp_cipher(const uint8_t *input, int input_len, uint8_t *output, uint32_t &output_len, const uint8_t *iv, bool encrypt);
	};
	
	Talker::Talker(int argc, char **argv)
	{
		init_argc = argc;
		init_argv = argv;
		dummy_hmac_key_ = (uint8_t*)"01234567890123456789012345678901";
	}
	
	Talker::~Talker()
	{
		if (ros::isStarted())
		{
			ros::shutdown();
			ros::waitForShutdown();
		}
	}
	
	bool Talker::aes256Encrypt(const uint8_t *plaintext,
    uint32_t plaintext_size, boost::shared_array<uint8_t> &buffer,
    uint32_t &size, uint32_t offset)
	{
		uint32_t cipher_block_size = EVP_CIPHER_block_size(evp_cipher_);
		uint32_t iv_size = EVP_CIPHER_iv_length(evp_cipher_);

		if (!buffer)
		{
			// take padding into account
			uint32_t max_ciphertext_size = (plaintext_size / cipher_block_size)
				  * cipher_block_size + cipher_block_size;
			uint32_t buffer_size = offset + iv_size + max_ciphertext_size;
			buffer = boost::shared_array<uint8_t>(new uint8_t[buffer_size]);
		}

		uint8_t *iv = buffer.get() + offset;
		uint8_t *ciphertext = iv + iv_size;

		// new random IV for this encryption
		if (1 != RAND_bytes(iv, iv_size))
		{
//			ERR_print_errors_cb(errorHandler, nullptr);
			return false;
		}

		uint32_t ciphertext_len = 0;
		if (!evp_cipher(plaintext, plaintext_size, ciphertext, ciphertext_len, iv,
			  true))
		{
			return false;
		}

		size = iv_size + ciphertext_len;

		return true;
	}
	
	
	
	bool Talker::evp_cipher(const uint8_t *input, int input_len,
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
		(1
			      != EVP_CipherInit_ex(cipher_ctx_, evp_cipher_, NULL, cipher_key_, iv,
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
		
	
	bool Talker::mdGenerate(const uint8_t *msg, uint32_t msg_len,
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
	
	
	void Talker::chatterCallback(const prototype::GuiMsgIn &msg)
	{
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
	
		prototype::SecureMsg new_msg;
		new_msg.hmacs = msg.hmacs;
		new_msg.encryption = msg.encryption;

		std::vector<uint8_t> plain = std::vector<uint8_t>(msg.data.begin(), msg.data.end());
		const uint8_t *plaintext = reinterpret_cast<const uint8_t*>(msg.data.data());
		uint32_t plaintext_len = msg.data.size();

		boost::shared_array<uint8_t> out_buffer_cph = nullptr;
		uint32_t out_written_cph = -1;
		aes256Encrypt(plaintext, plaintext_len, out_buffer_cph, out_written_cph, 0);

		new_msg.data = std::vector<uint8_t>(out_buffer_cph.get(), out_buffer_cph.get() + out_written_cph);
		
		boost::shared_array<uint8_t> md_buffer = nullptr;
		mdGenerate(out_buffer_cph.get(), out_written_cph, md_buffer, 0);
		
		new_msg.hmac = std::vector<uint8_t>(md_buffer.get(), md_buffer.get()+md_len_);
	
		ROS_INFO("Trusted Publisher: publishing: [%s]\n", msg.data.c_str());
		talker.publish(new_msg);
	}
	
	bool Talker::init()
	{
		ros::init(init_argc, init_argv, "talker");
		
		//check ROS
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
		talker = nh.advertise<prototype::SecureMsg>("prototype/chatter", 1000);
		relay_listener = nh.subscribe("prototype/chatter/input", 1000, &Talker::chatterCallback, this);
		
		return true;
	}
	
	void Talker::start()
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
	Talker talker(argc, argv);
	if(talker.init())
		talker.start();
  return 0;
}
