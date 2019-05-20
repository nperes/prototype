#include "ros/ros.h"
#include "std_msgs/String.h"
#include "prototype/SecureMsg.h"
#include <iostream>

#include <openssl/evp.h>
#include <openssl/hmac.h>
#include <openssl/crypto.h>

namespace {

	class Listener {
		int init_argc;
		char **init_argv;
		ros::Subscriber listener;
		ros::Publisher relay_talker;
		
		public:
				explicit Listener(int argc, char **argv);
				~Listener();

				bool init();
				void start();

		private:
			void chatterCallback(const prototype::SecureMsg &msg);
			uint8_t *hmac_key;
	};
	
	Listener::Listener(int argc, char **argv)
	{
		init_argc = argc;
		init_argv = argv;
		hmac_key = nullptr;
	}
	
	Listener::~Listener()
	{
		if (ros::isStarted())
		{
			ros::shutdown();
			ros::waitForShutdown();
		}
	}
	
	void Listener::chatterCallback(const prototype::SecureMsg &msg)
	{
			static uint32_t md_size_ = EVP_MD_size(EVP_sha256());
			static uint8_t *dummy_hmac_key_ = (uint8_t*)"01234567890123456789012345678901";
			
			const uint8_t *hmac = (const uint8_t*)msg.hmac.c_str();
			uint8_t *md_val = new uint8_t[md_size_];
			uint32_t md_size = 0;
			
			HMAC(EVP_sha256(), dummy_hmac_key_, md_size_, (const uint8_t*)msg.data.c_str(), msg.data.length(), md_val,
	    &md_size);
	    
	    	    ROS_ASSERT(md_size == md_size_);
	    
	    if(0 != CRYPTO_memcmp(md_val, hmac, md_size)) {
		    ROS_INFO("Trusted Listener: dropping [%s]\n", msg.data.c_str());
	    	return;
    	}
			
			ROS_INFO("Trusted Listener: listening: [%s]\n", msg.data.c_str());

			std_msgs::String new_msg;
			new_msg.data = msg.data;
			relay_talker.publish(new_msg);
	}
	
	bool Listener::init()
	{

		ros::init(init_argc, init_argv, "listener");
		if (!ros::master::check())
		{
			std::cerr << "ros down\n";
			return false;
		}
		//must call ros::start before ros::NodeHandle, otherwise, when NodeHandle  goes out of scope the node is automatically shutdown (as per documentation)
		ros::start();
		ros::NodeHandle nh;
		ros::start();
		listener = nh.subscribe("prototype/chatter", 1000, &Listener::chatterCallback, this);
		relay_talker = nh.advertise<std_msgs::String>("prototype/chatter/output", 1000);
		
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
