#include "ros/ros.h"
#include "std_msgs/String.h"
#include "prototype/SecureMsg.h"
#include <iostream>

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

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
	};
	
	Listener::Listener(int argc, char **argv)
	{
		init_argc = argc;
		init_argv = argv;
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
			ROS_INFO("Listening: [%s]\n", msg.data.c_str());
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
		//TODO(nmf) move private member of Listener 
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
