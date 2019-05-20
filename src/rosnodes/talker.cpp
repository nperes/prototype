#include "ros/ros.h"
#include "std_msgs/String.h"
#include "prototype/SecureMsg.h"
#include <sstream>

namespace {

	class Talker {
		int init_argc;
		char **init_argv;
		ros::Publisher talker;
		ros::Subscriber relay_listener;
		
		public:
				explicit Talker(int argc, char **argv);
				~Talker();

				bool init();
				void start();

		private:
			void chatterCallback(const std_msgs::String::ConstPtr &msg);
	};
	
	Talker::Talker(int argc, char **argv)
	{
		init_argc = argc;
		init_argv = argv;
	}
	
	Talker::~Talker()
	{
		if (ros::isStarted())
		{
			ros::shutdown();
			ros::waitForShutdown();
		}
	}
	
	void Talker::chatterCallback(const std_msgs::String::ConstPtr &msg)
	{
			ROS_INFO("Publishing: [%s]\n", msg->data.c_str());
			prototype::SecureMsg new_msg;
			new_msg.data = msg->data.c_str();
			talker.publish(new_msg);
	}
	
	bool Talker::init()
	{
		ros::init(init_argc, init_argv, "talker");
		if (!ros::master::check())
		{
			std::cerr << "ros down\n";
			return false;
		}
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
