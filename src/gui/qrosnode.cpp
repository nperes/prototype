#include "../include/prototype/gui/qrosnode.h"

#include <iostream>
#include <string>
#include <mutex>

namespace prototype {

namespace {


const std::string Default_Nodename = "qrosnode";
const int Default_Msg_Buffer_Size = 1000;
const int Default_Loop_Rate_Milli = 1000;
const std::string Chatter = "/prototype/chatter";
const std::string Chatter_Input = "/prototype/chatter/input";
const std::string Chatter_Output = "/prototype/chatter/output";

bool is_started = false;

std::mutex trusted_mtx;
std::atomic_flag skip_trusted_publishing;
std_msgs::String publish_msg;

std::mutex inject_mtx;
std::atomic_flag skip_inject_publishing;
prototype::SecureMsg inject_msg;

}//namespace

/******************************************************************************
	PUBLIC
******************************************************************************/

QRosNode::QRosNode(int argc, char **argv, QObject *parent)
	: QObject(parent)
	, init_argc(argc)
	, init_argv(argv)
{
	qRegisterMetaType<std::string>();
}

QRosNode::~QRosNode()
{}

// initial setup - check ros
bool QRosNode::setup()
{
	if(isStarted())
		return true;
	ros::init(init_argc, init_argv, Default_Nodename);
	if (!ros::master::check()) {
		emit error(QString::fromStdString("\nSetup failed: ROS is down"));
		return false;
	}
	ros::start();
	ros::Time::init();
	is_started = true;
	return true;
}

bool QRosNode::isStarted()
{
	return is_started;
}

/******************************************************************************
	PROTECTED
******************************************************************************/

void QRosNode::startNode()
{
	skip_trusted_publishing.test_and_set();
	
	ros::NodeHandle nh;
	trusted_subscriber = nh.subscribe(Chatter_Output, Default_Msg_Buffer_Size, &QRosNode::onNewLegitimateMsg, this);
	trusted_publisher = nh.advertise<std_msgs::String>(Chatter_Input, Default_Msg_Buffer_Size);
	mitm_subscriber = nh.subscribe(Chatter, Default_Msg_Buffer_Size, &QRosNode::onNewCapturedMsg, this);
	mitm_publisher = nh.advertise<prototype::SecureMsg>(Chatter, Default_Msg_Buffer_Size);
  ros::Rate loop_rate(Default_Loop_Rate_Milli);
  
  while (ros::ok() && isStarted())
  {
		if(!skip_trusted_publishing.test_and_set())
		{
	 		std::unique_lock<std::mutex> lck(trusted_mtx);
			trusted_publisher.publish(publish_msg);
			lck.unlock();
		}
		if(!skip_inject_publishing.test_and_set())
	 	{
			std::unique_lock<std::mutex> lck(inject_mtx);
			mitm_publisher.publish(inject_msg);
			lck.unlock();
		}

   ros::spinOnce();
   loop_rate.sleep();
  }
  //maybe a ctrl+c, maybe ros died,...
	is_started = false;
  emit finished();
}

/******************************************************************************
	PRIVATE
******************************************************************************/

void QRosNode::onNewLegitimateMsg(const std_msgs::String::ConstPtr& msg)
{
	emit newLegitimateMsg(msg->data);
}

void QRosNode::onNewCapturedMsg(const prototype::SecureMsg& msg)
{
	emit newCapturedMsg(msg.data);
}

/******************************************************************************
	SLOTS
******************************************************************************/

void QRosNode::onStart()
{
	
	startNode();
}

void QRosNode::onGuiPublishRequest(std::string msg)
{
	std_msgs::String aux;
	aux.data = msg;

	std::unique_lock<std::mutex> lck(trusted_mtx);
		publish_msg = aux;
		skip_trusted_publishing.clear();
	lck.unlock();	
}

void QRosNode::onGuiInjectRequest(std::string msg)
{
	prototype::SecureMsg aux;
	aux.data = msg.c_str();

	std::unique_lock<std::mutex> lck(inject_mtx);
		inject_msg = aux;
		skip_inject_publishing.clear();
	lck.unlock();	
}

}//namespace prototype
