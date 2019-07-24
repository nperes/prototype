#include "../include/prototype/gui/qrosnode.h"
#include "prototype/GuiMsgIn.h"
#include "prototype/GuiMsgOut.h"

#include <iostream>
#include <string>
#include <mutex>

#include<QDebug>

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
//TODO new
//std_msgs::String publish_msg;
GuiMsgIn publish_msg;

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
	//TODO(nmf) new
	trusted_subscriber = nh.subscribe(Chatter_Output, Default_Msg_Buffer_Size, &QRosNode::onNewLegitimateMsg, this);
	//TODO(nmf) new
	//trusted_publisher = nh.advertise<std_msgs::String>(Chatter_Input, Default_Msg_Buffer_Size);
	trusted_publisher = nh.advertise<GuiMsgIn>(Chatter_Input, Default_Msg_Buffer_Size);
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

//TODO(nmf) new
//void QRosNode::onNewLegitimateMsg(const std_msgs::String::ConstPtr& msg)
void QRosNode::onNewLegitimateMsg(const prototype::GuiMsgOut& msg)
{
	if(msg.hmacs == 1)
	{
		QByteArray hmac(reinterpret_cast<const char*>(msg.hmac.data()), msg.hmac.size());
		emit newHMAC(hmac);
	}

	QByteArray msg_(reinterpret_cast<const char*>(msg.msg.data()), msg.msg.size());
	emit newLegitimateMsg(msg_);
}

void QRosNode::onNewCapturedMsg(const prototype::SecureMsg& msg)
{
	size_t data_size = msg.hmac.size()+msg.data.size();	
	uint8_t *data = new uint8_t[data_size];

	memcpy(data, msg.hmac.data(), msg.hmac.size());
	memcpy(data+msg.hmac.size(), msg.data.data(), msg.data.size());
	
	/*
	QByteArray b = QByteArray((const char*)msg.data.data(), msg.data.size());
	if(msg.data.size()>0)
	{
		fprintf(stderr, "%02x\n", msg.data.data()[0]);
		fprintf(stderr, "%02x\n", (uint8_t)b.at(0));
	}
	const uint8_t *tester = (const uint8_t*)b.data();
	for(int i = 0; i < b.size();++i)
		fprintf(stderr, "%02x\n", tester[i]);
	*/
	
	std::ostringstream oss;
	std::copy(data, data+data_size, std::ostream_iterator<uint8_t>(oss));
	emit newCapturedMsg(oss.str());
}

/******************************************************************************
	SLOTS
******************************************************************************/

void QRosNode::onStart()
{
	
	startNode();
}

void QRosNode::onGuiPublishRequest(std::string msg, bool hmacs, bool encryption)
{
	std_msgs::String aux;
	aux.data = msg;
	
	GuiMsgIn new_msg;
	new_msg.data = msg;
	new_msg.hmacs = hmacs;
	new_msg.encryption = encryption;

	std::unique_lock<std::mutex> lck(trusted_mtx);
		publish_msg = new_msg;
		skip_trusted_publishing.clear();
	lck.unlock();	
}

void QRosNode::onGuiInjectRequest(std::string msg, bool hmacs, bool encryption)
{

	
	QByteArray b = QByteArray(msg.c_str(), msg.length());
	const uint8_t *tester = (const uint8_t*)b.data();

	prototype::SecureMsg aux;
//	aux.data = std::vector<uint8_t>(msg.begin(), msg.end());
	aux.data = std::vector<uint8_t>(tester, tester+b.size());
	aux.hmacs = hmacs;
	aux.encryption = encryption;

	std::unique_lock<std::mutex> lck(inject_mtx);
		inject_msg = aux;
		skip_inject_publishing.clear();
	lck.unlock();	
}

}//namespace prototype
