#ifndef QROSNODE_H
#define QROSNODE_H

#include "prototype/SecureMsg.h"

#include <ros/ros.h>
//#include <QtWidgets>
#include <QObject>
#include <std_msgs/String.h>



Q_DECLARE_METATYPE(std::string);

namespace prototype {

class QRosNode : public QObject
{
	Q_OBJECT

public:
	explicit QRosNode(int argc, char **argv, QObject *parent=0);
	virtual ~QRosNode();
	// start ros; no effect if ros already started
	bool setup();
	// indicates whether ros is already started
	bool isStarted();

	
protected:
	// requires: isStarted() == true
	void startNode();
	
private:
	int init_argc;
	char **init_argv;
	void onNewLegitimateMsg(const std_msgs::String::ConstPtr& msg);
	void onNewCapturedMsg(const prototype::SecureMsg& msg);
	ros::Subscriber trusted_subscriber;
	ros::Subscriber mitm_subscriber;
	ros::Publisher trusted_publisher;
	ros::Publisher mitm_publisher;
	
	

public slots:
	void onStart();
	void onGuiPublishRequest(std::string msg);
	void onGuiInjectRequest(std::string msg);

signals:
	void started();
	void finished();
	void error(QString);
	void newLegitimateMsg(std::string msg);
	void newCapturedMsg(std::string msg);
	
};//class QRosNode

}//namespace protoype
#endif//QROSNODE_H
