#ifndef QROSNODE_H
#define QROSNODE_H

#include "prototype/SecureMsg.h"
#include "prototype/GuiMsgOut.h"

#include <ros/ros.h>
//#include <QtWidgets>
#include <QObject>
#include <std_msgs/String.h>



Q_DECLARE_METATYPE(std::string);
Q_DECLARE_METATYPE(prototype::GuiMsgOut);

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
	void onNewLegitimateMsg(const prototype::GuiMsgOut& msg);
	void onNewCapturedMsg(const prototype::SecureMsg& msg);
	ros::Subscriber trusted_subscriber;
	ros::Subscriber mitm_subscriber;
	ros::Publisher trusted_publisher;
	ros::Publisher mitm_publisher;
	
	

public slots:
	void onStart();
	void onGuiPublishRequest(std::string msg, bool hmacs, bool encryption);
	void onGuiInjectRequest(std::string msg, bool hmacs, bool encryption);

signals:
	void started();
	void finished();
	void error(QString);
	//void newLegitimateMsg(std::string msg);
	void newLegitimateMsg(QByteArray msg);
	void newHMAC(QByteArray msg);
	void newLegitimateMsg1(QByteArray msg);
	void newCapturedMsg(std::string msg);
	
};//class QRosNode

}//namespace protoype
#endif//QROSNODE_H
