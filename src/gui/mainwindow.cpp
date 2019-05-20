#include "../include/prototype/gui/mainwindow.h"
#include "../include/prototype/gui/qrosnode.h"
#include "./ui_mainwindow.h"

#include <string>
#include <iostream>
#include <mutex>
#include <condition_variable>
#include <thread>

#include <QThread>
#include <QDebug>


namespace prototype {

namespace {

QColor disabled_widget_color()
{
	static QColor disabled_widget_color = QApplication::palette("QComboBox").color(QPalette::Disabled, QPalette::Window);
	return disabled_widget_color;
}

void set_text_field_disabled_look(QWidget *qwidget)
{
	QPalette pal = qwidget->palette();
	pal.setColor(QPalette::Base, disabled_widget_color());
	qwidget->setPalette(pal);
}

void init_ui(Ui::MainWindow *ui)
{
	set_text_field_disabled_look(ui->qtextedit_legitimateSubscriberMsg);
	set_text_field_disabled_look(ui->qtextedit_randomSubscriberMsg);
	ui->lineEdit_legitimatePublishRateMilli->setCurrentIndex(1);
	ui->lineEdit_randomPublishRateMilli->setCurrentIndex(1);
}

void set_publisher_style(bool shouldPublish , QGroupBox *parent)
{
	QTextEdit *msg = parent->findChild<QTextEdit*>();
	msg->setReadOnly(shouldPublish);
	QComboBox *publish_rate_millis = parent->findChild<QComboBox*>();
	publish_rate_millis->setEnabled(!shouldPublish);
	
	if(shouldPublish)
		set_text_field_disabled_look(msg);
	else
		msg->setPalette(msg->parentWidget()->palette());
		
	msg->setAutoFillBackground(true);
}

std::mutex trusted_publish_mtx;
std::condition_variable trusted_publish_cv;
std::chrono::milliseconds trusted_publish_publishing_rate_milli;
std::string trusted_publish_msg;
bool trusted_publish_requested;

std::mutex inject_publish_mtx;
std::condition_variable inject_publish_cv;
std::chrono::milliseconds inject_publish_publishing_rate_milli;
std::string inject_publish_msg;
bool inject_publish_requested;

bool running;

std::atomic_flag keep_alive = ATOMIC_FLAG_INIT;
std::atomic_flag legitimate_publish_pending = ATOMIC_FLAG_INIT;
std::atomic_flag random_publish_pending = ATOMIC_FLAG_INIT;


std::mutex gui_mtx;

}//namespace


/******************************************************************************
	PUBLIC
******************************************************************************/


MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
  : QMainWindow(parent)
  , ui(new Ui::MainWindow)
	, init_argc(argc)
	, init_argv(argv)
{
	// start ui (autoconnect done here)
	ui->setupUi(this);
	init_ui(ui);

// gui node (holds publishers and subscribers)
	QRosNode *gui_node = new QRosNode(init_argc, init_argv);
	QThread *thread_gui_node = new QThread();

	// signals & slots -> connect(sender, signal, receiver, slot)
	// start node on thread start
	connect(thread_gui_node, SIGNAL(started()), gui_node, SLOT(onStart()));
	// chain end-of-life actions to close as a whole
	connect(this, SIGNAL(destroyed()), thread_gui_node, SLOT(quit()));
	connect(gui_node, SIGNAL(destroyed()), thread_gui_node, SLOT(quit()));
	connect(gui_node, SIGNAL (finished()), thread_gui_node, SLOT(quit()));
	connect(thread_gui_node, SIGNAL(finished()), this, SLOT(close()));
	// allow error handling in main thread
	connect(gui_node, SIGNAL (error(QString)), this, SLOT(errorString(QString)));
	// connect gui published/subscribed signals to gui - gui reacts to pub/sub actions
	connect(gui_node, SIGNAL(newLegitimateMsg(std::string)), this, SLOT(onNewLegitimateMsg(std::string)));
	connect(gui_node, SIGNAL(newCapturedMsg(std::string)), this, SLOT(onNewCapturedMsg(std::string)));
	// connect publish/subscribe gui requests to publishers/subscribers -> pub/sub reacts to gui requests
	connect(this, SIGNAL(publish(std::string)), gui_node, SLOT(onGuiPublishRequest(std::string)), Qt::DirectConnection);
	connect(this, SIGNAL(inject(std::string)), gui_node, SLOT(onGuiInjectRequest(std::string)), Qt::DirectConnection);

	// launch gui node
	gui_node->moveToThread(thread_gui_node);
	if(gui_node->setup())
	{
		std::cerr << "\nGUI node starting...\n";
		thread_gui_node->start();
	}
	else
	
	{
		std::cerr << "Unable to set up ROS node: closing...\n";
		//TODO handle properly
		this->close();
	}
}

MainWindow::~MainWindow()
{
	if (ros::isStarted())
	{
		ros::shutdown();
		ros::waitForShutdown();
	}
	std::unique_lock<std::mutex> lock1(trusted_publish_mtx, std::defer_lock);
	std::unique_lock<std::mutex> lock2(inject_publish_mtx, std::defer_lock);
	{
		std::lock(lock1, lock2);
		running = false;
	}
	trusted_publish_cv.notify_all();
	inject_publish_cv.notify_all();
	delete ui;
}

/******************************************************************************
	PROTECTED
******************************************************************************/



/******************************************************************************
	PRIVATE
******************************************************************************/

void MainWindow::onGuiPublishRequest(std::string msg)
{
	emit publish(msg);
}


void MainWindow::onGuiInjectRequest(std::string msg)
{
		emit inject(msg);
}

void MainWindow::appendToTextEdit(QTextEdit *qTextEdit, std::string msg)
{
	QString q_msg = QString::fromStdString(msg+"\n");
	gui_mtx.lock();
	qTextEdit->insertPlainText(q_msg);
	qTextEdit->moveCursor (QTextCursor::End);
	gui_mtx.unlock();
}


/******************************************************************************
	SLOTS
******************************************************************************/

void MainWindow::on_pushButton_toggleLegitimatePublishing_clicked(bool check)
{
	set_publisher_style(check, ui->groupBox_legitimatePublisher);
	
	if(check)//must start publish
	{
		std::string msg = ui->textEdit_legitimatePublisherMsg->toPlainText().toUtf8().constData();
		int rate_milli = ui->lineEdit_legitimatePublishRateMilli->currentText().toInt();

		std::unique_lock<std::mutex> lck(trusted_publish_mtx);

		trusted_publish_publishing_rate_milli = std::chrono::milliseconds(rate_milli);
		
		trusted_publish_msg = msg;
		trusted_publish_requested = true;
		trusted_publish_cv.notify_one();
		lck.unlock();
	}
	else
	{
		std::unique_lock<std::mutex> lck(trusted_publish_mtx);
		trusted_publish_requested = false;
		lck.unlock();
	}
}

void MainWindow::on_pushButton_toggleRandomPublishing_clicked(bool check)
{
	set_publisher_style(check, ui->groupBox_randomPublisher);
	
	if(check)//must start publish
	{
		std::string msg = ui->textEdit_randomPublisherMsg->toPlainText().toUtf8().constData();
		int rate_milli = ui->lineEdit_randomPublishRateMilli->currentText().toInt();

		std::unique_lock<std::mutex> lck(inject_publish_mtx);

		inject_publish_publishing_rate_milli = std::chrono::milliseconds(rate_milli);
		inject_publish_msg = msg;
		inject_publish_requested = true;
		inject_publish_cv.notify_one();
		lck.unlock();
	}
	else
	{
		std::unique_lock<std::mutex> lck(inject_publish_mtx);
		inject_publish_requested = false;
		lck.unlock();
	}
}

void MainWindow::threading()
{
	running = true;

// simple worker -> do{publish current msg; sleep current time} while(not told to stop)
	auto test_worker1 = [&](void (MainWindow::*f)(std::string), bool &publish, std::mutex &mtx, std::condition_variable &cv, std::chrono::milliseconds &publishing_rate_milli, std::string &msg){
		for(;;)
			{
				std::unique_lock<std::mutex> lck(mtx);
				cv.wait(lck,[&]{return publish||!running;});
				if(!running) {
					lck.unlock();
					return;
				}
				std::string msg_copy = std::string(msg);
				lck.unlock();
				(this->*f)(msg_copy);
				std::this_thread::sleep_for(std::chrono::milliseconds(publishing_rate_milli));
			}		
	};

// trusted publisher
	std::thread t1(test_worker1, &MainWindow::onGuiPublishRequest, std::ref(trusted_publish_requested), std::ref(trusted_publish_mtx), std::ref(trusted_publish_cv), std::ref(trusted_publish_publishing_rate_milli), std::ref(trusted_publish_msg));
	t1.detach();
// untrusted publisher
	std::thread t2(test_worker1, &MainWindow::onGuiInjectRequest, std::ref(inject_publish_requested), std::ref(inject_publish_mtx), std::ref(inject_publish_cv), std::ref(inject_publish_publishing_rate_milli), std::ref(inject_publish_msg));
	t2.detach();
}

void MainWindow::onNewLegitimateMsg(std::string msg)
{
	appendToTextEdit(ui->qtextedit_legitimateSubscriberMsg, msg);
}

void MainWindow::onNewCapturedMsg(std::string msg)
{
	appendToTextEdit(ui->qtextedit_randomSubscriberMsg, msg);
}

void MainWindow::errorString(QString error_msg)
{
	qDebug() << qPrintable(error_msg);
}

}//namespace prototype
