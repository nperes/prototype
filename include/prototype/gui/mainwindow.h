#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMetaType>
#include <QMainWindow>
#include <QTextEdit>
//#include <QtWidgets>
#include <std_msgs/String.h>

//Q_DECLARE_METATYPE(std::string);

namespace Ui {
class MainWindow;
}

namespace prototype {

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	explicit MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();
	void threading();

public slots:
	void on_pushButton_toggleLegitimatePublishing_clicked(bool check);
	void on_pushButton_toggleRandomPublishing_clicked(bool check);
	void on_checkBox_hmacs_clicked(bool check);
//	void onNewLegitimateMsg(std::string msg);
	void onNewLegitimateMsg(QByteArray msg);
//	void onNewLegitimateMsg1(QByteArray msg);
	void onNewHMAC(QByteArray hmac);
	void onNewCapturedMsg(std::string msg);
	void errorString(QString);

signals:
	void publish(std::string msg, bool hmacs, bool encryption);
	void inject(std::string msg, bool hmacs, bool encryption);

private:
	Ui::MainWindow *ui;
	int init_argc;
	char **init_argv;
	void appendToTextEdit(QTextEdit *qTextEdit, std::string msg);
	void appendToTextEdit(QTextEdit *qTextEdit, QString q_msg);
	void onGuiPublishRequest(std::string msg, bool hmacs, bool encryption);
	void onGuiInjectRequest(std::string msg, bool hmacs, bool encryption);
};
}//namespace prototype
#endif // MAINWINDOW_H

