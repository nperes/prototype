#ifndef MAINWINDOW_H
#define MAINWINDOW_H


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
	void onNewLegitimateMsg(std::string msg);
	void onNewCapturedMsg(std::string msg);
	void errorString(QString);

signals:
	void publish(std::string msg);
	void inject(std::string msg);

private:
	Ui::MainWindow *ui;
	int init_argc;
	char **init_argv;
	void appendToTextEdit(QTextEdit *qTextEdit, std::string msg);
	void onGuiPublishRequest(std::string msg);
	void onGuiInjectRequest(std::string msg);
};
}//namespace prototype
#endif // MAINWINDOW_H

