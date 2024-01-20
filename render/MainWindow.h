#ifndef __RENDER_MAIN_WINDOW_H__
#define __RENDER_MAIN_WINDOW_H__
#include <string>
#include <vector>
#include <QMainWindow>
#include <QHBoxLayout>
#include <QSlider>
#include <QPushButton>
#include "MotionWidget.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
signals:
	
public slots:
	void togglePlay(const bool& _toggled);
	// void saveMotion();

public:
    MainWindow();
    MainWindow(std::string _motion, std::string _ppo);
	// QTextEdit* mSaveStart;
	// QTextEdit* mSaveEnd;
	
protected:
	QHBoxLayout* mMainLayout;
	QPushButton* mButton;
	MotionWidget* mMotionWidget;

	void initLayoutSetting(std::string _motion, std::string _ppo);

};
#endif