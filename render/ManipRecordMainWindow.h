#ifndef __RENDER_MANIP_RECORD_MAIN_WINDOW_H__
#define __RENDER_MANIP_RECORD_MAIN_WINDOW_H__
#include <string>
#include <vector>
#include <QMainWindow>
#include <QHBoxLayout>
#include <QSlider>
#include <QPushButton>
#include "ManipRecordWidget.h"

class ManipRecordMainWindow : public QMainWindow
{
    Q_OBJECT
    
signals:
	
public slots:
	void togglePlay(const bool& _toggled);
public:
    ManipRecordMainWindow();
    ManipRecordMainWindow(std::string _dir);

protected:
	QHBoxLayout* mMainLayout;
	QPushButton* mButton;
	ManipRecordWidget* mMotionWidget;

	void initLayoutSetting(std::string _dir);

};
#endif