#ifndef __RENDER_PP_RESULT_MAIN_WINDOW_H__
#define __RENDER_PP_RESULT_MAIN_WINDOW_H__
#include <string>
#include <vector>
#include <QMainWindow>
#include <QHBoxLayout>
#include <QSlider>
#include <QPushButton>
#include "PostProcessRecordWidget.h"

class PostProcessRecordMainWindow : public QMainWindow
{
    Q_OBJECT
    
signals:
	
public slots:
	void togglePlay(const bool& _toggled);
public:
    PostProcessRecordMainWindow();
    PostProcessRecordMainWindow(std::string _dir, bool _optimize=false);

protected:
	QHBoxLayout* mMainLayout;
	QPushButton* mButton;
	PostProcessRecordWidget* mMotionWidget;

	void initLayoutSetting(std::string _dir, bool _optimize);

};
#endif