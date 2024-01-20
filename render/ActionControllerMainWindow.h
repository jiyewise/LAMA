#ifndef __RENDER_AC_MAIN_WINDOW_H__
#define __RENDER_AC_MAIN_WINDOW_H__
#include <string>
#include <vector>
#include <QMainWindow>
#include <QHBoxLayout>
#include <QSlider>
#include <QPushButton>
#include <QListWidget>
#include <boost/filesystem.hpp>
#include "ActionControllerWidget.h"

class ActionControllerMainWindow : public QMainWindow
{
    Q_OBJECT
    
signals:
	
public slots:
	void togglePlay(const bool& _toggled);
	void saveOriginalMotion();
	void saveTransitionRecord();
	void saveOptimizedMotion();
	void saveActionLog();

public:
    ActionControllerMainWindow();
    ActionControllerMainWindow(std::string _env, std::string _ppo, std::string _dir, bool _optimize=false);
	std::string mDir;
	std::string mEnv;
	void createDirectory(std::string _dir);
protected:
	QHBoxLayout* mMainLayout;
	ActionControllerWidget* mMotionWidget;

	void initLayoutSetting(std::string _env, std::string _ppo, bool _optimize);
};
#endif