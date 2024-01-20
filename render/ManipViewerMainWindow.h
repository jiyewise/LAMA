#ifndef __RENDER_MANIP_VIEWER_MAIN_WINDOW_H__
#define __RENDER_MANIP_VIEWER_MAIN_WINDOW_H__
#include <string>
#include <vector>
#include <QMainWindow>
#include <QHBoxLayout>
#include <QSlider>
#include <QPushButton>
#include <QListWidget>
#include <QTextEdit>
#include <boost/filesystem.hpp>
#include "ManipViewerWidget.h"

class ManipViewerMainWindow : public QMainWindow
{
    Q_OBJECT
    
signals:
	
public slots:
	void togglePlay(const bool& _toggled);
	void optimize();
	void optimizeIK();
	void saveMotion();
	// void saveOriginalMotion();
	// void saveTransitionRecord();
	// void saveOptimizedMotion();

public:
    ManipViewerMainWindow();
    ManipViewerMainWindow(std::string _env, std::string _manipIC, std::string _dir, std::string _saveDir);
	QTextEdit* mOptStart;
	QTextEdit* mOptEnd;
	QTextEdit* mSaveStart;
	QTextEdit* mSaveEnd;

	std::string mDir, mEnv, mManipIC, mSaveDir;
	void createDirectory(std::string _env, std::string _manipIC, std::string _dir);

protected:
	QHBoxLayout* mMainLayout;
	ManipViewerWidget* mMotionWidget;

	void initLayoutSetting(std::string _env, std::string _manipIC, std::string _dir);
};
#endif