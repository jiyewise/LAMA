#ifndef __RENDER_INTERACTION_GEN_MAIN_WINDOW_H__
#define __RENDER_INTERACTION_GEN_MAIN_WINDOW_H__
#include <string>
#include <vector>
#include <QMainWindow>
#include <QHBoxLayout>
#include <QSlider>
#include <QPushButton>
#include <QListWidget>
#include <QComboBox>
#include <QTextEdit>
#include "InteractionGenWidget.h"
#include "Environment.h"

class InteractionGenMainWindow : public QMainWindow
{
    Q_OBJECT
    
signals:
	
public slots:
	void togglePlay(const bool& _toggled);
	void selectPoint(QListWidgetItem* _item);
	void selectCue(QListWidgetItem* _item);
	void updatePointList();
	void jointClicked();
	void addInteractionCue();
	void addContact();
	void viewInputAndIC();
	void modifySelectedIC();
	void exportInput();
	
public:
    InteractionGenMainWindow();
    InteractionGenMainWindow(std::string _env_path);

protected:
	QHBoxLayout* mMainLayout;
	QPushButton* mButton;
	InteractionGenWidget* mInteractionGenWidget;

	QListWidget* mPointList;
	QListWidget* mJointList; 
	QListWidget* mCueList; 
	std::vector<int> mJointClicked;
	QTextEdit* mFrameForCue;

	std::string mEnvPath;
	void initLayoutSetting(std::string _env_path);

};
#endif