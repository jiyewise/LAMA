#include <iostream>
#include <QtWidgets/QApplication>
#include <QFormLayout>
#include <QGLWidget>
#include <QLabel>
#include <QCheckBox>
#include <QFileDialog>
#include "ActionControllerMainWindow.h"

// namespace fs = boost::filesystem;
ActionControllerMainWindow::
ActionControllerMainWindow() :QMainWindow()
{
    setWindowTitle("Renderer");
}
ActionControllerMainWindow::
ActionControllerMainWindow(std::string _env, std::string _ppo, std::string _dir, bool _optimize)
{   
    ActionControllerMainWindow();
    mEnv = _env;
    createDirectory(_dir);
    initLayoutSetting(_env, _ppo, _optimize);
    if(_optimize) {
        saveOptimizedMotion();
        exit(0);
    }
}

void 
ActionControllerMainWindow:: 
createDirectory(std::string _dir)
{
    std::string savePath = LAMA_DIR + std::string("/result/") + _dir + std::string("/");
    boost::filesystem::path saveDir(savePath);
    if(!boost::filesystem::exists(saveDir)) {
        boost::filesystem::create_directory(saveDir);
    }
    
    mDir = _dir + std::string("/");
    // save env
    std::string path = LAMA_DIR + std::string("/result/") + mDir + std::string("/env.xml");
    std::string originalEnvPath = LAMA_DIR + std::string("/env/env_config/") + mEnv + std::string(".xml");

    boost::filesystem::path destPath(path);
    boost::filesystem::path sourcePath(originalEnvPath);
    if(!boost::filesystem::exists(destPath)) {
        boost::filesystem::copy_file(sourcePath, destPath);
    }
}

void
ActionControllerMainWindow::
initLayoutSetting(std::string _env, std::string _ppo, bool _optimize) {
    mMainLayout = new QHBoxLayout();
    setMaximumSize(1920*1.1,1080*1.1);
    setMinimumSize(1920*1.1,1080*1.1);

    QVBoxLayout *motionlayout = new QVBoxLayout();

    mMotionWidget = new ActionControllerWidget(_env, _ppo, _optimize);
    mMotionWidget->setMinimumSize(1920,1080);
    mMotionWidget->setMaximumSize(1920,1080);

    motionlayout->addWidget(mMotionWidget);

    QHBoxLayout *buttonlayout = new QHBoxLayout();
    buttonlayout->addStretch(1);
  
    QPushButton* button = new QPushButton("reset", this);
    connect(button, SIGNAL(clicked(bool)), mMotionWidget, SLOT(reset())); 
    buttonlayout->addWidget(button); 

    button = new QPushButton("prev", this);
    connect(button, SIGNAL(clicked(bool)), mMotionWidget, SLOT(prevFrame())); 
    buttonlayout->addWidget(button); 

    button = new QPushButton("play", this);
    button->setCheckable(true);
    connect(button, SIGNAL(toggled(bool)), this, SLOT(togglePlay(const bool&))); 
    buttonlayout->addWidget(button); 

    button = new QPushButton("next", this);
    connect(button, SIGNAL(clicked(bool)), mMotionWidget, SLOT(nextFrame())); 
    buttonlayout->addWidget(button);    

    button = new QPushButton("optimize", this);
    connect(button, SIGNAL(clicked(bool)), mMotionWidget, SLOT(optimize())); 
    buttonlayout->addWidget(button);    

    button = new QPushButton("save original", this);
    connect(button, SIGNAL(clicked(bool)), this, SLOT(saveOriginalMotion())); 
    buttonlayout->addWidget(button); 

    button = new QPushButton("save action log", this);
    connect(button, SIGNAL(clicked(bool)), this, SLOT(saveActionLog())); 
    buttonlayout->addWidget(button);    

    button = new QPushButton("save optimized", this);
    connect(button, SIGNAL(clicked(bool)), this, SLOT(saveOptimizedMotion())); 
    buttonlayout->addWidget(button);    

    buttonlayout->addStretch(1);

    motionlayout->addLayout(buttonlayout);
    mMainLayout->addLayout(motionlayout);

    setCentralWidget(new QWidget());
    centralWidget()->setLayout(mMainLayout);

}
void 
ActionControllerMainWindow::
togglePlay(const bool& _toggled)
{
    auto button = qobject_cast<QPushButton*>(sender());
    if(_toggled) {
        button->setText("pause");
    } else {
        button->setText("play");
    }
    mMotionWidget->togglePlay();
}

void 
ActionControllerMainWindow:: 
saveOriginalMotion()
{
    std::string path = LAMA_DIR + std::string("/result/") + mDir + std::string("original.txt");
    mMotionWidget->saveOriginalMotion(path);
}

void 
ActionControllerMainWindow:: 
saveTransitionRecord()
{
    std::string path = LAMA_DIR + std::string("/result/") + mDir + std::string("transitionRecord.txt");
    mMotionWidget->saveTransitionRecord(path);
}

void 
ActionControllerMainWindow:: 
saveActionLog()
{
    std::string path = LAMA_DIR + std::string("/result/") + mDir + std::string("actionLog.txt");
    mMotionWidget->saveActionLog(path);
}

void 
ActionControllerMainWindow:: 
saveOptimizedMotion()
{
    std::string path = LAMA_DIR + std::string("/result/") + mDir + std::string("optimized.txt");
    mMotionWidget->saveOptimizeMotion(path);
}