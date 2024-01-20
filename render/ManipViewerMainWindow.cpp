#include <iostream>
#include <QtWidgets/QApplication>
#include <QFormLayout>
#include <QGLWidget>
#include <QLabel>
#include <QCheckBox>
#include <QFileDialog>
#include "ManipViewerMainWindow.h"

// namespace fs = boost::filesystem;
ManipViewerMainWindow::
ManipViewerMainWindow() :QMainWindow()
{
    setWindowTitle("Renderer");
}
ManipViewerMainWindow::
ManipViewerMainWindow(std::string _env, std::string _manipIC, std::string _dir, std::string _saveDir)
{   
    ManipViewerMainWindow();
    createDirectory(_env, _manipIC, _saveDir);
    initLayoutSetting(_env, _manipIC, _dir);
}

void 
ManipViewerMainWindow:: 
createDirectory(std::string _env, std::string _manipIC, std::string _dir)
{
    std::string savePath = LAMA_DIR + std::string("/manip_result/") + _dir + std::string("/");
    boost::filesystem::path saveDir(savePath);
    if(!boost::filesystem::exists(saveDir)) {
        boost::filesystem::create_directory(saveDir);
    }
    
    mDir = _dir + std::string("/");

    // save env
    std::string path = LAMA_DIR + std::string("/manip_result/") + mDir + std::string("/env.xml");
    std::string originalEnvPath = LAMA_DIR + std::string("/env/env_config/") + _env + std::string(".xml");

    boost::filesystem::path destPath(path);
    boost::filesystem::path sourcePath(originalEnvPath);
    if(!boost::filesystem::exists(destPath)) {
        boost::filesystem::copy_file(sourcePath, destPath);
    }

    // save manip config
    path = LAMA_DIR + std::string("/manip_result/") + mDir + std::string("/manip_ic.xml");
    std::string originalManipICPath = LAMA_DIR + std::string("/data/manip_ic/") + _manipIC + std::string(".xml");

    boost::filesystem::path destPathIC(path);
    boost::filesystem::path sourcePathIC(originalManipICPath);
    if(!boost::filesystem::exists(destPathIC)) {
    boost::filesystem::copy_file(sourcePathIC, destPathIC);
    }

    mEnv = _env;
    mManipIC = _manipIC;
    mSaveDir = savePath;
}

void
ManipViewerMainWindow::
initLayoutSetting(std::string _env, std::string _manipIC, std::string _dir) {
    mMainLayout = new QHBoxLayout();
    setMaximumSize(1600*1.3,850*1.3);
    setMinimumSize(1600*1.3,850*1.3);

    QVBoxLayout *motionlayout = new QVBoxLayout();

    mMotionWidget = new ManipViewerWidget(_env, _manipIC, _dir);
    mMotionWidget->setMinimumSize(1300*1.3,750*1.3);
    mMotionWidget->setMaximumSize(1300*1.3,750*1.3);

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

    buttonlayout->addStretch(1);

    QLabel* label = new QLabel(QString::fromStdString("start:"));
    buttonlayout->addWidget(label);

    mOptStart = new QTextEdit();
    mOptStart->setMinimumSize(50,30);
    mOptStart->setMaximumSize(50,30);
    buttonlayout->addWidget(mOptStart);

    label = new QLabel(QString::fromStdString("end:"));
    buttonlayout->addWidget(label);

    mOptEnd = new QTextEdit();
    mOptEnd->setMinimumSize(50,30);
    mOptEnd->setMaximumSize(50,30);
    buttonlayout->addWidget(mOptEnd);

    button = new QPushButton("optimize", this);
    connect(button, SIGNAL(clicked(bool)), this, SLOT(optimize())); 
    buttonlayout->addWidget(button);    

    button = new QPushButton("IK", this);
    connect(button, SIGNAL(clicked(bool)), this, SLOT(optimizeIK())); 
    buttonlayout->addWidget(button);    

    buttonlayout->addStretch(1);

    label = new QLabel(QString::fromStdString("start:"));
    buttonlayout->addWidget(label);

    mSaveStart = new QTextEdit();
    mSaveStart->setMinimumSize(50,30);
    mSaveStart->setMaximumSize(50,30);
    buttonlayout->addWidget(mSaveStart);

    label = new QLabel(QString::fromStdString("end:"));
    buttonlayout->addWidget(label);

    mSaveEnd = new QTextEdit();
    mSaveEnd->setMinimumSize(50,30);
    mSaveEnd->setMaximumSize(50,30);
    buttonlayout->addWidget(mSaveEnd);

    button = new QPushButton("save", this);
    connect(button, SIGNAL(clicked(bool)), this, SLOT(saveMotion())); 
    buttonlayout->addWidget(button);   

    // button = new QPushButton("save original", this);
    // connect(button, SIGNAL(clicked(bool)), this, SLOT(saveOriginalMotion())); 
    // buttonlayout->addWidget(button); 

    // button = new QPushButton("save transition", this);
    // connect(button, SIGNAL(clicked(bool)), this, SLOT(saveTransitionRecord())); 
    // buttonlayout->addWidget(button);    

    // button = new QPushButton("save optimized", this);
    // connect(button, SIGNAL(clicked(bool)), this, SLOT(saveOptimizedMotion())); 
    // buttonlayout->addWidget(button);    

    motionlayout->addLayout(buttonlayout);
    mMainLayout->addLayout(motionlayout);

    setCentralWidget(new QWidget());
    centralWidget()->setLayout(mMainLayout);

}

void 
ManipViewerMainWindow:: 
optimize()
{
    int start = stoi(mOptStart->toPlainText().toStdString());
    int end = stoi(mOptEnd->toPlainText().toStdString());
    mMotionWidget->optimize(start, end);   
}

void 
ManipViewerMainWindow:: 
optimizeIK()
{
    int start = stoi(mOptStart->toPlainText().toStdString());
    int end = stoi(mOptEnd->toPlainText().toStdString());
    mMotionWidget->optimizeIK(start, end);   
}

void 
ManipViewerMainWindow:: 
saveMotion()
{
    // QString fileName = QFileDialog::getSaveFileName(this, "save file", 
    //                 QString::fromStdString(std::string(LAMA_DIR) + std::string("/result/manip_result/")), "Files (*.*)");  
    // if(fileName.toStdString() == "")
    //     return;
    int start = stoi(mSaveStart->toPlainText().toStdString());
    int end = stoi(mSaveEnd->toPlainText().toStdString());
    mMotionWidget->saveMotion(mSaveDir,start, end);
}

void 
ManipViewerMainWindow::
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
