#include <iostream>
#include <QtWidgets/QApplication>
#include <QFormLayout>
#include <QGLWidget>
#include <QLabel>
#include <QCheckBox>
#include "PostProcessRecordMainWindow.h"

PostProcessRecordMainWindow::
PostProcessRecordMainWindow() :QMainWindow()
{
    setWindowTitle("Renderer");
}
PostProcessRecordMainWindow::
PostProcessRecordMainWindow(std::string _dir, bool _optimize)
{   
    PostProcessRecordMainWindow();
    initLayoutSetting(_dir, _optimize);
}
void
PostProcessRecordMainWindow::
initLayoutSetting(std::string _dir, bool _optimize) {
    mMainLayout = new QHBoxLayout();
    setMaximumSize(1920*1.1,1080*1.1);
    setMinimumSize(1920*1.1,1080*1.1);

    QVBoxLayout *motionlayout = new QVBoxLayout();

    mMotionWidget = new PostProcessRecordWidget(_dir, _optimize);

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

    motionlayout->addLayout(buttonlayout);

    setCentralWidget(new QWidget());
    centralWidget()->setLayout(mMainLayout);
    mMainLayout->addLayout(motionlayout);

}
void 
PostProcessRecordMainWindow::
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

