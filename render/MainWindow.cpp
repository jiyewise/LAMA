#include <iostream>
#include <QtWidgets/QApplication>
#include <QFormLayout>
#include <QGLWidget>
#include <QLabel>
#include <QCheckBox>
#include "MainWindow.h"
MainWindow::
MainWindow() :QMainWindow()
{
    setWindowTitle("Renderer");
}
MainWindow::
MainWindow(std::string _motion, std::string _ppo)
{   
    MainWindow();
    initLayoutSetting(_motion, _ppo);
}
void
MainWindow::
initLayoutSetting(std::string _motion, std::string _ppo) {
    mMainLayout = new QHBoxLayout();
    setMaximumSize(1600*1.3,850*1.3);
    setMinimumSize(1600*1.3,850*1.3);

    QVBoxLayout *motionlayout = new QVBoxLayout();

    mMotionWidget = new MotionWidget(_motion, _ppo);

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
  

    motionlayout->addLayout(buttonlayout);

    setCentralWidget(new QWidget());
    centralWidget()->setLayout(mMainLayout);
    mMainLayout->addLayout(motionlayout);

}
void 
MainWindow::
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
