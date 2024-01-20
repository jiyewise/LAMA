#include <iostream>
#include <QtWidgets/QApplication>
#include <QFormLayout>
#include <QGLWidget>
#include <QLabel>
#include <QCheckBox>
#include <QFileDialog>
#include <fstream>
#include "InteractionGenMainWindow.h"

InteractionGenMainWindow::
InteractionGenMainWindow() :QMainWindow()
{
    setWindowTitle("Renderer");
}
InteractionGenMainWindow::
InteractionGenMainWindow(std::string _env_path)
{   
    InteractionGenMainWindow();
    initLayoutSetting(_env_path);
}
void
InteractionGenMainWindow::
initLayoutSetting(std::string _env_path) {
    mMainLayout = new QHBoxLayout();
    setMaximumSize(1600,850);
    setMinimumSize(1600,850);

    QVBoxLayout *motionlayout = new QVBoxLayout();

    mInteractionGenWidget = new InteractionGenWidget(_env_path);

    mInteractionGenWidget->setMinimumSize(1300,750);
    mInteractionGenWidget->setMaximumSize(1300,750);

    motionlayout->addWidget(mInteractionGenWidget);

    QVBoxLayout* paramlayout = new QVBoxLayout();

    // show penetrating points (points to select)
    QPushButton* button = new QPushButton("Load/Update", this);
    connect(button, SIGNAL(clicked(bool)), this, SLOT(updatePointList())); // TODO 

    mPointList = new QListWidget();
    mPointList->setMinimumSize(275,100);
    mPointList->setMaximumSize(275,100);

    paramlayout->addWidget(button);
    paramlayout->addWidget(mPointList);

    // saving contact joint to create interaction cue
    mJointList = new QListWidget();
    mJointList->setMinimumSize(275,250);
    mJointList->setMaximumSize(275,250);
    mJointList->setSelectionMode(QAbstractItemView::MultiSelection);
    
    // get list of joints from dart skeleton
    std::vector<std::string> jointNames = mInteractionGenWidget->getJointNames();
    for(int i = 0; i < jointNames.size(); i++) {
        if(jointNames[i] == "Hips" || jointNames[i] == "LeftFoot" || jointNames[i] == "RightFoot")
        mJointList->addItem(QString::fromStdString(jointNames[i]));
    }
    connect(mJointList, SIGNAL(itemSelectionChanged()), this, SLOT(jointClicked()));

    // enter frame (for manip mode)
    QHBoxLayout *buttonlayout = new QHBoxLayout();

    // QLabel* label = new QLabel(QString::fromStdString("frame:"));
    // buttonlayout->addWidget(label);

    // mFrameForCue = new QTextEdit();
    // mFrameForCue->setMinimumSize(50,30);
    // mFrameForCue->setMaximumSize(50,30);
    // buttonlayout->addWidget(mFrameForCue);

    // for adding contact and cue
    paramlayout->addWidget(mJointList);
    paramlayout->addLayout(buttonlayout);

    // button = new QPushButton("Add contact", this);
    // connect(button, SIGNAL(clicked(bool)), this, SLOT(addContact())); // TODO 
    // paramlayout->addWidget(button);

    button = new QPushButton("Add cue", this);
    connect(button, SIGNAL(clicked(bool)), this, SLOT(addInteractionCue())); // TODO     
    paramlayout->addWidget(button);

    // select and view cue
    mCueList = new QListWidget();
    mCueList->setMinimumSize(275,100);
    mCueList->setMaximumSize(275,100);
    paramlayout->addWidget(mCueList);

    button = new QPushButton("Modify cue", this);
    connect(button, SIGNAL(clicked(bool)), this, SLOT(modifySelectedIC())); // TODO     
    paramlayout->addWidget(button);

    button = new QPushButton("View Initial and Selected Cue", this);
    connect(button, SIGNAL(clicked(bool)), this, SLOT(viewInputAndIC())); // TODO 
    paramlayout->addWidget(button);

    // button = new QPushButton("View Initial and All Cues", this);
    // connect(button, SIGNAL(clicked(bool)), this, SLOT(viewInputAndIC())); // TODO 
    // paramlayout->addWidget(button);

    // save cue
    button = new QPushButton("Export selected IC", this);
    connect(button, SIGNAL(clicked(bool)), this, SLOT(exportInput())); // TODO     
    paramlayout->addWidget(button);

    setCentralWidget(new QWidget());
    centralWidget()->setLayout(mMainLayout);
    mMainLayout->addLayout(motionlayout);
    mMainLayout->addLayout(paramlayout);

    mEnvPath = _env_path;
}

void 
InteractionGenMainWindow::
updatePointList()
{
    mInteractionGenWidget->updatePointList();

    mPointList->clear();
    std::vector<ENV::IntersectPoint> intersectPoints = mInteractionGenWidget->getIntersectPoints(); 
    for(int i = 0; i < intersectPoints.size(); i++) {
        mPointList->addItem(QString::fromStdString("point "+std::to_string(i)));
    }
    connect(mPointList, SIGNAL(itemDoubleClicked(QListWidgetItem*)), this, SLOT(selectPoint(QListWidgetItem*)));
    
    mInteractionGenWidget->mCurMode = std::string("view");
}

void 
InteractionGenMainWindow::
selectPoint(QListWidgetItem* _item)
{
    int selectIdx = mPointList->row(_item);
    mInteractionGenWidget->setSelectedPoint(selectIdx); 

    // debug: print location of the face center, face num, and face center offset
    // ENV::IntersectPoint p = mInteractionGenWidget->mIntersectPointList[mInteractionGenWidget->mSelectPointIdx];
    // std::cout << "obj idx: " << p.objIdx << " bn idx: " << p.bnIdx << " face num: " << p.faceNum << std::endl; 
    // std::cout << "p: " << p.pos.transpose() << " face center offset: " << p.faceCenterOffset.transpose()
    //                     << " face center: " << mInteractionGenWidget->mEnvScene->mObjectList[p.objIdx].getFaceCenter(p.bnIdx, p.faceNum).transpose() << std::endl;
}

void 
InteractionGenMainWindow::
selectCue(QListWidgetItem* _item)
{
    int selectIdx = mCueList->row(_item);
    mInteractionGenWidget->setSelectedCue(selectIdx);
}

void 
InteractionGenMainWindow::
jointClicked()
{
    QList<QListWidgetItem *> items = mJointList->selectedItems();
    mJointClicked.clear();
    for(auto i : items) {
        int idx = mJointList->row(i);
        mJointClicked.push_back(idx);
    }
}

void 
InteractionGenMainWindow::
addContact()
{
    // get frame and send to widget
    int frame;
    // if(mFrameForCue->toPlainText().toStdString().size() == 0) 
    //     frame = -1;
    // else
    //     frame = stoi(mFrameForCue->toPlainText().toStdString());
    frame = -1;
    mInteractionGenWidget->createInteractionContact(mJointClicked, frame);

}

void 
InteractionGenMainWindow:: 
viewInputAndIC()
{
    mInteractionGenWidget->viewInputAndIC(false);
}

void 
InteractionGenMainWindow:: 
modifySelectedIC()
{
    mInteractionGenWidget->modifyIC();
}

void
InteractionGenMainWindow::
addInteractionCue()
{
    // get frame and send to widget
    ENV::InteractionCue ic = mInteractionGenWidget->createInteractionCue();

    // add interaction cue to list
    if(ic.mType != "") {
        mInteractionGenWidget->addInteractionCue(ic);
    }

    mCueList->clear();
    std::vector<ENV::InteractionCue> iterCueList = mInteractionGenWidget->mInteractionCueList; 
    for(int i = 0; i < iterCueList.size(); i++) {
        mCueList->addItem(QString::fromStdString("cue "+std::to_string(i) + " type " + iterCueList[i].mType));
    }
    connect(mCueList, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(selectCue(QListWidgetItem*)));
}

void 
InteractionGenMainWindow::
exportInput()
{
    // get directory from _env_path
    boost::filesystem::path p(std::string(LAMA_DIR) + std::string("/env/env_config/") + mEnvPath);
    boost::filesystem::path dir = p.parent_path();
    
    // QString filters("Music files (*.mp3);;Text files (*.txt);;All files (*.*)");
    // QString defaultFilter("Text files (*.txt)");

    QString filters("Files (*.*)");
    QString defaultFilter("Files (*.*)");

    QString fileName = QFileDialog::getSaveFileName(this, "save file", 
                    QString::fromStdString(dir.string()), filters, &defaultFilter, QFileDialog::DontUseNativeDialog);  
    
    if(fileName.toStdString() == "")
        return;

    // save file path to widget and save
    mInteractionGenWidget->exportInput(fileName.toStdString());
}

void 
InteractionGenMainWindow::
togglePlay(const bool& _toggled)
{
    auto button = qobject_cast<QPushButton*>(sender());
    if(_toggled) {
        button->setText("pause");
    } else {
        button->setText("play");
    }
    mInteractionGenWidget->togglePlay();
}
