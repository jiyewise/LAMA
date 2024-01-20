#include <string>
#include <iostream>
#include <boost/program_options.hpp>
#include <GL/glut.h>
#include <QApplication>
#include <QGLWidget>
#include "MainWindow.h"
#include "InteractionGenMainWindow.h"
#include "ActionControllerMainWindow.h"
#include "PostProcessRecordMainWindow.h"
#include "ManipViewerMainWindow.h"
#include "ManipRecordMainWindow.h"

class GLWidget : public QGLWidget{
    void initializeGL(){
        glClearColor(0.0, 1.0, 1.0, 1.0);
    }
    
    void qgluPerspective(GLdouble fovy, GLdouble aspect, GLdouble zNear, GLdouble zFar){
        const GLdouble ymax = zNear * tan(fovy * M_PI / 360.0);
        const GLdouble ymin = -ymax;
        const GLdouble xmin = ymin * aspect;
        const GLdouble xmax = ymax * aspect;
        glFrustum(xmin, xmax, ymin, ymax, zNear, zFar);
    }
    
    void resizeGL(int width, int height){
        if (height==0) height=1;
        glViewport(0,0,width,height);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        qgluPerspective(45.0f,(GLfloat)width/(GLfloat)height,0.1f,100.0f);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
     }
    
    void paintGL(){
        glMatrixMode(GL_MODELVIEW);         
        glLoadIdentity();
        glClear(GL_COLOR_BUFFER_BIT);  

        glBegin(GL_POLYGON); 
            glVertex2f(-0.5, -0.5); 
            glVertex2f(-0.5, 0.5);
            glVertex2f(0.5, 0.5); 
            glVertex2f(0.5, -0.5); 
        glEnd();
    }
};
int main(int argc,char** argv)
{
	boost::program_options::options_description desc("allowed options");
	desc.add_options()
	("bvh,b",boost::program_options::value<std::string>())
	("ppo,p",boost::program_options::value<std::string>())
	("type,t",boost::program_options::value<std::string>())
	("env,e",boost::program_options::value<std::string>())
	("dir,d",boost::program_options::value<std::string>())
	("save,s",boost::program_options::value<std::string>())
	("manip,m",boost::program_options::value<std::string>())
	("num,n",boost::program_options::value<std::string>())
    ("optimize", "optimize");
	;

	boost::program_options::variables_map vm;
	boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
	std::string bvh="", type="", env="", ppo="", dir="", manip="", num="", save="";
    bool optimize=false;
	if(vm.count("bvh")) {
		bvh = vm["bvh"].as<std::string>();
	}
	if(vm.count("type")) {
		type = vm["type"].as<std::string>();
	}
	if(vm.count("env")) {
		env = vm["env"].as<std::string>();
	}
	if(vm.count("ppo")) {
		ppo = vm["ppo"].as<std::string>();
	}
	if(vm.count("dir")) {
		dir = vm["dir"].as<std::string>();
	}
	if(vm.count("manip")) {
		manip = vm["manip"].as<std::string>();
	}
	if(vm.count("num")) {
		num = vm["num"].as<std::string>();
	}
	if(vm.count("save")) {
		save = vm["save"].as<std::string>();
	}
	if(vm.count("optimize")) {
        optimize=true;
	}

	glutInit(&argc,argv);
	QApplication a(argc, argv);
    
    // open window by type
    // type: setting and viewing interaction cue
    if (type == "int_gen") {
        InteractionGenMainWindow* main_window = new InteractionGenMainWindow(env);
        main_window->resize(2560,1440);
        main_window->show();        
    }
    else if (type == "action_control") {
        ActionControllerMainWindow* main_window = new ActionControllerMainWindow(env, ppo, dir, optimize);
        main_window->resize(2560,1440);
        main_window->show();        
    }
    else if (type == "pp_record") {
        PostProcessRecordMainWindow* main_window = new PostProcessRecordMainWindow(dir, optimize);
        main_window->resize(2560,1440);
        main_window->show();
    }
    else if (type == "manip_view") {
        ManipViewerMainWindow* main_window = new ManipViewerMainWindow(env, manip, dir, save);
        main_window->resize(2560,1440);
        main_window->show();
    }
    else if (type == "manip_record") {
        ManipRecordMainWindow* main_window = new ManipRecordMainWindow(dir);
        main_window->resize(2560,1440);
        main_window->show();
    }
    else {
        MainWindow* main_window = new MainWindow(bvh, std::string("")); // ignore ppo. only for viewing bvh
        main_window->resize(2560,1440);
        main_window->show();
    }
    
    return a.exec();

}
