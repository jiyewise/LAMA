// copied some functions from Dart library
// dart/gui/OpenGLRenderInterface.cpp

#include <assimp/cimport.h>
#include <iostream>
#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include "GL/glut.h"
#include "GLFunctions.h"

#define RESOLUTION 16

static GLUquadricObj *quadObj;
static void initQuadObj(void)
{
    quadObj = gluNewQuadric();
    if(!quadObj)
        // DART modified error output
        std::cerr << "OpenGL: Fatal Error in DART: out of memory." << std::endl;
}
#define QUAD_OBJ_INIT { if(!quadObj) initQuadObj(); }
void
GUI::
drawSphere(double _r)
{
    QUAD_OBJ_INIT;
    gluQuadricDrawStyle(quadObj, GLU_FILL);
    gluQuadricNormals(quadObj, GLU_SMOOTH);

    gluSphere(quadObj, _r, RESOLUTION, RESOLUTION);
}
void
DrawOpenDome(double _radius, int _slices, int _stacks)
{
  // (2pi/Stacks)
  auto pi = M_PI;
  auto drho = pi / _stacks / 2.0;
  auto dtheta = 2.0 * pi / _slices;

  auto rho = drho;
  auto srho = std::sin(rho);
  auto crho = std::cos(rho);

  // Many sources of OpenGL sphere drawing code uses a triangle fan
  // for the caps of the sphere. This however introduces texturing
  // artifacts at the poles on some OpenGL implementations
  glBegin(GL_TRIANGLE_FAN);
  glNormal3d(0.0, 0.0, _radius);
  glVertex3d(0.0, 0.0, _radius);
  for (int j = 0; j <= _slices; ++j)
  {
    auto theta = (j == _slices) ? 0.0 : j * dtheta;
    auto stheta = -std::sin(theta);
    auto ctheta = std::cos(theta);

    auto x = srho * stheta;
    auto y = srho * ctheta;
    auto z = crho;

    glNormal3d(x, y, z);
    glVertex3d(x * _radius, y * _radius, z * _radius);
  }
  glEnd();

  for (int i = 1; i < _stacks; ++i)
  {
    auto rho = i * drho;
    auto srho = std::sin(rho);
    auto crho = std::cos(rho);
    auto srhodrho = std::sin(rho + drho);
    auto crhodrho = std::cos(rho + drho);

    // Many sources of OpenGL sphere drawing code uses a triangle fan
    // for the caps of the sphere. This however introduces texturing
    // artifacts at the poles on some OpenGL implementations
    glBegin(GL_TRIANGLE_STRIP);

    for (int j = 0; j <= _slices; ++j)
    {
      auto theta = (j == _slices) ? 0.0 : j * dtheta;
      auto stheta = -std::sin(theta);
      auto ctheta = std::cos(theta);

      auto x = srho * stheta;
      auto y = srho * ctheta;
      auto z = crho;

      glNormal3d(x, y, z);
      glVertex3d(x * _radius, y * _radius, z * _radius);

      x = srhodrho * stheta;
      y = srhodrho * ctheta;
      z = crhodrho;

      glNormal3d(x, y, z);
      glVertex3d(x * _radius, y * _radius, z * _radius);
    }
    glEnd();
  }
}
void
GUI::
drawCapsule(double _radius, double _height)
{
    GLint slices = RESOLUTION;
    GLint stacks = RESOLUTION;

    // Graphics assumes Cylinder is centered at CoM
    // gluCylinder places base at z = 0 and top at z = height
    glTranslated(0.0, 0.0, -0.5*_height);

    // Code taken from glut/lib/glut_shapes.c
    QUAD_OBJ_INIT;
    gluQuadricDrawStyle(quadObj, GLU_FILL);
    gluQuadricNormals(quadObj, GLU_SMOOTH);

    gluCylinder(quadObj, _radius, _radius, _height, slices, stacks); //glut/lib/glut_shapes.c

    // Upper hemisphere
    glTranslated(0.0, 0.0, _height);
    DrawOpenDome(_radius, slices, stacks);

    // Lower hemisphere
    glTranslated(0.0, 0.0, -_height);
    glRotated(180.0, 0.0, 1.0, 0.0);
    DrawOpenDome(_radius, slices, stacks);
}
void
GUI::
drawCylinder(double _radius, double _height)
{
    GLint slices = RESOLUTION;
    GLint stacks = RESOLUTION;

    // Graphics assumes Cylinder is centered at CoM
    // gluCylinder places base at z = 0 and top at z = height
    glTranslated(0.0, 0.0, -0.5*_height);

    // Code taken from glut/lib/glut_shapes.c
    QUAD_OBJ_INIT;
    gluQuadricDrawStyle(quadObj, GLU_FILL);
    gluQuadricNormals(quadObj, GLU_SMOOTH);

    gluCylinder(quadObj, _radius, _radius, _height, slices, stacks); //glut/lib/glut_shapes.c
}
void
GUI::
drawCube(Eigen::Vector3d _size)
{
    glScaled(_size(0), _size(1), _size(2));

    // Code taken from glut/lib/glut_shapes.c
    static GLfloat n[6][3] =
    {
        {-1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {1.0, 0.0, 0.0},
        {0.0, -1.0, 0.0},
        {0.0, 0.0, 1.0},
        {0.0, 0.0, -1.0}
    };
    static GLfloat vn[8][3] =
    {
        {-1.0/3.0, -1.0/3.0, -1.0/3.0},
        {-1.0/3.0, -1.0/3.0, 1.0/3.0},
        {-1.0/3.0, 1.0/3.0, 1.0/3.0},
        {-1.0/3.0, 1.0/3.0, -1.0/3.0},
        {1.0/3.0, -1.0/3.0, -1.0/3.0},
        {1.0/3.0, -1.0/3.0, 1.0/3.0},
        {1.0/3.0, 1.0/3.0, 1.0/3.0},
        {1.0/3.0, 1.0/3.0, -1.0/3.0}
    };
    static GLint faces[6][4] =
    {
        {0, 1, 2, 3},
        {3, 2, 6, 7},
        {7, 6, 5, 4},
        {4, 5, 1, 0},
        {5, 6, 2, 1},
        {7, 4, 0, 3}
    };
    GLfloat v[8][3];
    GLint i;
    GLfloat size = 1;

    v[0][0] = v[1][0] = v[2][0] = v[3][0] = -size / 2;
    v[4][0] = v[5][0] = v[6][0] = v[7][0] = size / 2;
    v[0][1] = v[1][1] = v[4][1] = v[5][1] = -size / 2;
    v[2][1] = v[3][1] = v[6][1] = v[7][1] = size / 2;
    v[0][2] = v[3][2] = v[4][2] = v[7][2] = -size / 2;
    v[1][2] = v[2][2] = v[5][2] = v[6][2] = size / 2;

    for (i = 5; i >= 0; i--) {
        glBegin(GL_QUADS);
        glNormal3fv(&n[i][0]);
        glVertex3fv(&v[faces[i][0]][0]);
        glVertex3fv(&v[faces[i][1]][0]);
        glVertex3fv(&v[faces[i][2]][0]);
        glVertex3fv(&v[faces[i][3]][0]);
        glEnd();
    }
}
void
GUI::
drawTriangle(Eigen::Vector3d _p0, Eigen::Vector3d _p1, Eigen::Vector3d _p2, Eigen::Vector3d _color)
{
    glColor3f(_color[0], _color[1], _color[2]);
    glBegin(GL_TRIANGLES);
    glVertex3f(_p0[0], _p0[1], _p0[2]);
    glVertex3f(_p1[0], _p1[1], _p1[2]);
    glVertex3f(_p2[0], _p2[1], _p2[2]);
    glEnd();
}
void
GUI::
drawLine(Eigen::Vector3d _p0, Eigen::Vector3d _p1, Eigen::Vector3d _color)
{
    glColor3f(_color[0], _color[1], _color[2]);
    glBegin(GL_LINES);
    glNormal3f(0.0, 1.0, 0.0);
    glVertex3f(_p0[0], _p0[1], _p0[2]);
    glVertex3f(_p1[0], _p1[1], _p1[2]);
    glEnd();
}
void
GUI::
drawPoint(Eigen::Vector3d _p0, Eigen::Vector3d _color, double _scale)
{
    glPointSize(_scale);
    glColor3f(_color[0], _color[1], _color[2]);
    glBegin(GL_POINTS);
    glNormal3f(0.0, 1.0, 0.0);
    glVertex3f(_p0[0], _p0[1], _p0[2]);
    glEnd();
}
void 
GUI::
drawRoundedBox(Eigen::Vector3d _size, double _radius){
    drawRoundedBoxPlanes(_size, _radius);

    drawRoundedBoxCylinder(Eigen::Vector2d(_size[0], _size[2]), _size[1] - _radius * 2, _radius);
    glPushMatrix();
    glRotated(90, 0, 0, 1);
    drawRoundedBoxCylinder(Eigen::Vector2d(_size[1], _size[2]), _size[0] - _radius * 2, _radius);
    glPopMatrix();
    glPushMatrix();
    glRotated(90, 1, 0, 0);
    drawRoundedBoxCylinder(Eigen::Vector2d(_size[0], _size[1]), _size[2] - _radius * 2, _radius);
    glPopMatrix();

    drawRoundedBoxSphere(_size, _radius);
}
void
drawOpenDomeQuater(double _radius, int _slices, int _stacks)
{
    // (2pi/Stacks)
    auto pi = M_PI;
    auto drho = pi / _stacks / 2.0; // x, y
    auto dtheta = 0.5 * pi / _slices; // z

    auto rho = drho;
    auto srho = std::sin(rho);
    auto crho = std::cos(rho);

    // Many sources of OpenGL sphere drawing code uses a triangle fan
    // for the caps of the sphere. This however introduces texturing
    // artifacts at the poles on some OpenGL implementations
    glBegin(GL_TRIANGLE_FAN);
    glNormal3d(0.0, 0.0, _radius);
    glVertex3d(0.0, 0.0, _radius);
    for (int j = 0; j <= _slices; ++j)
    {
        auto theta = j * dtheta;
        auto stheta = -std::sin(theta);
        auto ctheta = std::cos(theta);

        auto x = srho * stheta;
        auto y = srho * ctheta;
        auto z = crho;

        glNormal3d(x, y, z);
        glVertex3d(x * _radius, y * _radius, z * _radius);
    }
    glEnd();

    for (int i = 1; i < _stacks; ++i)
    {
        auto rho = i * drho;
        auto srho = std::sin(rho);
        auto crho = std::cos(rho);
        auto srhodrho = std::sin(rho + drho);
        auto crhodrho = std::cos(rho + drho);

        // Many sources of OpenGL sphere drawing code uses a triangle fan
        // for the caps of the sphere. This however introduces texturing
        // artifacts at the poles on some OpenGL implementations
        glBegin(GL_TRIANGLE_STRIP);

        for (int j = 0; j <= _slices; ++j)
        {
            auto theta = j * dtheta;
            auto stheta = -std::sin(theta);
            auto ctheta = std::cos(theta);

            auto x = srho * stheta;
            auto y = srho * ctheta;
            auto z = crho;

            glNormal3d(x, y, z);
            glVertex3d(x * _radius, y * _radius, z * _radius);

            x = srhodrho * stheta;
            y = srhodrho * ctheta;
            z = crhodrho;

            glNormal3d(x, y, z);
            glVertex3d(x * _radius, y * _radius, z * _radius);
        }
        glEnd();
    }
}
void GUI::drawRoundedBoxSphere(Eigen::Vector3d _size, double _radius){
    GLfloat v[8][3];

    v[0][0] = v[1][0] = v[2][0] = v[3][0] = -_size[0] / 2 + _radius;
    v[4][0] = v[5][0] = v[6][0] = v[7][0] = _size[0] / 2 - _radius;
    v[0][1] = v[1][1] = v[4][1] = v[5][1] = -_size[1] / 2 + _radius;
    v[2][1] = v[3][1] = v[6][1] = v[7][1] = _size[1] / 2 - _radius;
    v[0][2] = v[3][2] = v[4][2] = v[7][2] = -_size[2] / 2 + _radius;
    v[1][2] = v[2][2] = v[5][2] = v[6][2] = _size[2] / 2 - _radius;


    glPushMatrix();
    glTranslated(v[0][0], v[0][1], v[0][2]);
    glRotated(90, 0, 0, 1);
    glRotated(270, 0, 1, 0);
    drawOpenDomeQuater(_radius, RESOLUTION, RESOLUTION);
    glPopMatrix();

    glPushMatrix();
    glTranslated(v[1][0], v[1][1], v[1][2]);
    glRotated(90, 0, 0, 1);
    drawOpenDomeQuater(_radius, RESOLUTION, RESOLUTION);
    glPopMatrix();

    glPushMatrix();
    glTranslated(v[2][0], v[2][1], v[2][2]);
    drawOpenDomeQuater(_radius, RESOLUTION, RESOLUTION);
    glPopMatrix();

    glPushMatrix();
    glTranslated(v[3][0], v[3][1], v[3][2]);
    glRotated(-90, 0, 1, 0);
    drawOpenDomeQuater(_radius, RESOLUTION, RESOLUTION);
    glPopMatrix();

    glPushMatrix();
    glTranslated(v[4][0], v[4][1], v[4][2]);
    glRotated(180, 0, 0, 1);
    glRotated(-90, 0, 1, 0);
    drawOpenDomeQuater(_radius, RESOLUTION, RESOLUTION);
    glPopMatrix();

    glPushMatrix();
    glTranslated(v[5][0], v[5][1], v[5][2]);
    glRotated(180, 0, 0, 1);
    drawOpenDomeQuater(_radius, RESOLUTION, RESOLUTION);
    glPopMatrix();

    glPushMatrix();
    glTranslated(v[6][0], v[6][1], v[6][2]);
    glRotated(90, 0, 1, 0);
    drawOpenDomeQuater(_radius, RESOLUTION, RESOLUTION);
    glPopMatrix();

    glPushMatrix();
    glTranslated(v[7][0], v[7][1], v[7][2]);
    glRotated(180, 0, 1, 0);
    drawOpenDomeQuater(_radius, RESOLUTION, RESOLUTION);
    glPopMatrix();
}
void drawRoundedBoxCylinderByParts(double _height, double _radius, double _startDegree, double _endDegree, int _slides){
    glScaled(_radius, _height, _radius);

    double gap = (_endDegree - _startDegree) / _slides;

    for (int i = 0; i < _slides; i++){
        double t1 = (_startDegree + i * gap) * M_PI / 180.0;
        double t2 = (_startDegree + (i + 1) * gap) * M_PI / 180.0;

        double x0 = 1.0 * cos(t1);
        double z0 = 1.0 * sin(t1);

        double x1 = 1.0 * cos(t2);
        double z1 = 1.0 * sin(t2);

        glBegin(GL_QUADS);
        glNormal3f(x0, 0, z0);
        glVertex3f(x0, 0.5, z0);
        glNormal3f(x0, 0, z0);
        glVertex3f(x0, -0.5, z0);
        glNormal3f(x1, 0, z1);
        glVertex3f(x1, -0.5, z1);
        glNormal3f(x1, 0, z1);
        glVertex3f(x1, 0.5, z1);

        glEnd();
    }
}
void GUI::drawRoundedBoxCylinder(Eigen::Vector2d _size, double _height, double _radius){
    glPushMatrix();
    glTranslated(_size[0]/2 - _radius, 0, _size[1]/2 - _radius);
    drawRoundedBoxCylinderByParts(_height, _radius, 90, 0, RESOLUTION);
    glPopMatrix();

    glPushMatrix();
    glTranslated(_size[0]/2 - _radius, 0, -_size[1]/2 + _radius);
    drawRoundedBoxCylinderByParts(_height, _radius, 0, -90, RESOLUTION);
    glPopMatrix();

    glPushMatrix();
    glTranslated(-_size[0]/2 + _radius, 0, -_size[1]/2 + _radius);
    drawRoundedBoxCylinderByParts(_height, _radius, -90, -180, RESOLUTION);
    glPopMatrix();

    glPushMatrix();
    glTranslated(-_size[0]/2 + _radius, 0, _size[1]/2 - _radius);
    drawRoundedBoxCylinderByParts(_height, _radius, -180, -270, RESOLUTION);
    glPopMatrix();
}
void moveToCenter(GLfloat _from[3], GLfloat _to[3], double _radius, GLfloat _res[3]) {
    GLfloat vec[3];
    for (int i = 0; i < 3; i++) vec[i] = _to[i] - _from[i];
    GLfloat minV = abs(vec[0]);
    int minIdx = 0;
    for (int i = 1; i < 3; i++) {
        if (abs(vec[i]) < minV) {
            minV = abs(vec[i]);
            minIdx = i;
        }
    }

    for (int i = 0; i < 3; i++) {
        if (minIdx == i) vec[i] = 0;
        else {
            if (vec[i] > 0) vec[i] = 1;
            else vec[i] = -1;
        }
        vec[i] *= _radius;
    }
    for (int i = 0; i < 3; i++) {
        _res[i] = _from[i] + vec[i];
    }
}

void GUI::drawRoundedBoxPlanes(Eigen::Vector3d _size, double _radius)
{
    // Code taken from glut/lib/glut_shapes.c
    static GLfloat n[6][3] =
            {
                    {-1.0, 0.0, 0.0},
                    {0.0, 1.0, 0.0},
                    {1.0, 0.0, 0.0},
                    {0.0, -1.0, 0.0},
                    {0.0, 0.0, 1.0},
                    {0.0, 0.0, -1.0}
            };
    static GLfloat vn[8][3] =
            {
                    {-1.0/3.0, -1.0/3.0, -1.0/3.0},
                    {-1.0/3.0, -1.0/3.0, 1.0/3.0},
                    {-1.0/3.0, 1.0/3.0, 1.0/3.0},
                    {-1.0/3.0, 1.0/3.0, -1.0/3.0},
                    {1.0/3.0, -1.0/3.0, -1.0/3.0},
                    {1.0/3.0, -1.0/3.0, 1.0/3.0},
                    {1.0/3.0, 1.0/3.0, 1.0/3.0},
                    {1.0/3.0, 1.0/3.0, -1.0/3.0}
            };
    static GLint faces[6][4] =
            {
                    {0, 1, 2, 3},
                    {3, 2, 6, 7},
                    {7, 6, 5, 4},
                    {4, 5, 1, 0},
                    {5, 6, 2, 1},
                    {7, 4, 0, 3}
            };

    GLfloat faceCenter[6][3] =
            {
                    {-(GLfloat)(_size[0] / 2.0), 0.0, 0.0},
                    {0.0, (GLfloat)(_size[1] / 2.0), 0.0},
                    {(GLfloat)(_size[0] / 2.0), 0.0, 0.0},
                    {0.0, -(GLfloat)(_size[1] / 2.0), 0.0},
                    {0.0, 0.0, (GLfloat)(_size[2] / 2.0)},
                    {0.0, 0.0, -(GLfloat)(_size[2] / 2.0)}
            };

    GLfloat v[8][3];
    GLint i;

    v[0][0] = v[1][0] = v[2][0] = v[3][0] = -_size[0] / 2;
    v[4][0] = v[5][0] = v[6][0] = v[7][0] = _size[0] / 2;
    v[0][1] = v[1][1] = v[4][1] = v[5][1] = -_size[1] / 2;
    v[2][1] = v[3][1] = v[6][1] = v[7][1] = _size[1] / 2;
    v[0][2] = v[3][2] = v[4][2] = v[7][2] = -_size[2] / 2;
    v[1][2] = v[2][2] = v[5][2] = v[6][2] = _size[2] / 2;

    for (i = 5; i >= 0; i--) {
        glBegin(GL_QUADS);
        glNormal3fv(&n[i][0]);
        GLfloat _v[3];
        moveToCenter(v[faces[i][0]], faceCenter[i], _radius, _v);
        glVertex3fv(&_v[0]);
        moveToCenter(v[faces[i][1]], faceCenter[i], _radius, _v);
        glVertex3fv(&_v[0]);
        moveToCenter(v[faces[i][2]], faceCenter[i], _radius, _v);
        glVertex3fv(&_v[0]);
        moveToCenter(v[faces[i][3]], faceCenter[i], _radius, _v);
        glVertex3fv(&_v[0]);
        glEnd();
    }
}
void
GUI::
drawBezierCurve(
    Eigen::Vector3d _p0,
    Eigen::Vector3d _p1,
    Eigen::Vector3d _p2,
    Eigen::Vector3d _color)
{
    glColor3f(_color[0], _color[1], _color[2]);
    glBegin(GL_LINE_STRIP);
    for(double s = 0; s <= 1.0; s += 0.05)
    {
        Eigen::Vector3d p = 
            _p0 * (1-s) * (1-s) +
            _p1 * 2 * s * (1-s) +
            _p2 *s * s;

        glVertex3f(p[0],p[1],p[2]);
    }
    glEnd();
}
// void 
// GUI::
// drawTrajectory(std::vector<Eigen::Vector3d> _points, int _idx, Eigen::Vector3d _color, bool _line)
// {
//     if(_line) {
//         for(int i = std::max(0, _idx - 60); i < _idx; i += 2) {
//             if(_points.size() <= 2 || i + 2 >= _points.size()) return;
//             else drawBezierCurve(_points[i], _points[i+1], _points[i+2], _color);
//         }
//     } else {
//         for(int i = 0; i < _idx; i++) {
//             GUI::drawPoint(_points[i], _color, 10);
//         }
//     }
// }

void 
GUI::
drawTrajectory(std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> _points, Eigen::Vector3d _color)
{
    for(int i = 0; i < _points.first.size(); i++) {
       glPushMatrix();
       glTranslated(_points.first[i](0), _points.first[i](1), _points.first[i](2));
       glColor3f(_color[0], _color[1], _color[2]);
       DrawOpenDome(0.05, 30, 30);
       glPopMatrix();

       Eigen::Vector3d p0 = _points.first[i];
       Eigen::Vector3d p1 = _points.first[i] + 0.25 * _points.second[i];

       p0(1) = 0.05;
       p1(1) = 0.05;
       drawLine(p0, p1, _color);
    }
}

void
GUI::
drawStringOnScreen(float _x, float _y, std::string _s, bool _bigFont, Eigen::Vector3d _color)
{
    glColor3f(_color[0], _color[1], _color[2]);
    
    // draws text on the screen
    GLint oldMode;
    glGetIntegerv(GL_MATRIX_MODE, &oldMode);
    glMatrixMode(GL_PROJECTION);

    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0.0, 1.0, 0.0, 1.0);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glRasterPos2f(_x, _y);
    unsigned int length = _s.length();
    for (unsigned int c = 0; c < length; c++) {
    if (_bigFont)
      glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, _s.at(c) );
    else
      glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, _s.at(c) );
    }  
    glPopMatrix();

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(oldMode);
}

void 
GUI::
drawGround(int _comX, int _comZ, double _groundHeight)
{
    float groundShininess[] = {128.0};
    float groundSpecular[]  = {0.0, 0.0, 0.0, 0.};
    float groundDiffuse[]   = {0.4, 0.4, 0.4, 0.36};
    float groundAmbient[]  = {0.4, 0.4, 0.4, 0.36};

    glEnable(GL_LIGHTING);
    // glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_LINE_SMOOTH);

    double radius = 50;
    double radius2 = radius+2;
    double numPieces = 1;
    double len = 1.0 / numPieces;
    for(int x =- radius2; x <= radius2; x += 1){
        for(int z =- radius2; z <= radius2; z += 1){
                glColor3f(0.9, 0.9, 0.9);
                glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, groundShininess);
                glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  groundSpecular);
                glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,   groundDiffuse);
                glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT,   groundAmbient);
                glBegin(GL_QUADS);
                    glNormal3f(0.0, 1.0, 0.0);
                    glVertex3f(x+_comX,_groundHeight,z+_comZ);
                    glVertex3f(x+_comX,_groundHeight,z+_comZ+1);
                    glVertex3f(x+_comX+1,_groundHeight,z+_comZ+1);
                    glVertex3f(x+_comX+1,_groundHeight,z+_comZ);
                glEnd();
                glColor3f(0.7, 0.7, 0.7);
                glBegin(GL_LINE_STRIP);
                    glVertex3f(x+_comX,_groundHeight+0.01,z+_comZ);
                    glVertex3f(x+_comX,_groundHeight+0.01,z+_comZ+1);
                    glVertex3f(x+_comX+1,_groundHeight+0.01,z+_comZ+1);
                    glVertex3f(x+_comX+1,_groundHeight+0.01,z+_comZ);
                glEnd();
                glColor3f(0.9, 0.9, 0.9);
           
        }   
    }
    float groundSpecular2[]  = {0.0, 0.0, 0.0, 0.0};
    float groundDiffuse2[]   = {0.0, 0.0, 0.0, 1.0};
    float groundAmbient2[]  = {0.0, 0.0, 0.0, 1.0};

    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  groundSpecular2);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,   groundDiffuse2);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT,   groundAmbient2);
    glColor3f(1.0, 1.0, 1.0);


}

void color4_to_float4(const aiColor4D* c, float f[4])
{
  f[0] = c->r;
  f[1] = c->g;
  f[2] = c->b;
  f[3] = c->a;
}

void set_float4(
    float f[4], float a, float b, float c, float d)
{
  f[0] = a;
  f[1] = b;
  f[2] = c;
  f[3] = d;
}

// This function is taken from the examples coming with assimp
void applyMaterial(const struct aiMaterial* mtl)
{
  float c[4];

  GLenum fill_mode;
  int ret1;
  aiColor4D diffuse;
  aiColor4D specular;
  aiColor4D ambient;
  aiColor4D emission;
  float shininess, strength;
  int two_sided;
  int wireframe;
  unsigned int max;

  set_float4(c, 0.8f, 0.8f, 0.8f, 1.0f);
  if (AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_DIFFUSE, &diffuse))
    color4_to_float4(&diffuse, c);
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, c);

  set_float4(c, 0.0f, 0.0f, 0.0f, 1.0f);
  if (AI_SUCCESS
      == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_SPECULAR, &specular))
    color4_to_float4(&specular, c);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, c);

  set_float4(c, 0.2f, 0.2f, 0.2f, 1.0f);
  if (AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_AMBIENT, &ambient))
    color4_to_float4(&ambient, c);
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, c);

  set_float4(c, 0.0f, 0.0f, 0.0f, 1.0f);
  if (AI_SUCCESS
      == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_EMISSIVE, &emission))
    color4_to_float4(&emission, c);
  glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, c);

  max = 1;
  ret1 = aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS, &shininess, &max);
  if (ret1 == AI_SUCCESS)
  {
    max = 1;
    const int ret2 = aiGetMaterialFloatArray(
        mtl, AI_MATKEY_SHININESS_STRENGTH, &strength, &max);
    if (ret2 == AI_SUCCESS)
      glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess * strength);
    else
      glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
  }
  else
  {
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 0.0f);
    set_float4(c, 0.0f, 0.0f, 0.0f, 0.0f);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, c);
  }

  max = 1;
  if (AI_SUCCESS
      == aiGetMaterialIntegerArray(
             mtl, AI_MATKEY_ENABLE_WIREFRAME, &wireframe, &max))
    fill_mode = wireframe ? GL_LINE : GL_FILL;
  else
    fill_mode = GL_FILL;
  glPolygonMode(GL_FRONT_AND_BACK, fill_mode);

  max = 1;
  if ((AI_SUCCESS
       == aiGetMaterialIntegerArray(mtl, AI_MATKEY_TWOSIDED, &two_sided, &max))
      && two_sided)
    glEnable(GL_CULL_FACE);
  else
    glDisable(GL_CULL_FACE);
}

// This function is taken from the examples coming with assimp
void 
recursiveRender(
    const struct aiScene* sc, const struct aiNode* nd, const Eigen::Vector4d& color, bool _useMeshColor)
{
  unsigned int i;
  unsigned int n = 0, t;
  aiMatrix4x4 m = nd->mTransformation;

  // update transform
  aiTransposeMatrix4(&m);
  glPushMatrix();
  glMultMatrixf((float*)&m);

  // draw all meshes assigned to this node
  for (; n < nd->mNumMeshes; ++n)
  {
    const struct aiMesh* mesh = sc->mMeshes[nd->mMeshes[n]];

    glPushAttrib(GL_POLYGON_BIT | GL_LIGHTING_BIT); // for applyMaterial()
    if (mesh->mMaterialIndex
        != (unsigned int)(-1)) // -1 is being used by us to indicate no material
      applyMaterial(sc->mMaterials[mesh->mMaterialIndex]);

    if (mesh->mNormals == nullptr)
    {
      glDisable(GL_LIGHTING);
    }
    else
    {
      glEnable(GL_LIGHTING);
    }

    for (t = 0; t < mesh->mNumFaces; ++t)
    {
      const struct aiFace* face = &mesh->mFaces[t];
      GLenum face_mode;

      switch (face->mNumIndices)
      {
        case 1:
          face_mode = GL_POINTS;
          break;
        case 2:
          face_mode = GL_LINES;
          break;
        case 3:
          face_mode = GL_TRIANGLES;
          break;
        default:
          face_mode = GL_POLYGON;
          break;
      }
      glBegin(face_mode);

      for (i = 0; i < face->mNumIndices; i++)
      {
        int index = face->mIndices[i];
        if(_useMeshColor) {
          if (mesh->mColors[0] != nullptr) {
            glColor4fv((GLfloat*)&mesh->mColors[0][index]);
          }
          else 
            glColor4f(color[0], color[1], color[2], color[3]);
        }
        else {
          glColor4f(color[0], color[1], color[2], color[3]);
        }
        // if (mesh->mColors[0] != nullptr) {
          // glColor4f(color[0], color[1], color[2], color[3]);
        // }
        if (mesh->mNormals != nullptr)
          glNormal3fv(&mesh->mNormals[index].x);
        glVertex3fv(&mesh->mVertices[index].x);
      }

      glEnd();
    }

    glPopAttrib(); // for applyMaterial()
  }

  // draw all children
  for (n = 0; n < nd->mNumChildren; ++n)
  {
    recursiveRender(sc, nd->mChildren[n], color, _useMeshColor);
  }

  glPopMatrix();
}

void
GUI::
drawMesh(const Eigen::Vector3d& scale, const aiScene* mesh, const Eigen::Vector4d& color, bool _useMeshColor)
{
 if (!mesh) {
  return;
 }
  // glColor3f(color[0],color[1],color[2]);
  glPushMatrix();

  glScaled(scale[0], scale[1], scale[2]);
  recursiveRender(mesh, mesh->mRootNode, color, _useMeshColor);

  glPopMatrix();
}

void
GUI::
drawGrid(Eigen::MatrixXd _grid, Eigen::MatrixXd _bbox, double _height)
{
  int rows = _grid.rows();
  int cols = _grid.cols();
  
  double gridsize_x, gridsize_z;
  gridsize_x = (_bbox(1, 0) - _bbox(0, 0)) / (double) rows;
  gridsize_z = (_bbox(1, 2) - _bbox(0, 2)) / (double) cols;

  // exit(0);
  glColor4f(123./255., 123./255., 123./255., 1.0);

  double x_base, z_base;
  for(int i = 0; i < rows; i++) {
    x_base = _bbox(0,0) + i*gridsize_x;
    for(int j = 0; j < cols; j++) { 
      z_base = _bbox(0,2) + j*gridsize_z;
      if(_grid(i, j) > 0.5) {
        glBegin(GL_QUADS);
          glVertex3f(x_base, _height, z_base);
          glVertex3f(x_base, _height, z_base + gridsize_z);
          glVertex3f(x_base + gridsize_x, _height, z_base + gridsize_z);
          glVertex3f(x_base + gridsize_x, _height, z_base);
        glEnd();
      } 
    }
  }

  // glColor4f(80./255., 80./255., 200./255., 1.0);
  // // draw base
  // glBegin(GL_QUADS);
  //   glVertex3f(_bbox(0, 0), 0.015, _bbox(0, 2));
  //   glVertex3f(_bbox(1, 0), 0.015, _bbox(0, 2));
  //   glVertex3f(_bbox(1, 0), 0.015, _bbox(1, 2));
  //   glVertex3f(_bbox(0, 0), 0.015, _bbox(1, 2));
  // glEnd();

}

void
GUI::
drawPathInGrid(std::vector<std::pair<int, int>> _path, Eigen::MatrixXd _bbox, double _gridsize_x, double _gridsize_z, double _height)
{
  glColor4f(200./255., 73./255., 73./255., 1.0);
  double x_base, z_base;
  for(std::pair<int, int> p : _path) {
    x_base = _bbox(0,0) + p.first * _gridsize_x;
    z_base = _bbox(0,2) + p.second * _gridsize_z;
    glBegin(GL_QUADS);
      glVertex3f(x_base, _height, z_base);
      glVertex3f(x_base, _height, z_base + _gridsize_z);
      glVertex3f(x_base + _gridsize_x, _height, z_base + _gridsize_z);
      glVertex3f(x_base + _gridsize_x, _height, z_base);
    glEnd();
  }
}