/*
 *
 * Alice AR Engine
 *
 * @author 	: keith@robot9.me
 * @date	: 2016/9/20
 *
 */

#include "ArViewer.h"
#include "Config.h"

//#define AR_DRAW_CUBE

namespace Alice {

    int arWindow;
    cv::Mat image;
    bool *quitFlag = NULL;

    double projectMat[16];
    double modelViewMat[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

    std::mutex drawableMutex;
    std::vector<Drawable *> drawableItems;

    void Drawable::draw() {
        glPushMatrix();
        glMultMatrixd(glMat);
        drawContent();
        glPopMatrix();
    }

    void Drawable::drawContent() {
#ifdef AR_DRAW_CUBE
        glColor3d(1.0, 0.0, 0.0);
        glLineWidth(4);
        glScaled(width, height, 0.5f);
        glTranslated(0, 0, -0.5f);
        glutWireCube(1.0f);
#else
        glColor3d(0.0, 0.0, 1.0);
        glRotated(90, -1.0, 0.0, 0.0);
        glutWireTeapot(0.3);
#endif
    }

    void ArViewer::drawAr() {
        glutSetWindow(arWindow);
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

        drawVideo();

        glMatrixMode(GL_PROJECTION);
        glLoadMatrixd(projectMat);

        glMatrixMode(GL_MODELVIEW);
        glLoadMatrixd(modelViewMat);

        drawContent();

        glutSwapBuffers();
        glutPostRedisplay();
    }

    void ArViewer::drawContent() {
        drawableMutex.lock();
        for (unsigned i = 0; i < drawableItems.size(); ++i) {
            if (drawableItems[i]->isVisible()) {
                drawableItems[i]->draw();
            }
        }
        drawableMutex.unlock();
    }

    void ArViewer::start(int argc, char **argv, int w, int h) {
        glutInit(&argc, argv);
        glutInitDisplayMode(GLUT_DEPTH | GLUT_RGB | GLUT_DOUBLE);
        glutInitWindowSize(w, h);

        arWindow = glutCreateWindow("AR");
        glutDisplayFunc(ArViewer::drawAr);
        glutKeyboardFunc(ArViewer::onKeys);
        glutPositionWindow(0, 0);

        glEnable(GL_CULL_FACE);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_COLOR_MATERIAL);

        glutMainLoop();
    }

    void ArViewer::onKeys(unsigned char key, int x, int y) {
        if (key == KEY_QUIT_APP) { // ESC
            if (quitFlag != NULL) {
                *quitFlag = true;
            }
        } else if (key == KEY_SHOW_FPS) { // FPS
            Config::showFPS = !Config::showFPS;
        } else if (key == KEY_SHOW_MARKER_QUAD) { // marker quad
            Config::showMarkerQuad = !Config::showMarkerQuad;
        }
    }

    void ArViewer::onQuitCmd(bool *flag) {
        quitFlag = flag;
    }

    void ArViewer::setVideoFrame(cv::Mat &frame) {
        image = frame.clone();
    }

    void ArViewer::setGlProjMatrix(double p[16]) {
        memcpy(projectMat, p, sizeof(double) * 16);
    }

    void ArViewer::setGlModelViewMatrix(double p[16]) {
        memcpy(modelViewMat, p, sizeof(double) * 16);
    }

    void ArViewer::drawVideo() {
        if (image.empty()) return;

        glDepthMask(GL_FALSE);
        glDisable(GL_DEPTH_TEST);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

        glColor3f(1, 1, 1);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.cols, image.rows, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, image.data);

        glEnable(GL_TEXTURE_2D);

        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        glOrtho(0, 1, 0, 1, 0, 1);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        glBegin(GL_QUADS);
        {
            glTexCoord2d(0.0, 1.0);
            glVertex2d(0.0, 0.0);
            glTexCoord2d(1.0, 1.0);
            glVertex2d(1.0, 0.0);
            glTexCoord2d(1.0, 0.0);
            glVertex2d(1.0, 1.0);
            glTexCoord2d(0.0, 0.0);
            glVertex2d(0.0, 1.0);
        }
        glEnd();

        glDisable(GL_TEXTURE_2D);
        glDepthMask(GL_TRUE);
        glEnable(GL_DEPTH_TEST);

        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
    }

    void ArViewer::clearDrawable() {
        drawableMutex.lock();
        drawableItems.clear();
        drawableMutex.unlock();
    }

    void ArViewer::addDrawable(Drawable *item) {
        drawableMutex.lock();
        drawableItems.push_back(item);
        drawableMutex.unlock();
    }
}

