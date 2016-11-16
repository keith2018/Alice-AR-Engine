/*
 *
 * Alice AR Engine
 *
 * @author 	: keith@robot9.me
 * @date	: 2016/9/20
 *
 */

#ifndef ALICE_ARVIEWER_H
#define ALICE_ARVIEWER_H


#include <opencv2/opencv.hpp>

#ifdef _WIN32
#   include <windows.h>
#   include <GL/gl.h>
#   include <GL/glu.h>
#   include <glut.h>
#elif defined(__APPLE__)
#   include <OpenGL/gl.h>
#   include <OpenGL/glu.h>
#   include <GLUT/glut.h>
#else
#   include <GL/gl.h>
#   include <GL/glu.h>
#   include <GL/glut.h>
#endif

#define MAX_ERR_FRAME 5

namespace Alice {
    class Drawable {
    public:
        Drawable() :
                width(1.0f),
                height(1.0f),
                visible(false),
                errFrameCnt(0) {}

        virtual void draw();

        virtual void drawContent();

        double glMat[16];

        void setVisible(bool isVisible) {
            if (isVisible) {
                visible = true;
                errFrameCnt = 0;
            } else {
                if (visible) {
                    if (errFrameCnt >= MAX_ERR_FRAME) {
                        visible = false;
                        errFrameCnt = 0;
                    } else {
                        errFrameCnt++;
                    }
                }
            }
        }

        inline bool isVisible() {
            return visible;
        }

        inline void setSize(float w, float h) {
            width = w;
            height = h;
        }

    private:
        float width;
        float height;
        bool visible;
        int errFrameCnt;
    };

    namespace ArViewer {
        void start(int argc, char **argv, int w, int h);

        void onKeys(unsigned char key, int x, int y);

        void onQuitCmd(bool *flag);

        void setVideoFrame(cv::Mat &frame);

        void setGlProjMatrix(double p[16]);

        void setGlModelViewMatrix(double p[16]);

        void drawVideo();

        void drawAr();

        void drawContent();

        void clearDrawable();

        void addDrawable(Drawable *item);
    }
}


#endif //ALICE_ARVIEWER_H
