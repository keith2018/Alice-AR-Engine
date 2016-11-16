/*
 *
 * Alice AR Engine
 *
 * @author 	: keith@robot9.me
 * @date	: 2016/9/20
 *
 */

#include "Pipeline.h"

#define CAM_INDEX 0
#define VIDEO_W 640
#define VIDEO_H 480

int main(int argc, char **argv) {
    Alice::Pipeline pipeline(CAM_INDEX, VIDEO_W, VIDEO_H);

    pipeline.addMarker("butterfly.jpg");
//    pipeline.addMarker("kfc.jpg");

    pipeline.start();

    return 0;
}
