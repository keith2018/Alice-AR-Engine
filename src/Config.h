/*
 *
 * Alice AR Engine
 *
 * @author 	: keith@robot9.me
 * @date	: 2016/9/21
 *
 */

#ifndef ALICE_CONFIG_H
#define ALICE_CONFIG_H


#define KEY_QUIT_APP 27
#define KEY_SHOW_MARKER_QUAD 'd'
#define KEY_SHOW_FPS 'f'

namespace Alice {

    class Config {
    public:
        static bool showMarkerQuad;
        static bool showFPS;
    };
}

#endif //ALICE_CONFIG_H
