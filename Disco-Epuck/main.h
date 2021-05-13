#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"
#include "../lib/e-puck2_main-processor/src/audio/play_sound_file.h"

enum dance_selection{
	//dances available
	ZELDA_COFFRE = 0,
	SMASH_INTRO,
	LAST_OF_US,
	NB_DANCES,
};

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;
void change_mode(bool mode);
void change_dance(int dance_nb);
void change_blink_speed (int blink);
bool get_mode(void);

#ifdef __cplusplus
}
#endif

#endif
