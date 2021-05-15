#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <audio/microphone.h>

#include <audio_processing.h>
#include <fft.h>
#include <arm_math.h>

//includes for disco-epuck
#include <proximity.h>
#include "leds.h"
#include "../lib/e-puck2_main-processor/src/sdio.h"
#include "../lib/e-puck2_main-processor/src/fat.h"
#include "../lib/e-puck2_main-processor/src/audio/audio_thread.h"
#include "../lib/e-puck2_main-processor/src/spi_comm.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


//uncomment to use disco program only
#define DISCO_EPUCK
#define reverseled 2
#define NB_DANCES 3
#define VOLUME_MAX 50
static bool Dance_playing = 0;
static int Dance_choice =0;
static int blink_ms = 1000;


static const int red_rgb[NB_DANCES][NUM_RGB_LED] = {
	{47,13,0,52},
	{43,20,10,0},
	{34,2,56,6},
};

static const int green_rgb[NB_DANCES][NUM_RGB_LED] = {
	{9,0,84,93},
	{85,44,32,22},
	{0,83,5,13},
};
static const int blue_rgb[NB_DANCES][NUM_RGB_LED] = {
	{86,0,10,0},
	{15,47,21,49},
	{4,67,42,47},
};

void toggle_dance_leds(bool dance ,int choice){
	if(dance){
		//blink body and front leds
		set_body_led(reverseled);
		set_front_led(reverseled);
		//blink for the red LED
		for (int i = 0; i< NUM_LED; i++){
			set_led(i, reverseled);
		}
		//Blink for the RGB LED
		for(int i = 0; i< NUM_RGB_LED; i++){
			toggle_rgb_led(i,RED_LED,red_rgb[choice][i]);
			toggle_rgb_led(i,GREEN_LED,green_rgb[choice][i]);
			toggle_rgb_led(i,BLUE_LED,blue_rgb[choice][i]);
		}
	}
	else{
		clear_leds();
		set_body_led(dance);
		set_front_led(dance);
	}
}

void play_music(bool dance ,int choice){
	char robot[]= "robot/ZELDA.wav";
	char robot1[]= "robot/SMAS.wav";
	char robot2[]= "robot/3.wav";

	if (isSDCardMounted()&&dance){
		switch(choice){
			case ZELDA_COFFRE:
				playSoundFile(robot, SF_SIMPLE_PLAY);
				break;
			case SMASH_INTRO:
				playSoundFile(robot1, SF_SIMPLE_PLAY);
				break;
			case PIRATES_CARAIBES:
				playSoundFile(robot2, SF_SIMPLE_PLAY);
				break;
		}
	}
}

void change_mode(bool mode){
	Dance_playing = mode;
}

void change_dance(int dance_nb){
	Dance_choice = dance_nb;
}

void change_blink_speed (int blink){
	blink_ms = blink;
}

bool get_mode(void){
	return Dance_playing;
}

int main(void)
{
	//init of bus :
	messagebus_init(&bus, &bus_lock, &bus_condvar);
    halInit();
    chSysInit();
    mpu_init();
    clear_leds();
    set_body_led(0);
    set_front_led(0);
    //inits the motors
    motors_init();
    //init the proximity sensors and calibrate
    proximity_start();
    calibrate_ir();
    //start for the disco-epuck sound-system
   	dac_start();
    //starts the serial communication
   	spi_comm_start();
    //mounting the SDCARD
    sdio_start();
    mountSDCard();
    //starting THD soundfiles
    playSoundFileStart();
    setSoundFileVolume(VOLUME_MAX*0.6);

#ifdef DISCO_EPUCK
    mic_start(&processAudioData);
    while(1){
    	//main thread is charged of the blinks of the leds depending of dance.
    	toggle_dance_leds(Dance_playing,Dance_choice);
    	play_music(Dance_playing,Dance_choice);
    	chThdSleepMilliseconds(blink_ms);
    }
#endif /* DISCO_EPUCK */
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
