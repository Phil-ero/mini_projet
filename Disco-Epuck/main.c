#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <audio/microphone.h>

#include <audio_processing.h>
#include <fft.h>
#include <communications.h>
#include <arm_math.h>

//includes for disco-epuck
//#include "disco_epuck.h"
#include "leds.h"
#include "../lib/e-puck2_main-processor/src/sdio.h"
#include "../lib/e-puck2_main-processor/src/fat.h"
#include "../lib/e-puck2_main-processor/src/audio/audio_thread.h"
//#include "../lib/e-puck2_main-processor/src/audio/play_melody.h"
#include "../lib/e-puck2_main-processor/src/spi_comm.h"


//uncomment to send the FFTs results from the real microphones
#define SEND_FROM_MIC

//uncomment to use double buffering to send the FFT to the computer
#define DOUBLE_BUFFERING

//uncomment to use disco program only
#define DISCO_EPUCK
#define reverseled 2
#define NB_DANCES 3
#define VOLUME_MAX 50
static bool Dance_playing = 0;
static int Dance_choice =0;
static int blink_ms = 1000;


static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

static void timer12_start(void){
    //General Purpose Timer configuration   
    //timer 12 is a 16 bit timer so we can measure time
    //to about 65ms with a 1Mhz counter
    static const GPTConfig gpt12cfg = {
        1000000,        /* 1MHz timer clock in order to measure uS.*/
        NULL,           /* Timer callback.*/
        0,
        0
    };

    gptStart(&GPTD12, &gpt12cfg);
    //let the timer count to max value
    gptStartContinuous(&GPTD12, 0xFFFF);
}

static const int red_rgb[NB_DANCES][NUM_RGB_LED] = {
	{47,13,0,52},
	{43,20,10,0},
	{34,2,56,6},
//	{37,25,6,69},
};

static const int green_rgb[NB_DANCES][NUM_RGB_LED] = {
	{9,0,84,93},
	{85,44,32,22},
	{0,83,5,13},
//	{4,57,50,39},
};
static const int blue_rgb[NB_DANCES][NUM_RGB_LED] = {
	{86,0,10,0},
	{15,47,21,49},
	{4,67,42,47},
//	{2,5,34,0},
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
	char robot[]= "robot_dance/1.wav";
	char robot1[]= "robot_dance/2.wav";
	char robot2[]= "robot_dance/3.wav";

	if (isSDCardMounted()&&dance){
		switch(choice){
			case ZELDA_COFFRE:
				playSoundFile(robot, SF_SIMPLE_PLAY);
				break;
			case SMASH_INTRO:
				playSoundFile(robot1, SF_SIMPLE_PLAY);
				break;
			case LAST_OF_US:
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
    halInit();
    chSysInit();
    mpu_init();
    //inits LEDS for DISCO_EPUCK
    clear_leds();
    set_body_led(0);
    set_front_led(0);
    //starts the USB communication
    usb_start();
    //inits the motors
    motors_init();
    //start for the disco-epuck sound-system
   	dac_start();
    //starts the serial communication
   	spi_comm_start();
    serial_start();
    //starts timer 12
    timer12_start();
    //mounting the SDCARD
    sdio_start();
    mountSDCard();
//  playMelodyStart();
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
#else
    //send_tab is used to save the state of the buffer to send (double buffering)
    //to avoid modifications of the buffer while sending it
    static float send_tab[FFT_SIZE];
    static complex_float temp_tab[FFT_SIZE];

#ifdef SEND_FROM_MIC
    //starts the microphones processing thread.
    //it calls the callback given in parameter when samples are ready
    mic_start(&processAudioData);
#endif  /* SEND_FROM_MIC */

    /* Infinite loop. */
    while (1) {
#ifdef SEND_FROM_MIC
        //waits until a result must be sent to the computer
        wait_send_to_computer();
#ifdef DOUBLE_BUFFERING
        //we copy the buffer to avoid conflicts
        arm_copy_f32(get_audio_buffer_ptr(LEFT_OUTPUT), send_tab, FFT_SIZE);
        SendFloatToComputer((BaseSequentialStream *) &SD3, send_tab, FFT_SIZE);
#else
        SendFloatToComputer((BaseSequentialStream *) &SD3, get_audio_buffer_ptr(LEFT_OUTPUT), FFT_SIZE);
#endif  /* DOUBLE_BUFFERING */
#else

        float* bufferCmplxInput = get_audio_buffer_ptr(LEFT_CMPLX_INPUT);
        float* bufferOutput = get_audio_buffer_ptr(LEFT_OUTPUT);
        //time measurement variables
        volatile uint16_t time_fft_opti = 0;
        volatile uint16_t time_mag = 0;
        volatile uint16_t time_fft_non =0;

        uint16_t size = ReceiveInt16FromComputer((BaseSequentialStream *) &SD3, bufferCmplxInput, FFT_SIZE);

        if(size == FFT_SIZE){
        	/*
        	* Non optimized FFT
        	*/
        	//converting float buffer into complex_float
        	for(uint16_t i = 0 ; i < (2*FFT_SIZE) ; i+=2){
        	temp_tab[i/2].real = bufferCmplxInput[i];
        	temp_tab[i/2].imag = bufferCmplxInput[i+1];
        	}
        	//measure time of FFT_C
        	chSysLock();
        	//reset the timer counter
        	GPTD12.tim->CNT = 0;
        	doFFT_c(FFT_SIZE, temp_tab);
        	time_fft_non = GPTD12.tim->CNT;
        	chSysUnlock();

        	//reconverts the result into a float buffer
        	for(uint16_t i = 0 ; i < (2*FFT_SIZE) ; i+=2){
        	bufferCmplxInput[i] = temp_tab[i/2].real;
        	bufferCmplxInput[i+1] = temp_tab[i/2].imag;
        	}
        	/*
        	* Optimized FFT
        	*/
        	 chSysLock();
        	 //reset the timer counter
        	 GPTD12.tim->CNT = 0;
        	 doFFT_optimized(FFT_SIZE, bufferCmplxInput);
        	 time_fft_opti = GPTD12.tim->CNT;
        	 chSysUnlock();

        	chSysLock();
        	//reset the timer counter
        	GPTD12.tim->CNT = 0;
        	arm_cmplx_mag_f32(bufferCmplxInput, bufferOutput, FFT_SIZE);
        	time_mag = GPTD12.tim->CNT;
        	chSysUnlock();

            SendFloatToComputer((BaseSequentialStream *) &SD3, bufferOutput, FFT_SIZE);
            chprintf((BaseSequentialStream *) &SDU1, "time_fft_non = %d us, time_fft_opti = %d us, time_magnitude = %d us\n",
            time_fft_non, time_fft_opti, time_mag);
        }
#endif  /* SEND_FROM_MIC */
    }
#endif /* DISCO_EPUCK */
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
