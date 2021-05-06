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
#include "leds.h"
#include "../lib/e-puck2_main-processor/src/sdio.h"
#include "../lib/e-puck2_main-processor/src/fat.h"
#include "../lib/e-puck2_main-processor/src/audio/audio_thread.h"
#include "../lib/e-puck2_main-processor/src/audio/play_melody.h"
#include "../lib/e-puck2_main-processor/src/audio/play_sound_file.h"
#include "../lib/e-puck2_main-processor/src/spi_comm.h"

//uncomment to send the FFTs results from the real microphones
#define SEND_FROM_MIC

//uncomment to use double buffering to send the FFT to the computer
#define DOUBLE_BUFFERING

//uncomment to use disco program only
#define DISCO_EPUCK
#define reverseled 5
#define VOLUME_MAX 50
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

#ifdef DISCO_EPUCK
    while(1){
    	if (isSDCardMounted()){
    		set_body_led(reverseled);
    		set_front_led(reverseled);
    		// char root[] ="robot_dance";
    		  //  scan_files((BaseSequentialStream *) &SDU1, root);
    		   // BYTE size_SD = getSDCardClusterSize();
    		    //chprintf((BaseSequentialStream *) &SDU1,"size_SD %d",size_SD);
    		    char robot[]= "robot_dance/1.wav";
    		    setSoundFileVolume(VOLUME_MAX);
    		    playSoundFile(robot, SF_SIMPLE_PLAY);
    	}
    	else {
    		//max value 255
    		toggle_rgb_led(LED2,RED_LED,200);
    		toggle_rgb_led(LED2,GREEN_LED,200);
    		toggle_rgb_led(LED2,BLUE_LED,0);
    		set_rgb_led(LED4,25,0,23);
    	}
    	chThdSleepMilliseconds(500);
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
