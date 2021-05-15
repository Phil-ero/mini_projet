#include "ch.h"
#include "hal.h"
#include <main.h>
#include <chprintf.h>

#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <fft.h>
#include <arm_math.h>

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//For our application, we use only the left mic
//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
//static float micRight_cmplx_input[2 * FFT_SIZE];
//static float micFront_cmplx_input[2 * FFT_SIZE];
//static float micBack_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
//static float micRight_output[FFT_SIZE];
//static float micFront_output[FFT_SIZE];
//static float micBack_output[FFT_SIZE];

#define MIN_VALUE_THRESHOLD 10000
//resolution of 8kHz/512 = 15.625 Hz // 512 des 1024 utile car dans une FFT il a le pic dans les negatifs
// et celui dans les positifs.(-8kHz a 8kHz dans une FFT)
#define MIN_FREQ 25 //we don't analyze before this index to not use resources for nothing
#define FREQ_MODE 26 //406.25Hz
#define FREQ_DANCE1 58 //906.25Hz
#define FREQ_DANCE2 61 //953.125HZ
#define FREQ_DANCE3 65 //1015.625Hz
#define MAX_FREQ 67 //we don't analyze after this index to not use resources for nothing
#define FREQ_MODE_L (FREQ_MODE-1)
#define FREQ_MODE_H (FREQ_MODE+1)
#define FREQ_DANCE1_L (FREQ_DANCE1-1)
#define FREQ_DANCE1_H (FREQ_DANCE1+1)
#define FREQ_DANCE2_L (FREQ_DANCE2-1)
#define FREQ_DANCE2_H (FREQ_DANCE2+1)
#define FREQ_DANCE3_L (FREQ_DANCE3-1)
#define FREQ_DANCE3_H (FREQ_DANCE3+1)
#define dance_mode true
#define chasse_mode false

/*
* Simple function used to detect the highest value in a buffer
* and to execute a motor command depending on it
*/
void sound_remote(float* data){
	float max_norm = MIN_VALUE_THRESHOLD*3.5;
	int16_t max_norm_index = -1;
	//search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
			max_norm_index = i;
		}
	}
	//CHANGE MODE for chasse
	if(max_norm_index >= FREQ_MODE_L && max_norm_index <= FREQ_MODE_H && get_mode()){
		stopCurrentSoundFile();
		waitSoundFileHasFinished();
		play_true();
		change_mode(chasse_mode);
		change_blink_speed (1000);
		left_motor_set_speed(0);
		right_motor_set_speed(0);
	}
	//dance 1
	else if(max_norm_index >= FREQ_DANCE1_L && max_norm_index <= FREQ_DANCE1_H && !get_mode()){
		change_dance(ZELDA_COFFRE);
		change_mode(dance_mode);
		change_blink_speed (500);
		left_motor_set_speed(-600);
		right_motor_set_speed(600);
	}
	//dance 2
	else if(max_norm_index >= FREQ_DANCE2_L && max_norm_index <= FREQ_DANCE2_H && !get_mode()){
		change_dance(SMASH_INTRO);
		change_mode(dance_mode);
		change_blink_speed (200);
		left_motor_set_speed(600);
		right_motor_set_speed(-600);
	}
	//dance 3
	else if(max_norm_index >= FREQ_DANCE3_L && max_norm_index <= FREQ_DANCE3_H && !get_mode()){
		change_dance(PIRATES_CARAIBES);
		change_mode(dance_mode);
		change_blink_speed (1000);
		left_motor_set_speed(0);
		right_motor_set_speed(0);
	}
	else{
		;
	}
}

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void processAudioData(int16_t *data, uint16_t num_samples){
	/*
	*
	* We get 160 samples per mic every 10ms
	* So we fill the samples buffers to reach
	* 1024 samples, then we compute the FFTs.
	*
	*/
	static uint16_t nb_samples = 0;
	//static uint8_t mustSend = 0;
	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4){
		//construct an array of complex numbers. Put 0 to the imaginary part
		//micRight_cmplx_input[nb_samples] = (float)data[i + MIC_RIGHT];
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];
		//micBack_cmplx_input[nb_samples] = (float)data[i + MIC_BACK];
		//micFront_cmplx_input[nb_samples] = (float)data[i + MIC_FRONT];
		nb_samples++;
		//micRight_cmplx_input[nb_samples] = 0;
		micLeft_cmplx_input[nb_samples] = 0;
		//micBack_cmplx_input[nb_samples] = 0;
		//micFront_cmplx_input[nb_samples] = 0;
		nb_samples++;
	//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}
	if(nb_samples >= (2 * FFT_SIZE)){
		/* FFT proccessing
		*
		* This FFT function stores the results in the input buffer given.
		* This is an "In Place" function.
		*/
		//doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		//doFFT_optimized(FFT_SIZE, micFront_cmplx_input);
		//doFFT_optimized(FFT_SIZE, micBack_cmplx_input);
		/* Magnitude processing
		*
		* Computes the magnitude of the complex numbers and
		* stores them in a buffer of FFT_SIZE because it only contains
		* real numbers.
		*
		*/
		//arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		//arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);
		//arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);
		//sends only one FFT result over 10 for 1 mic to not flood the computer
		//sends to UART3 ?
//		if(mustSend > 8){
//			//signals to send the result to the computer
//			chBSemSignal(&sendToComputer_sem);
//			mustSend = 0;
//		}
		nb_samples = 0;
		//mustSend++;
		sound_remote(micLeft_output);
		}
}
