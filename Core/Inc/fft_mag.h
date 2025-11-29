/*
 * fft_mag.h
 *
 *  Created on: Nov 16, 2025
 *      Author: space-monkey
 */

#ifndef INC_FFT_MAG_H_
#define INC_FFT_MAG_H_

#include <stdint.h>
#include "arm_math.h"

#define FFT_SIZE 4096

void fftMagCalc(arm_rfft_fast_instance_f32 *S, float32_t* const inp, float32_t* const outp);

#endif /* INC_FFT_MAG_H_ */
