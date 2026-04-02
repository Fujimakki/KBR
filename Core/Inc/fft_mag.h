#ifndef INC_FFT_MAG_H_
#define INC_FFT_MAG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "arm_math.h"

#define FFT_SIZE 4096

void fftMagCalc(arm_rfft_fast_instance_f32 *S, float32_t* const inp, float32_t* const outp);

#ifdef __cplusplus
}
#endif

#endif /* INC_FFT_MAG_H_ */
