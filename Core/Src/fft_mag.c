#include "fft_mag.h"

/*
 * @brief Calculate fft of the given data
 *
 * @param S the given RFFT/RIFFT function instance
 * @param inp the given pointer to the first element of the input data
 * @param outp the given pointer to the first element of the output data
 */
void fftMagCalc(arm_rfft_fast_instance_f32 *S, float32_t* const inp)
{
  static float32_t fftOutp[FFT_SIZE];
  arm_rfft_fast_f32(S, inp, fftOutp, 0); /* Calculate real FFT of adc_input_f32 to fft_output */

  inp[0] = fabsf(fftOutp[0]) / FFT_SIZE;
  inp[FFT_SIZE / 2 - 1] = fabsf(fftOutp[1]) / FFT_SIZE;

  arm_cmplx_mag_f32(&(fftOutp[2]), &(inp[1]), FFT_SIZE / 2 - 2); /* Get a magnitude from the FFT */
  for(int i = 1; i < FFT_SIZE / 2 - 1; i++)
  {
    inp[i] *= 2.0f / FFT_SIZE;
    inp[i + FFT_SIZE / 2] = 0;
  }
}
