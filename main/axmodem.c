/* ========================================
 *
 * Copyright © 2015,2019, Dana H. Myers.
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Dana H. Myers.
 *
 * ========================================
*/

/*
 * Following copyright notice is for the arm_* functions taken
 * from CMSIS-DSP, used until Espressif ESP-DSP has fast sin/cos functions
 */

/*
 * Copyright (C) 2010-2019 ARM Limited or its affiliates. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/i2s.h"

#include "axmodem.h"
#include "axfcs.h"
#include "kbuf.h"

#include "igate/tnc2.h"
#include "igate/igate.h"

#include <math.h>

#include "dsps_tone_gen.h"
#include "dsps_fir.h"
#include "dsps_mul.h"
#include "dsps_add.h"
#include "dsps_sub.h"


/*
 * XXX: really want to re-factor the dspIn/dspOut
 * tasks out of here, as well as the KISS processing
 * dspOut depends on the (temporarily-hijacked) ARM
 * fast sin/cos code so it lives here
 *
 * Maybe refactor the ARM sin/cos out until Espressif
 * updates esp-dsp?
 */


/*
 * XXX: lengths for receive/transmit queues. Probably
 * ought to size the outbound/transmit queue much larger
 * since the host can fill it rapidly, while receive
 * queue is limited to radio bit rate
 */
#define RXF_QUEUE_LEN   10
#define TXF_QUEUE_LEN   10


/*
 *
 */
static int16_t txBuffer[DMA_BLOCK_LEN * 2];

/*
 *
 */
static const char* TAG = "axmodem_proc";

/*
 * arm fast math taken from CMSIS-DSP
 * XXX: replace with esp-dsp when available
 */

#define FAST_MATH_TABLE_SIZE  512

static const float sinTable_f32[FAST_MATH_TABLE_SIZE + 1] = {
   0.00000000f, 0.01227154f, 0.02454123f, 0.03680722f, 0.04906767f, 0.06132074f,
   0.07356456f, 0.08579731f, 0.09801714f, 0.11022221f, 0.12241068f, 0.13458071f,
   0.14673047f, 0.15885814f, 0.17096189f, 0.18303989f, 0.19509032f, 0.20711138f,
   0.21910124f, 0.23105811f, 0.24298018f, 0.25486566f, 0.26671276f, 0.27851969f,
   0.29028468f, 0.30200595f, 0.31368174f, 0.32531029f, 0.33688985f, 0.34841868f,
   0.35989504f, 0.37131719f, 0.38268343f, 0.39399204f, 0.40524131f, 0.41642956f,
   0.42755509f, 0.43861624f, 0.44961133f, 0.46053871f, 0.47139674f, 0.48218377f,
   0.49289819f, 0.50353838f, 0.51410274f, 0.52458968f, 0.53499762f, 0.54532499f,
   0.55557023f, 0.56573181f, 0.57580819f, 0.58579786f, 0.59569930f, 0.60551104f,
   0.61523159f, 0.62485949f, 0.63439328f, 0.64383154f, 0.65317284f, 0.66241578f,
   0.67155895f, 0.68060100f, 0.68954054f, 0.69837625f, 0.70710678f, 0.71573083f,
   0.72424708f, 0.73265427f, 0.74095113f, 0.74913639f, 0.75720885f, 0.76516727f,
   0.77301045f, 0.78073723f, 0.78834643f, 0.79583690f, 0.80320753f, 0.81045720f,
   0.81758481f, 0.82458930f, 0.83146961f, 0.83822471f, 0.84485357f, 0.85135519f,
   0.85772861f, 0.86397286f, 0.87008699f, 0.87607009f, 0.88192126f, 0.88763962f,
   0.89322430f, 0.89867447f, 0.90398929f, 0.90916798f, 0.91420976f, 0.91911385f,
   0.92387953f, 0.92850608f, 0.93299280f, 0.93733901f, 0.94154407f, 0.94560733f,
   0.94952818f, 0.95330604f, 0.95694034f, 0.96043052f, 0.96377607f, 0.96697647f,
   0.97003125f, 0.97293995f, 0.97570213f, 0.97831737f, 0.98078528f, 0.98310549f,
   0.98527764f, 0.98730142f, 0.98917651f, 0.99090264f, 0.99247953f, 0.99390697f,
   0.99518473f, 0.99631261f, 0.99729046f, 0.99811811f, 0.99879546f, 0.99932238f,
   0.99969882f, 0.99992470f, 1.00000000f, 0.99992470f, 0.99969882f, 0.99932238f,
   0.99879546f, 0.99811811f, 0.99729046f, 0.99631261f, 0.99518473f, 0.99390697f,
   0.99247953f, 0.99090264f, 0.98917651f, 0.98730142f, 0.98527764f, 0.98310549f,
   0.98078528f, 0.97831737f, 0.97570213f, 0.97293995f, 0.97003125f, 0.96697647f,
   0.96377607f, 0.96043052f, 0.95694034f, 0.95330604f, 0.94952818f, 0.94560733f,
   0.94154407f, 0.93733901f, 0.93299280f, 0.92850608f, 0.92387953f, 0.91911385f,
   0.91420976f, 0.90916798f, 0.90398929f, 0.89867447f, 0.89322430f, 0.88763962f,
   0.88192126f, 0.87607009f, 0.87008699f, 0.86397286f, 0.85772861f, 0.85135519f,
   0.84485357f, 0.83822471f, 0.83146961f, 0.82458930f, 0.81758481f, 0.81045720f,
   0.80320753f, 0.79583690f, 0.78834643f, 0.78073723f, 0.77301045f, 0.76516727f,
   0.75720885f, 0.74913639f, 0.74095113f, 0.73265427f, 0.72424708f, 0.71573083f,
   0.70710678f, 0.69837625f, 0.68954054f, 0.68060100f, 0.67155895f, 0.66241578f,
   0.65317284f, 0.64383154f, 0.63439328f, 0.62485949f, 0.61523159f, 0.60551104f,
   0.59569930f, 0.58579786f, 0.57580819f, 0.56573181f, 0.55557023f, 0.54532499f,
   0.53499762f, 0.52458968f, 0.51410274f, 0.50353838f, 0.49289819f, 0.48218377f,
   0.47139674f, 0.46053871f, 0.44961133f, 0.43861624f, 0.42755509f, 0.41642956f,
   0.40524131f, 0.39399204f, 0.38268343f, 0.37131719f, 0.35989504f, 0.34841868f,
   0.33688985f, 0.32531029f, 0.31368174f, 0.30200595f, 0.29028468f, 0.27851969f,
   0.26671276f, 0.25486566f, 0.24298018f, 0.23105811f, 0.21910124f, 0.20711138f,
   0.19509032f, 0.18303989f, 0.17096189f, 0.15885814f, 0.14673047f, 0.13458071f,
   0.12241068f, 0.11022221f, 0.09801714f, 0.08579731f, 0.07356456f, 0.06132074f,
   0.04906767f, 0.03680722f, 0.02454123f, 0.01227154f, 0.00000000f, -0.01227154f,
   -0.02454123f, -0.03680722f, -0.04906767f, -0.06132074f, -0.07356456f,
   -0.08579731f, -0.09801714f, -0.11022221f, -0.12241068f, -0.13458071f,
   -0.14673047f, -0.15885814f, -0.17096189f, -0.18303989f, -0.19509032f,
   -0.20711138f, -0.21910124f, -0.23105811f, -0.24298018f, -0.25486566f,
   -0.26671276f, -0.27851969f, -0.29028468f, -0.30200595f, -0.31368174f,
   -0.32531029f, -0.33688985f, -0.34841868f, -0.35989504f, -0.37131719f,
   -0.38268343f, -0.39399204f, -0.40524131f, -0.41642956f, -0.42755509f,
   -0.43861624f, -0.44961133f, -0.46053871f, -0.47139674f, -0.48218377f,
   -0.49289819f, -0.50353838f, -0.51410274f, -0.52458968f, -0.53499762f,
   -0.54532499f, -0.55557023f, -0.56573181f, -0.57580819f, -0.58579786f,
   -0.59569930f, -0.60551104f, -0.61523159f, -0.62485949f, -0.63439328f,
   -0.64383154f, -0.65317284f, -0.66241578f, -0.67155895f, -0.68060100f,
   -0.68954054f, -0.69837625f, -0.70710678f, -0.71573083f, -0.72424708f,
   -0.73265427f, -0.74095113f, -0.74913639f, -0.75720885f, -0.76516727f,
   -0.77301045f, -0.78073723f, -0.78834643f, -0.79583690f, -0.80320753f,
   -0.81045720f, -0.81758481f, -0.82458930f, -0.83146961f, -0.83822471f,
   -0.84485357f, -0.85135519f, -0.85772861f, -0.86397286f, -0.87008699f,
   -0.87607009f, -0.88192126f, -0.88763962f, -0.89322430f, -0.89867447f,
   -0.90398929f, -0.90916798f, -0.91420976f, -0.91911385f, -0.92387953f,
   -0.92850608f, -0.93299280f, -0.93733901f, -0.94154407f, -0.94560733f,
   -0.94952818f, -0.95330604f, -0.95694034f, -0.96043052f, -0.96377607f,
   -0.96697647f, -0.97003125f, -0.97293995f, -0.97570213f, -0.97831737f,
   -0.98078528f, -0.98310549f, -0.98527764f, -0.98730142f, -0.98917651f,
   -0.99090264f, -0.99247953f, -0.99390697f, -0.99518473f, -0.99631261f,
   -0.99729046f, -0.99811811f, -0.99879546f, -0.99932238f, -0.99969882f,
   -0.99992470f, -1.00000000f, -0.99992470f, -0.99969882f, -0.99932238f,
   -0.99879546f, -0.99811811f, -0.99729046f, -0.99631261f, -0.99518473f,
   -0.99390697f, -0.99247953f, -0.99090264f, -0.98917651f, -0.98730142f,
   -0.98527764f, -0.98310549f, -0.98078528f, -0.97831737f, -0.97570213f,
   -0.97293995f, -0.97003125f, -0.96697647f, -0.96377607f, -0.96043052f,
   -0.95694034f, -0.95330604f, -0.94952818f, -0.94560733f, -0.94154407f,
   -0.93733901f, -0.93299280f, -0.92850608f, -0.92387953f, -0.91911385f,
   -0.91420976f, -0.90916798f, -0.90398929f, -0.89867447f, -0.89322430f,
   -0.88763962f, -0.88192126f, -0.87607009f, -0.87008699f, -0.86397286f,
   -0.85772861f, -0.85135519f, -0.84485357f, -0.83822471f, -0.83146961f,
   -0.82458930f, -0.81758481f, -0.81045720f, -0.80320753f, -0.79583690f,
   -0.78834643f, -0.78073723f, -0.77301045f, -0.76516727f, -0.75720885f,
   -0.74913639f, -0.74095113f, -0.73265427f, -0.72424708f, -0.71573083f,
   -0.70710678f, -0.69837625f, -0.68954054f, -0.68060100f, -0.67155895f,
   -0.66241578f, -0.65317284f, -0.64383154f, -0.63439328f, -0.62485949f,
   -0.61523159f, -0.60551104f, -0.59569930f, -0.58579786f, -0.57580819f,
   -0.56573181f, -0.55557023f, -0.54532499f, -0.53499762f, -0.52458968f,
   -0.51410274f, -0.50353838f, -0.49289819f, -0.48218377f, -0.47139674f,
   -0.46053871f, -0.44961133f, -0.43861624f, -0.42755509f, -0.41642956f,
   -0.40524131f, -0.39399204f, -0.38268343f, -0.37131719f, -0.35989504f,
   -0.34841868f, -0.33688985f, -0.32531029f, -0.31368174f, -0.30200595f,
   -0.29028468f, -0.27851969f, -0.26671276f, -0.25486566f, -0.24298018f,
   -0.23105811f, -0.21910124f, -0.20711138f, -0.19509032f, -0.18303989f,
   -0.17096189f, -0.15885814f, -0.14673047f, -0.13458071f, -0.12241068f,
   -0.11022221f, -0.09801714f, -0.08579731f, -0.07356456f, -0.06132074f,
   -0.04906767f, -0.03680722f, -0.02454123f, -0.01227154f, -0.00000000f
};


IRAM_ATTR
static float
arm_sin_f32(float x)
{
  float sinVal, fract, in;                   /* Temporary input, output variables */
  uint16_t index;                                /* Index variable */
  float a, b;                                /* Two nearest output values */
  int32_t n;
  float findex;

  /* input x is in radians */
  /* Scale input to [0 1] range from [0 2*PI] , divide input by 2*pi */
  in = x * 0.159154943092f;

  /* Calculation of floor value of input */
  n = (int32_t) in;

  /* Make negative values towards -infinity */
  if (in < 0.0f)
  {
    n--;
  }

  /* Map input value to [0 1] */
  in = in - (float) n;

  /* Calculation of index of the table */
  findex = (float)FAST_MATH_TABLE_SIZE * in;
  index = (uint16_t)findex;

  /* when "in" is exactly 1, we need to rotate the index down to 0 */
  if (index >= FAST_MATH_TABLE_SIZE) {
    index = 0;
    findex -= (float)FAST_MATH_TABLE_SIZE;
  }

  /* fractional value calculation */
  fract = findex - (float) index;

  /* Read two nearest values of input value from the sin table */
  a = sinTable_f32[index];
  b = sinTable_f32[index+1];

  /* Linear interpolation process */
  sinVal = (1.0f - fract) * a + fract * b;

  /* Return output value */
  return (sinVal);
}




IRAM_ATTR
static float
arm_cos_f32(float x)
{
  float cosVal, fract, in;                   /* Temporary input, output variables */
  uint16_t index;                                /* Index variable */
  float a, b;                                /* Two nearest output values */
  int32_t n;
  float findex;

  /* input x is in radians */
  /* Scale input to [0 1] range from [0 2*PI] , divide input by 2*pi, add 0.25 (pi/2) to read sine table */
  in = x * 0.159154943092f + 0.25f;

  /* Calculation of floor value of input */
  n = (int32_t) in;

  /* Make negative values towards -infinity */
  if (in < 0.0f)
  {
    n--;
  }

  /* Map input value to [0 1] */
  in = in - (float) n;

  /* Calculation of index of the table */
  findex = (float)FAST_MATH_TABLE_SIZE * in;
  index = (uint16_t)findex;

  /* when "in" is exactly 1, we need to rotate the index down to 0 */
  if (index >= FAST_MATH_TABLE_SIZE) {
    index = 0;
    findex -= (float)FAST_MATH_TABLE_SIZE;
  }

  /* fractional value calculation */
  fract = findex - (float) index;

  /* Read two nearest values of input value from the cos table */
  a = sinTable_f32[index];
  b = sinTable_f32[index+1];

  /* Linear interpolation process */
  cosVal = (1.0f - fract) * a + fract * b;

  /* Return output value */
  return (cosVal);
}

/*
 * Delay-only version of FIR
 * Does not use coefficients; implements
 * (N-1)/2 delay output
 * Assumes odd-length delay line
 */
IRAM_ATTR static esp_err_t
delay_fir_f32_ansi(fir_f32_t *fir, const float *input, float *output, int len)
{
    int delayPos;

    /*
     * Calculate the delayed index
     */
    delayPos = fir->N / 2 + fir->pos + 1;
    /* check for wrap */
    if (delayPos >= fir->N) {
        delayPos -= fir->N;
    }

    for (int i = 0 ; i < len ; i++) {
        fir->delay[fir->pos++] = input[i];
        /* check for delay wrap */
        if (fir->pos >= fir->N) {
            fir->pos = 0;
        }

        output[i] = fir->delay[delayPos++];
        /* check for wrap */
        if (delayPos >= fir->N) {
            delayPos = 0;
        }
    }
    return (ESP_OK);
}


/*
 *
 */
QueueHandle_t rxFrameQueue;
kbuf_t *rxFrameQueueStorage[RXF_QUEUE_LEN];
StaticQueue_t rxFrameQueueBuffer;

QueueHandle_t txFrameQueue;
kbuf_t *txFrameQueueStorage[TXF_QUEUE_LEN];
StaticQueue_t txFrameQueueBuffer;


/*
 * DPLL
 */
#define D_PLL_INC       (SAMPLES_BIT)
#define D_PLL_MAX       (D_PLL_INC * SAMPLES_BIT)
#define D_PLL_MARGIN    (2)
#define	D_ERR_DIV		(6)

/*
 *
 */
static uint8_t     dataCarrier;
static uint64_t    t_sample;

/*
 *
 */
enum {
	AX_START_FRAME = 0x100, AX_END_FRAME
};

static volatile uint32_t frame = 0;
static volatile uint32_t good_frame = 0;
static volatile uint32_t dropped_frame = 0;

enum {
	RX_WAITING, RX_DCD, RX_DECODING
};
static uint_fast8_t rx_state = RX_WAITING;

/*
 * Bandpass 1000-2400Hz, sinc 2.2
 *
 * Produced with Iowa Hills FIR tool
 *
 */
#define  BPF_FIR_LEN   219

static const float bpfCoeffs[BPF_FIR_LEN] = {
    -140.4603029908907390E-9, -265.7522305638085530E-9, 1.245757590104237920E-6,
     3.998971851295209930E-6, 3.666285273312804180E-6, -486.1997118480872470E-9,
    -595.3045679658054040E-9, 6.833591747273724870E-6, 6.821787697635157240E-6,
    -17.04045494549157520E-6, -48.20968303961758750E-6, -44.36429516747108440E-6,
     3.426248584342812990E-6, 42.58469567623706810E-6, 26.10874851905444770E-6,
    -5.821754030867900550E-6, 31.40572214776314650E-6, 126.9909667092130690E-6,
     145.2148542301725630E-6, -527.1371725226213130E-9, -189.1600933473912390E-6,
    -213.4547670365302570E-6, -67.44890876185709770E-6, 14.84454591679207520E-6,
    -107.8766415896280930E-6, -224.1795719785781390E-6, -21.96339153802984970E-6,
     419.0635200210573430E-6, 617.6359031919890870E-6, 299.8457467102275590E-6,
    -172.1331906656511140E-6, -241.3284912313663710E-6, 45.43726566717658240E-6,
     42.30238812367892360E-6, -547.7044537192380180E-6, -0.001074047911917989,
    -693.0064901286506260E-6, 407.5008621289786110E-6, 0.001077595535833429,
     688.1824388237276940E-6, 40.26362762925707270E-6, 266.8283818125415220E-6,
     0.001096427368729800, 0.001020327710013794, -577.6588807449775230E-6,
    -0.002227962430452355, -0.002071615417324783, -383.5139336163275630E-6,
     663.5389286797469590E-6, -17.03083621966822040E-6, -770.1087822363559780E-6,
     505.3913540826229680E-6, 0.003034283493879875, 0.003672054314190117,
     0.001064451313521444, -0.002155900087046190, -0.002596113882917897,
    -691.2241821911827630E-6, -136.5511532418327650E-6, -0.002482566912277331,
    -0.004369852024286790, -0.001858505211571159, 0.003643810217380910,
     0.006398340314570538, 0.003699452660436780, -333.8314077447200250E-6,
    -336.0841922385941980E-6, 0.002584748043598547, 0.002069076740213545,
    -0.004190709596143625, -0.009906466393320736, -0.007750839257247273,
     501.1313903488029380E-6, 0.005602872021466720, 0.003074408642419833,
    -571.8933533435470050E-6, 0.002862824587548103, 0.010579866885027527,
     0.011094211028342611, -12.34937049261777100E-6, -0.012305889655613743,
    -0.012973428422646587, -0.003866473277003208, 829.6630707071961980E-6,
    -0.005308320553674150, -0.010614705655951672, -0.001024459108828777,
     0.018181123414248903, 0.025867196893788282, 0.012189428251383673,
    -0.006770240085581663, -0.009380826126307451, 0.001549845827072821,
     0.001565027497843119, -0.019865588182397979, -0.038969660570560359,
    -0.025304669270438725, 0.014940589442091678, 0.040464431277711356,
     0.026706585788103227, 0.001678466680514313, 0.010899615808432217,
     0.048790659025406878, 0.049923174383240912, -0.031612572025894875,
    -0.143909315997260645, -0.167986718570691279, -0.044210914103858315,
     0.143503883801203297, 0.233276367187500000, 0.143503883801203297,
    -0.044210914103858315, -0.167986718570691279, -0.143909315997260645,
    -0.031612572025894875, 0.049923174383240912, 0.048790659025406878,
     0.010899615808432217, 0.001678466680514313, 0.026706585788103227,
     0.040464431277711356, 0.014940589442091678, -0.025304669270438725,
    -0.038969660570560359, -0.019865588182397979, 0.001565027497843119,
     0.001549845827072821, -0.009380826126307451, -0.006770240085581663,
     0.012189428251383673, 0.025867196893788282, 0.018181123414248903,
    -0.001024459108828777, -0.010614705655951672, -0.005308320553674150,
     829.6630707071961980E-6, -0.003866473277003208, -0.012973428422646587,
    -0.012305889655613743, -12.34937049261777100E-6, 0.011094211028342611,
     0.010579866885027527, 0.002862824587548103, -571.8933533435470050E-6,
     0.003074408642419833, 0.005602872021466720, 501.1313903488029380E-6,
    -0.007750839257247273, -0.009906466393320736, -0.004190709596143625,
     0.002069076740213545, 0.002584748043598547, -336.0841922385941980E-6,
    -333.8314077447200250E-6, 0.003699452660436780, 0.006398340314570538,
     0.003643810217380910, -0.001858505211571159, -0.004369852024286790,
    -0.002482566912277331, -136.5511532418327650E-6, -691.2241821911827630E-6,
    -0.002596113882917897, -0.002155900087046190, 0.001064451313521444,
     0.003672054314190117, 0.003034283493879875, 505.3913540826229680E-6,
    -770.1087822363559780E-6, -17.03083621966822040E-6, 663.5389286797469590E-6,
    -383.5139336163275630E-6, -0.002071615417324783, -0.002227962430452355,
    -577.6588807449775230E-6, 0.001020327710013794, 0.001096427368729800,
     266.8283818125415220E-6, 40.26362762925707270E-6, 688.1824388237276940E-6,
     0.001077595535833429, 407.5008621289786110E-6, -693.0064901286506260E-6,
    -0.001074047911917989, -547.7044537192380180E-6, 42.30238812367892360E-6,
     45.43726566717658240E-6, -241.3284912313663710E-6, -172.1331906656511140E-6,
     299.8457467102275590E-6, 617.6359031919890870E-6, 419.0635200210573430E-6,
    -21.96339153802984970E-6, -224.1795719785781390E-6, -107.8766415896280930E-6,
     14.84454591679207520E-6, -67.44890876185709770E-6, -213.4547670365302570E-6,
    -189.1600933473912390E-6, -527.1371725226213130E-9, 145.2148542301725630E-6,
     126.9909667092130690E-6, 31.40572214776314650E-6, -5.821754030867900550E-6,
     26.10874851905444770E-6, 42.58469567623706810E-6, 3.426248584342812990E-6,
    -44.36429516747108440E-6, -48.20968303961758750E-6, -17.04045494549157520E-6,
     6.821787697635157240E-6, 6.833591747273724870E-6, -595.3045679658054040E-9,
    -486.1997118480872470E-9, 3.666285273312804180E-6, 3.998971851295209930E-6,
     1.245757590104237920E-6, -265.7522305638085530E-9, -140.4603029908907390E-9
};

static float bpfDelay[BPF_FIR_LEN];
static fir_f32_t   bpfFir;

/*
 * Quadrature down-mixer
 */
#define	MIX_LO_INCR	((float)((2.0 * M_PI * 1700.0) / (double)SAMPLE_RATE))
static float mixLoPhase;

/*
 * Mixer LPF
 *
 * Coefficients are shared between I and Q channels
 * Coefficients generated by liquid-sdr:
 * fc = 708.0 / (double)SAMPLE_RATE;
 * ft = 1100.0 / (double)SAMPLE_RATE;
 * As = 67.0;
 * mu = 0.0;
 *
 * liquid_firdes_kaiser(lpf_h_len, fc, As, mu, lpf_h);
 */
#define	MIX_LPF_TAPS    45
static float mixCoeffs[MIX_LPF_TAPS] = {
	0.001780, 0.003674, 0.005737, 0.006928, 0.005724, 0.000408,
	-0.010396, -0.027057, -0.048254, -0.070499, -0.088063, -0.093443,
	-0.078412, -0.035519, 0.040184, 0.149571, 0.288398, 0.447094,
	0.611552, 0.764879, 0.889843, 0.971570, 1.000000, 0.971570,
	0.889843, 0.764879, 0.611552, 0.447094, 0.288398, 0.149571,
	0.040184, -0.035519, -0.078412, -0.093443, -0.088063, -0.070499,
	-0.048254, -0.027057, -0.010396, 0.000408, 0.005724, 0.006928,
	0.005737, 0.003674, 0.001780
};

static float mixIDelay[MIX_LPF_TAPS];
static float mixQDelay[MIX_LPF_TAPS];
static fir_f32_t   mixIFir;
static fir_f32_t   mixQFir;


/*
 * Produced with Fc = 3000Hz (0.25 * Fs)
 * Length was experimentally determined and should
 * be tuned for possibly shorter lengths?
 */

#define	DIFF_TAPS	131

static float diffCoeffs[DIFF_TAPS] = {
	-0.002666, -0.003906, -0.002863, -0.000083, 0.002837, 0.004167,
	0.003061, 0.000095, -0.003032, -0.004464, -0.003289, -0.000109,
	0.003255, 0.004808, 0.003553, 0.000127, -0.003514, -0.005208,
	-0.003863, -0.000150, 0.003817, 0.005682, 0.004233, 0.000180,
	-0.004178, -0.006250, -0.004681, -0.000220, 0.004613, 0.006944,
	0.005235, 0.000275, -0.005150, -0.007812, -0.005937, -0.000354,
	0.005828, 0.008929, 0.006856, 0.000471, -0.006711, -0.010417,
	-0.008111, -0.000658, 0.007908, 0.012500, 0.009928, 0.000982,
	-0.009620, -0.015625, -0.012785, -0.001624, 0.012266, 0.020833,
	0.017931, 0.003183, -0.016863, -0.031250, -0.029847, -0.008842,
	0.026352, 0.062500, 0.083934, 0.079577, 0.048302, 0.000000,
	-0.048302, -0.079577, -0.083934, -0.062500, -0.026352, 0.008842,
	0.029847, 0.031250, 0.016863, -0.003183, -0.017931, -0.020833,
	-0.012266, 0.001624, 0.012785, 0.015625, 0.009620, -0.000982,
	-0.009928, -0.012500, -0.007908, 0.000658, 0.008111, 0.010417,
	0.006711, -0.000471, -0.006856, -0.008929, -0.005828, 0.000354,
	0.005937, 0.007812, 0.005150, -0.000275, -0.005235, -0.006944,
	-0.004613, 0.000220, 0.004681, 0.006250, 0.004178, -0.000180,
	-0.004233, -0.005682, -0.003817, 0.000150, 0.003863, 0.005208,
	0.003514, -0.000127, -0.003553, -0.004808, -0.003255, 0.000109,
	0.003289, 0.004464, 0.003032, -0.000095, -0.003061, -0.004167,
	-0.002837, 0.000083, 0.002863, 0.003906, 0.002666
};

static float iDiffDelay[DIFF_TAPS];
static float qDiffDelay[DIFF_TAPS];
static fir_f32_t   iDiffFir;
static fir_f32_t   qDiffFir;

/*
 * Delay FIRs have no coefficients
 */
static float iDelayDelay[DIFF_TAPS];
static float qDelayDelay[DIFF_TAPS];
static fir_f32_t   iDelayFir;
static fir_f32_t   qDelayFir;

/*
 * Data filter
 *
 * Produced with liquid-dsp:
 * _k = SAMPLES_BIT;
 * _m = 5;
 * data_h_len = 2 * _m * _k + 1;
 * data_h = malloc(data_h_len * sizeof(float));
 * liquid_firdes_prototype(LIQUID_FIRFILT_RKAISER, _k, _m, 0.750, 0, data_h);
 *
 * and subsequently normalized
 */

#define	DATA_TAPS	101

static float dataCoeffs[DATA_TAPS]= {
	-0.000003, -0.000007, -0.000010, -0.000012, -0.000010, 0.000000,
	0.000022, 0.000056, 0.000102, 0.000153, 0.000200, 0.000226,
	0.000213, 0.000141, -0.000004, -0.000226, -0.000515, -0.000838,
	-0.001146, -0.001369, -0.001429, -0.001251, -0.000776, 0.000016,
	0.001094, 0.002365, 0.003671, 0.004797, 0.005498, 0.005523,
	0.004668, 0.002811, -0.000037, -0.003708, -0.007855, -0.011969,
	-0.015411, -0.017471, -0.017449, -0.014747, -0.008957, 0.000060,
	0.012121, 0.026703, 0.042971, 0.059831, 0.076039, 0.090316,
	0.101485, 0.108599, 0.111041, 0.108599, 0.101485, 0.090316,
	0.076039, 0.059831, 0.042971, 0.026703, 0.012121, 0.000060,
	-0.008957, -0.014747, -0.017449, -0.017471, -0.015411, -0.011969,
	-0.007855, -0.003708, -0.000037, 0.002811, 0.004668, 0.005523,
	0.005498, 0.004797, 0.003671, 0.002365, 0.001094, 0.000016,
	-0.000776, -0.001251, -0.001429, -0.001369, -0.001146, -0.000838,
	-0.000515, -0.000226, -0.000004, 0.000141, 0.000213, 0.000226,
	0.000200, 0.000153, 0.000102, 0.000056, 0.000022, 0.000000,
	-0.000010, -0.000012, -0.000010, -0.000007, -0.000003
};

static float dataDelay[DATA_TAPS];
static fir_f32_t   dataFir;

/*
 * axmodem prototype
 * discriminator version
 */

static float A0[DSP_BLOCK_LEN];
static float A1[DSP_BLOCK_LEN];
static float A2[DSP_BLOCK_LEN];
static float A3[DSP_BLOCK_LEN];
static float B0[DSP_BLOCK_LEN];
static float B1[DSP_BLOCK_LEN];

/*********************************************************************/

/*
 *
 */
IRAM_ATTR static void
setDataCarrier(int on) {
	if (dataCarrier && !on) {
		dataCarrier = false;
	} else if (!dataCarrier && on) {
		dataCarrier = true;
	}
}

/*
 *
 */
IRAM_ATTR
static void
process_rx_byte(uint_fast16_t rxb) {
    static uint8_t rxBuffer[KB_MAX_FRAME_SIZE];
    static uint16_t rxIndex;
    static uint16_t rxFcs;

	if (rxb == AX_START_FRAME) {
        /* reset frame buffer */
        rxIndex = 0;
        rxFcs = 0xffff;
		return;
	}

	if (rxb == AX_END_FRAME) {
		if ((rx_state == RX_DECODING)) {
			frame++;

            if ((rxIndex >= KB_MIN_FRAME_SIZE) &&
              (rxFcs == VALID_FCS)) {
                kbuf_t *fbFrame;

                good_frame++;
                /* drop FCS from frame */
                fbFrame = kbuf_alloc(rxIndex - 2, rxBuffer);
                if (fbFrame != NULL) {
                    if (xQueueSend(rxFrameQueue, &fbFrame, 0) != pdPASS) {
                        // drop frame
                        dropped_frame++;
                        kbuf_free(fbFrame);
                        puts("** RX frame drop 2 **");
                    }
                } else {
                    puts("** RX frame drop 1 **");
                    dropped_frame++;
                }
            }
		}
		return;
	}

    if (rxIndex < KB_MAX_FRAME_SIZE) {
        rxBuffer[rxIndex++] = rxb;
        fcs_update(&rxFcs, rxb);
    } else {
        // XXX: overrun; FCS should cause this to be discarded
    }
}

#define PROCESS_RX_BIT(bit)     \
        { \
            data_register = (data_register >> 1) | (bit ? 0x80 : 0x00); \
	        if (++bitcount == 8) { \
                bitcount = 0; \
                if (dataCarrier) { \
                	if ((framebytes < 13) && (data_register & 1)) { \
                		process_rx_byte(AX_END_FRAME); \
                        setDataCarrier(0); \
                        rx_state = RX_WAITING; \
					} else { \
						process_rx_byte(data_register); \
						framebytes++; \
					} \
                } \
            } \
        }

#define BH_CMP(x, y, z) (((x) & (z)) == ((y) & (z)))
#define BH_IDLE         0x17171717UL
#define BH_IDLE_MASK    0x000000ffUL
#define BH_TT4          0x11111111UL
#define BH_TT4_MASK     0xffffffffUL

static int8_t dcdMissBits = 6;

IRAM_ATTR
static void
process_bit_period(uint32_t bits) {
	static uint_fast8_t data_register = 0;
	static uint_fast8_t bitcount = 0;
	static uint32_t framebytes = 0;
	static uint32_t bh = 0;
	static int_fast8_t dcd_miss;
	uint32_t i;

	/*
	 * bits is a total count of 1s followed by a single 0
	 * Special cases:
	 *      0: the DPLL never produces this
	 *      1: a single '0' bit
	 *      6: bit-stuffed sequence; no 0 to follow
	 *      7: partial-flag
	 *      8 or more: ABORT sequence
	 */
	/* Ignore a zero-count; never happens because DPLL samples */
	if (bits == 0) {
		return;
	}

	if (bits > 7) {
		/* abort */
		process_rx_byte(AX_END_FRAME);
		if (rx_state != RX_WAITING) {
			setDataCarrier(0);
			rx_state = RX_WAITING;
		}

		bh <<= 4;
		return;
	}

	/*
	 * Update bh; this is used for DCD below
	 */
	bh = (bh << 4) | bits;

	switch (rx_state) {
	case RX_WAITING:
		if ((bits == 1) && (BH_CMP(bh, BH_TT4, BH_TT4_MASK))) {
			setDataCarrier(1);
			rx_state = RX_DCD;
			dcd_miss = dcdMissBits;
		} else if ((bits == 7) && (BH_CMP(bh, BH_IDLE, BH_IDLE_MASK))) {
			setDataCarrier(1);
			rx_state = RX_DECODING;
			process_rx_byte(AX_START_FRAME);
			framebytes = bitcount = 0;
		}
		break;

	case RX_DCD:
		if (bits == 7) {
			/* found a flag */
			setDataCarrier(1);
			rx_state = RX_DECODING;
			process_rx_byte(AX_START_FRAME);
			framebytes = bitcount = 0;
		} else if ((bits != 1) && (--dcd_miss <= 0)) {
			/* anything but TT preamble or flag aborts */
			setDataCarrier(0);
			rx_state = RX_WAITING;
		} else if ((bits == 1) && (dcd_miss < dcdMissBits)) {
			dcd_miss++;
		}
		break;

	case RX_DECODING:
		if (bits == 7) {
			/*
			 * a single flag ends a frame and starts another
			 * If no bytes have been processed, no need to
			 * reset the frame
			 */
			if (framebytes > 0) {
				process_rx_byte(AX_END_FRAME);
				process_rx_byte(AX_START_FRAME);
				framebytes = 0;
			}
			bitcount = 0;
		} else {
			/* actually decode the bits */
			bits -= 1; /* adjust to count of 1s */
			for (i = 0; i < bits; i++) {
				PROCESS_RX_BIT(1);
			}

			/* drop the 0 after 5 1s */
			if (bits < 5) {
				PROCESS_RX_BIT(0);
			}
		}
		break;
	}
}


/*
 *
 */
IRAM_ATTR
void
modemProcessInput(float *a) {
	static uint64_t last_t_sample = 0;
	static uint_fast8_t d_b0 = 0, d_b1 = 0, raw_b0 = 0, raw_b1;
	static int d_pll = 0.0;

	int dpll_error;
	// osEvent evt;
	uint32_t t_period, bit_period;
	uint_fast16_t i, aborted = 0;
    // int cc;

	/*
	 * BPF input block in-place
	 */
    dsps_fir_f32_ae32(&bpfFir, a, a, DSP_BLOCK_LEN);

	/*
	 * incoming audio to baseband conversion
	 */
	for (i = 0; i < DSP_BLOCK_LEN; i++) {
		/* Mix the incoming audio down to IQ baseband */
        A0[i] = a[i] * arm_cos_f32(mixLoPhase);
		A2[i] = a[i] * arm_sin_f32(mixLoPhase);

		/* Roll-over at 2*Pi radians */
		mixLoPhase += MIX_LO_INCR;
		if (mixLoPhase >= (float)(2.0 * M_PI)) {
			mixLoPhase -= (float)(2.0 * M_PI);
		}
	}

    (void)dsps_fir_f32_ae32(&mixIFir, A0, A0, DSP_BLOCK_LEN);   // A0: I
    (void)dsps_fir_f32_ae32(&mixQFir, A2, A2, DSP_BLOCK_LEN);   // A0: Q
    (void)dsps_fir_f32_ae32(&iDiffFir, A0, A1, DSP_BLOCK_LEN);   // A1: Idiff
    (void)delay_fir_f32_ansi(&iDelayFir, A0, A0, DSP_BLOCK_LEN);   // A0: Idelay
    (void)dsps_fir_f32_ae32(&qDiffFir, A2, A3, DSP_BLOCK_LEN);   // A3: Qdiff
    (void)delay_fir_f32_ansi(&qDelayFir, A2, A2, DSP_BLOCK_LEN);   // A2: Qdelay

    (void)dsps_mul_f32_ae32(A0, A3, B0, DSP_BLOCK_LEN, 1, 1, 1); // B0: Idelay * Qdiff
    (void)dsps_mul_f32_ae32(A2, A1, B1, DSP_BLOCK_LEN, 1, 1, 1); // B0: Qdelay * Idiff

    (void)dsps_sub_f32_ae32(B0, B1, B0, DSP_BLOCK_LEN, 1, 1, 1); // B0: unscaled out
    (void)dsps_mul_f32_ae32(A0, A0, A0, DSP_BLOCK_LEN, 1, 1, 1); // A0: Idelay^2
    (void)dsps_mul_f32_ae32(A2, A2, A2, DSP_BLOCK_LEN, 1, 1, 1); // A2: Qdelay^2
    (void)dsps_add_f32_ae32(A0, A2, A0, DSP_BLOCK_LEN, 1, 1, 1); // A0: mag(delay)

	// A0 is i^2 + q^2
	for (i = 0; i < DSP_BLOCK_LEN; i++) {
		/* avoid division by zero */
		if (A0[i] < 1e-12f) {
			B0[i] = 0.0f;
		} else {
			B0[i] /= A0[i];
		}
	}

	/* recovered data filter */
    (void)dsps_fir_f32_ae32(&dataFir, B0, B0, DSP_BLOCK_LEN);

	/*
	 * Slice and decode bit-stream
	 */
	for (i = 0; i < DSP_BLOCK_LEN; i++) {

		raw_b1 = (B0[i] >= 0.0f);

		/*
		 * Simple Digital PLL for data recovery
		 */
		if (raw_b0 != raw_b1) {
			dpll_error = d_pll - (D_PLL_MAX / 2);
			if (dpll_error > (D_PLL_MARGIN)) {
			    d_pll -= dataCarrier ? D_PLL_MARGIN :
			      (dpll_error + D_ERR_DIV / 2) / D_ERR_DIV;
			} else if (dpll_error < (-D_PLL_MARGIN)) {
			    d_pll += dataCarrier ? D_PLL_MARGIN :
			      (dpll_error + D_ERR_DIV / 2) / D_ERR_DIV;
			}
		}

		d_pll += D_PLL_INC;

		if (d_pll >= D_PLL_MAX) {
			/* slice */
			d_b1 = raw_b1;
			t_period = t_sample - last_t_sample;
			if ((d_b1 != d_b0)) {
				aborted = 0;
				last_t_sample = t_sample;
				bit_period = (t_period + SAMPLES_BIT / 2) / SAMPLES_BIT;
				process_bit_period(bit_period);
			} else if (t_period >= (SAMPLES_BIT * 8)) {
				/* insert an ABORT after 8 bit period */
				if (!aborted) {
					process_bit_period(8);
					aborted = 1;
				}
			}
			d_b0 = d_b1;
			d_pll -= D_PLL_MAX;
		}

		raw_b0 = raw_b1;
		t_sample++;
	}
}

/*
 * Main interface to queue outbound frame
 */
void
transmitFrame(kbuf_t *fp)
{

    if (xQueueSend(txFrameQueue, &fp, 0) !=
      pdPASS) {
        // XXX: dropped KISS frame
        puts("** KISS frame drop 2 **");
        kbuf_free(fp);
    }
}

/*
 * KISS interface
 */
static TaskHandle_t rxTaskHandle;
static TaskHandle_t kissTaskHandle;

enum { KISS_FEND = 0xc0, KISS_FESC = 0xdb, KISS_TFEND = 0xdc, KISS_TFESC = 0xdd };


/*
 * Process incoming (from host to TNC) KISS bytes
 */
IRAM_ATTR static void
processKissRx(uint8_t c)
{
    static uint8_t kissRxBuffer[KB_MAX_FRAME_SIZE + 1];
    static uint16_t kissRxIndex;
    static bool kissRxEscaped = false;
    kbuf_t *kissRxFrame;

    /*
     * FEND handling
     */
    if (c == KISS_FEND) {
        /*
         * Discard zero-length frame
         */
        if (kissRxIndex == 0) {
            kissRxEscaped = false;
            return;
        }

        switch (kissRxBuffer[0]) {
        case 0:
            /* data frame */

            /*
             * Account for command byte in buffer
             */
            if (kissRxIndex >= (KB_MIN_FRAME_SIZE + 1)) {
                kissRxFrame = kbuf_alloc(kissRxIndex - 1, kissRxBuffer + 1);
                if (kissRxFrame != NULL) {
#if 1
                	tnc2buf_t *tb = ax25ToTNC2(kissRxFrame);

                	if (tb) {
                		printf("TX: %s>%s", tb->srcAddr, tb->destAddr);
                		if (tb->digiPath) {
                			printf(",%s", tb->digiPath);
                		}
                		printf(":%.*s\n", tb->msgLen, tb->msg);
                		freeTNC2Buf(tb);
                	}
#endif
                	transmitFrame(kissRxFrame);
                } else {
                    puts("** KISS frame drop 1 **");
                    // XXX: dropped KISS frame
                }
            }
            break;

        default:
            // handle control frames
            if (kissRxIndex > 1) {
                switch (kissRxBuffer[0]) {
                case 1:
// XXX:					kissSetTxDelay(kissRxBuffer[1]);
                    break;
                case 2:
// XXX:					kissSetPPersist(kissRxBuffer[1]);
                    break;
                case 3:
// XXX:					kissSetSlotTime(kissRxBuffer[1]);
                    break;
                case 4:
// XXX:					kissSetFullDuplex(kissRxBuffer[1]);
                    break;
                case 5:
// XXX:                 kissStartCalibration(kissRxBuffer[1]);
                    break;

                default:
                    break;
                }
            }
            break;
        }

        /*
         * reset for next frame
         */
        kissRxIndex = 0;
        kissRxEscaped = false;
        return;
    }

    /*
     * Escape sequence handling
     */
    if (kissRxEscaped) {
        kissRxEscaped = false;
        if (c == KISS_TFEND) {
            c = KISS_FEND;
        } else if (c == KISS_TFESC) {
            c = KISS_FESC;
        } else {
            /* invalid, ignore */
            return;
        }
    } else if (c == KISS_FESC) {
        kissRxEscaped = true;
        return;
    }

    /*
     * Data handling
     */
    if (kissRxIndex < (KB_MAX_FRAME_SIZE + 1)) {
        kissRxBuffer[kissRxIndex++] = c;
    }
}


//#define  KISS_SERIAL_HOST
#define  KISS_TCP_HOST

#ifdef  KISS_SERIAL_HOST
/*
 *
 */
IRAM_ATTR static void
kissProcessHost(void *x)
{
    uint8_t rbuff;

    (void)uart_flush(UART_NUM_0);

    while (1) {
        /*
         * ESP-IDF apparently requires multiple threads to release
         * the UART to give each other a turn, so timeout 20mS
         * and send up 0xC0 KISS frame terminators
         */
        int count = uart_read_bytes(UART_NUM_0, &rbuff, 1,
          20 / portTICK_PERIOD_MS);
        processKissRx(count > 0 ? rbuff : KISS_FEND);
    }

}

/*
 * Process received frames; currently just KISS out via serial port
 */
IRAM_ATTR static void
rxProcessFrame(void *x)
{
    kbuf_t *fp;
    size_t i;

    while (1) {
        /* assume valid return */
        (void)xQueueReceive(rxFrameQueue, &fp, portMAX_DELAY);

        uart_write_bytes(UART_NUM_0, "\xc0\x00", 2);
        i = kbuf_len(fp);
        while (i--) {
            char c;
           
            c = kbuf_get_8(fp);
            switch (c) {
            case KISS_FEND:
                uart_write_bytes(UART_NUM_0, "\xdb\xdc", 2);
                break;
            case KISS_FESC:
                uart_write_bytes(UART_NUM_0, "\xdb\xdd", 2);
                break;
            default:
                uart_write_bytes(UART_NUM_0, &c, 1);
                break;
            }
        }
        uart_write_bytes(UART_NUM_0, "\xc0", 1);
        kbuf_free(fp);
    }
}
#endif /* KISS_SERIAL_HOST */


#ifdef  KISS_TCP_HOST

/*
 * XXX: refactor this!
 */

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"

/*
 * KISS host socket
 */
static int hostSocket = -1;
static SemaphoreHandle_t kissHostMutex;

/*
 * get/set KISS Host Socket under a mutex
 */
static int
getKissHostSocket()
{
	int socket;

	xSemaphoreTake(kissHostMutex, portMAX_DELAY);
	socket = hostSocket;
	xSemaphoreGive(kissHostMutex);
	return (socket);
}

static void
setKissHostSocket(int socket)
{

	xSemaphoreTake(kissHostMutex, portMAX_DELAY);
	hostSocket = socket;
	xSemaphoreGive(kissHostMutex);
}

/*
 *
 */
IRAM_ATTR static void
kissProcessHost(void *x)
{
    static uint8_t rxBuffer[64];
    int len;

    struct sockaddr_in destAddr;
    struct sockaddr_in6 sourceAddr;
    uint addrLen;
    int listenSock, acceptSock, err;
    int addrFamily, ipProtocol;

    /* XXX: add support for IPv6 */
    addrFamily = AF_INET;
    ipProtocol = IPPROTO_IP;
    destAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    destAddr.sin_family = addrFamily;
    destAddr.sin_port = htons(8001);    // XXX: listen port

    processKissRx(KISS_FEND);
    processKissRx(KISS_FEND);

    /* listen for incoming connection */
    listenSock = socket(addrFamily, SOCK_STREAM, ipProtocol);
    if (listenSock < 0) {
        ESP_LOGE(TAG, "unable to create socket: %d", errno);
        return;
    }

    err = bind(listenSock, (struct sockaddr *) &destAddr, sizeof (destAddr));
    if (err) {
        ESP_LOGE(TAG, "socket bind failed: %d", errno);
        return;
    }

    err = listen(listenSock, 1);
    if (err) {
        ESP_LOGE(TAG, "socket listen failed: %d", errno);
        return;
    }

    while (true) {
        addrLen = sizeof(sourceAddr);
        acceptSock = accept(listenSock,
          (struct sockaddr *) &sourceAddr, &addrLen);

        if (acceptSock < 0) {
            ESP_LOGE(TAG, "unable to accept connection: %d", errno);
            continue;
        }

        setKissHostSocket(acceptSock);

        while (true) {
            len = recv(getKissHostSocket(), rxBuffer, sizeof (rxBuffer), 0);
            if (len < 0) {
                ESP_LOGE(TAG, "recv() failed: %d", errno);
                // XXX: close socket?
                close(getKissHostSocket());
                setKissHostSocket(-1);
                processKissRx(KISS_FEND);
                processKissRx(KISS_FEND);
                ESP_LOGI(TAG, "Host TCP connection closed");
                break;
            } else if (len == 0) {
            	// XXX: close socket?
            	close(getKissHostSocket());
            	setKissHostSocket(-1);
                processKissRx(KISS_FEND);
                processKissRx(KISS_FEND);
                ESP_LOGI(TAG, "Host TCP connection closed");
                break;
            } else {
                /* data received */
                for (int i = 0; i < len; i++) {
                    processKissRx(rxBuffer[i]);
                }
            }
        }
    }
}

/*
 * Process received frames out via TCP
 */

#define SEND_HOST(x)   { \
                    txBuffer[txIndex++] = (x); \
                    if (txIndex >= sizeof (txBuffer)) { \
                        send(getKissHostSocket(), txBuffer, \
                          sizeof (txBuffer), 0); \
                        txIndex = 0; \
                    } \
                }

IRAM_ATTR static void
rxProcessFrame(void *x)
{
    static uint8_t txBuffer[128];
    static int txIndex;
    kbuf_t *fp;
    size_t i;

    while (1) {
        /* assume valid return */
        (void)xQueueReceive(rxFrameQueue, &fp, portMAX_DELAY);

        /* send up to IS (also digipeats) */
        iGateToIS(fp);

        /* discard frames until host connects */
        if (getKissHostSocket() < 0) {
            kbuf_free(fp);
            continue;
        }

        SEND_HOST(KISS_FEND);
        SEND_HOST(0x00);

        i = kbuf_len(fp);
        while (i--) {
            char c;
           
            c = kbuf_get_8(fp);
            switch (c) {
            case KISS_FEND:
                SEND_HOST(KISS_FESC);
                SEND_HOST(KISS_TFEND);
                break;
            case KISS_FESC:
                SEND_HOST(KISS_FESC);
                SEND_HOST(KISS_TFESC);
                break;
            default:
                SEND_HOST(c);
                break;
            }
        }

        // send(hostSocket, "\xc0", 1, 0);
        SEND_HOST(KISS_FEND);

        /* flush buffer */
        if (txIndex > 0) {
            send(getKissHostSocket(), txBuffer, txIndex, 0);
            txIndex = 0;
        }

        kbuf_free(fp);
    }
}
#endif  /* KISS_TCP_HOST */


/*
 *
 */
void
modemInitialize()
{
	BaseType_t err;

    (void)dsps_fir_init_f32(&bpfFir, bpfCoeffs, bpfDelay, BPF_FIR_LEN);
    (void)dsps_fir_init_f32(&mixIFir, mixCoeffs, mixIDelay, MIX_LPF_TAPS);
    (void)dsps_fir_init_f32(&mixQFir, mixCoeffs, mixQDelay, MIX_LPF_TAPS);
    (void)dsps_fir_init_f32(&iDiffFir, diffCoeffs, iDiffDelay, DIFF_TAPS);
    (void)dsps_fir_init_f32(&qDiffFir, diffCoeffs, qDiffDelay, DIFF_TAPS);
    (void)dsps_fir_init_f32(&iDelayFir, NULL, iDelayDelay, DIFF_TAPS);
    (void)dsps_fir_init_f32(&qDelayFir, NULL, qDelayDelay, DIFF_TAPS);
    (void)dsps_fir_init_f32(&dataFir, dataCoeffs, dataDelay, DATA_TAPS);

    /*
     * Received frame queue
     */
    rxFrameQueue = xQueueCreate(RXF_QUEUE_LEN, sizeof (kbuf_t *));

    /*
     * Transmit frame queue
     */
    txFrameQueue = xQueueCreate(TXF_QUEUE_LEN, sizeof (kbuf_t *));

    /*
     * KISS Host socket handle
     */
    kissHostMutex = xSemaphoreCreateMutex();

    /*
     * Received frame processing task
     */
    err = xTaskCreate(rxProcessFrame, "rxFrame",
      ESP_TASK_MAIN_STACK,
      ( void * ) 1,
      ESP_TASK_PRIO_MAX - 2,
	  &rxTaskHandle);

    /*
     * Host KISS frame processing task
     */
    err = xTaskCreate(kissProcessHost, "kissHost",
      ESP_TASK_MAIN_STACK,
      ( void * ) 1,
      ESP_TASK_PRIO_MAX - 2,
	  &kissTaskHandle);

}


/*
 * Input and output tasks
 */

#define HDLC_FLAG   (0x7e)

/*
 * Transmitter state
 */
#define TX_IDLE         0       // inactive
#define TX_SLOT_WAIT    1       // frame ready, waiting slot time
#define TX_DELAY        2       // transmitter active, frame ready, flags
#define TX_START_FLAG   3       // send 1 start flag, reset CSUM
#define TX_DATA         4       // send bit-stuffed data
#define TX_FCS_1        5       // send FCS byte #1
#define TX_FCS_2        6       // send FCS byte #2
#define TX_END_FLAG     7       // send end flag(s)
#define TX_TAIL         8       // transmitter inactive, send mute

#define TX_CAL_LOW      9       // CALIBRATE low
#define TX_CAL_HIGH     10      // CALIBRATE high
#define TX_CAL_ALT      11      // CALIBRATE alternating tones

#define TX_BIT_MUTE     (-1)
#define TX_BIT_LOW      0
#define TX_BIT_HIGH     1


/*
 * Transmitter state
 */
static int txState = TX_IDLE;
static uint8_t txPersistVal = 32;
static uint8_t txSlotTimeVal = 120;    // slot time in bits 
static uint8_t txDelayVal = 45;        // txDelay in flag bytes
static bool txReleasePTT;

/*
 *
 */
static kbuf_t *
getTxFrame()
{
    kbuf_t *fp = NULL;

    (void)xQueueReceive(txFrameQueue, &fp, (TickType_t) 0);
    return (fp);
}

/*
 *
 */
static bool
txPersistOk()
{
    return ((esp_random() & 0xff) < txPersistVal);
}

/*
 * NRZI machine, bit-stuffing
 */
static int
txNextBit(uint8_t *bits, uint8_t *count, bool stuff)
{
    static bool bitState = false;
    static bool bitStuffed = false;
    static uint8_t stuffCount;
    bool b;

    /*
     * Stuff a zero-bit if needed
     */
    if (bitStuffed) {
        bitStuffed = false;
        bitState = !bitState;
        return (bitState);
    }

    /*
     * If bits are all shifted, indicate end
     */
    if ((*count)-- == 0) {
        return (TX_BIT_MUTE);
    }

    /*
     * get the bit to send
     */
    b = (*bits & 0x01) != 0;
    *bits >>= 1;

    /*
     * Stuff a zero-bit if needed
     */
    if (stuff && b) {
        if (++stuffCount >= 5) {
            stuffCount = 0;
            bitStuffed = true;  // stuff a bit next time
        }
    } else {
        /* reset stuffCount if not stuffing */
        stuffCount = 0;
    }

    /* update NRZI bit */
    if (!b) {
        bitState = !bitState;
    }

    return (bitState ? TX_BIT_HIGH : TX_BIT_LOW);
}

/*
 * Frame transmission is done one bit a time
 *
 * This function is called once per bit, returns the next bit
 * to transmit. State changes take-effect on the next bit, so
 * the START/END flag states are used to de-couple data-handling
 * from flag handling.
 *
 */
static int
getTxBit()
{
    static kbuf_t *curFrame;
    static uint16_t curFcs;
    static uint16_t slotTimer;
    static uint16_t flagTimer;

    static uint8_t  txBits;
    static uint8_t  bitCount;

    int b = TX_BIT_MUTE;
    static bool calBit;

    switch (txState) {
    case TX_IDLE:
        if ((curFrame = getTxFrame()) == NULL) {
            // nothing to transmit
            return (TX_BIT_MUTE);
        }

        /*
         * Fall into SLOT_WAIT with expired slot-timer,
         * which immediately attempts to jump on the channel.
         */
        slotTimer = 0;
        txState = TX_SLOT_WAIT;
        __attribute__ ((fallthrough));

    case TX_SLOT_WAIT:
        if (slotTimer-- > 0) {
            /* timer not expired */
            return (TX_BIT_MUTE);
        }

        /*
         * slot time expired; attempt transmit
         */
        if (dataCarrier || !txPersistOk()) {
            /* restart slot timer */
            slotTimer = txSlotTimeVal;
            return (TX_BIT_MUTE);
        }

        // assert PTT
        gpio_set_level(GPIO_NUM_23, 1);

        /*
         * Send FLAGs for TX_DELAY period - falls into
         * TX_DELAY
         */
        flagTimer = txDelayVal;
        txState = TX_DELAY;
        txBits = HDLC_FLAG;
        bitCount = 8;
        __attribute__ ((fallthrough));

    case TX_DELAY:
        b = txNextBit(&txBits, &bitCount, false);
        if (b == TX_BIT_MUTE) {
            /*
             * Always start another flag; it's either
             * the next TX_DELAY flag, or it's the START_FLAG.
             * This avoids handling frame data from *this* state
             */
            txBits = HDLC_FLAG;
            bitCount = 8;
            b = txNextBit(&txBits, &bitCount, false);

            /* if TX_DELAY is done, this is START_FLAG */
            if (flagTimer-- == 0) {
                txState = TX_START_FLAG;
            }
        }

        return (b);

    case TX_START_FLAG:
        b = txNextBit(&txBits, &bitCount, false);
        if (b != TX_BIT_MUTE) {
            return (b);
        }

        /*
         * Fall into TX_DATA with empty txBits register to
         * start data transmission
         */
        txState = TX_DATA;
        bitCount = 0;
        curFcs = kbuf_fcs(curFrame);
        __attribute__ ((fallthrough));

    case TX_DATA:
        b = txNextBit(&txBits, &bitCount, true);
        if (b == TX_BIT_MUTE) {
            int16_t d;
            // get next byte
            d = kbuf_get_8(curFrame);
            if (d < 0) {
                /* start FCS byte 1 */
                txBits = curFcs & 0xff;
                curFcs >>= 8;
                bitCount = 8;
                txState = TX_FCS_1;
                b = txNextBit(&txBits, &bitCount, true);

            } else {
                // start next byte
                txBits = d;
                bitCount = 8;
                b = txNextBit(&txBits, &bitCount, true);
            }
            // always leave here with non-MUTE in b
        }
        return (b);

    case TX_FCS_1:
    case TX_FCS_2:
        b = txNextBit(&txBits, &bitCount, true);
        if (b == TX_BIT_MUTE) {
            if (txState == TX_FCS_1) {
                /* start FCS byte 2 */
                txBits = curFcs & 0xff;
                bitCount = 8;
                txState = TX_FCS_2;
                b = txNextBit(&txBits, &bitCount, true);
            } else {
                /* frame done, start END_FLAG(s) */
                flagTimer = 2;
                txBits = HDLC_FLAG;
                bitCount = 8;
                txState = TX_END_FLAG;
                b = txNextBit(&txBits, &bitCount, false);
            }
        }
        break;

    case TX_END_FLAG:
        b = txNextBit(&txBits, &bitCount, false);
        if (b == TX_BIT_MUTE && flagTimer-- > 0) {
            txBits = HDLC_FLAG;
            bitCount = 8;
            b = txNextBit(&txBits, &bitCount, false);
        }

        if (b != TX_BIT_MUTE) {
            return (b);
        }

        /*
         * frame finished; free and start next if available
         */
        kbuf_free(curFrame);
        if ((curFrame = getTxFrame()) == NULL) {
            // nothing to transmit
            // turn off PTT
            txReleasePTT = true;
            txState = TX_IDLE;
        } else {
            txState = TX_START_FLAG;
            txBits = HDLC_FLAG;
            bitCount = 8;
            b = txNextBit(&txBits, &bitCount, false);
        }

        return (b);

    case TX_CAL_LOW:
        return (TX_BIT_LOW);

    case TX_CAL_HIGH:
        return (TX_BIT_HIGH);

    case TX_CAL_ALT:
        calBit = !calBit;
        return (calBit ? TX_BIT_HIGH : TX_BIT_LOW);

    default:
        return (TX_BIT_MUTE);
    }

    return (b);
}


/*
 * DSP output task
 * Continuously generates audio output; when
 * not actually transmitting, generates 0.0V
 *
 */

float outputAmplitude = 0.6;

#define	TX_LOW_INCR	    ((float)((2.0 * M_PI * 1200.0) / (double)SAMPLE_RATE))
#define	TX_HIGH_INCR	((float)((2.0 * M_PI * 2200.0) / (double)SAMPLE_RATE))

IRAM_ATTR void
dspOutProcess(void *x)
{
    size_t bytes_written;
    float   phase = 0.0f, phaseIncr;
    int b;
    int txIndex = 0;


    while (1) {
        b = getTxBit();

        if (b < 0) {
            /*
             * Mute: set phase for zero crossing
             * and phaseIncr to 0.0
             */
            phase = (float)(M_PI + M_PI / 2.0);
            phaseIncr = 0.0f;
        } else if (b == 0) {
            phaseIncr = TX_LOW_INCR;
        } else {
            phaseIncr = TX_HIGH_INCR;
        }

        // fill txBuffer with samples_bit samples
        for (int i = 0; i < SAMPLES_BIT; i++) {
            int16_t v;
           
            v = (int16_t) (arm_cos_f32(phase) * 32767.0 * outputAmplitude);
            phase += phaseIncr;
            if (phase >= (float)(2.0 * M_PI)) {
                phase -= (float)(2.0 * M_PI);
            }

            /*
             * XXX: AC101 specific: one 'frame' is 4x frames
             */
            txBuffer[txIndex * 8 + 0] = v;
            txBuffer[txIndex * 8 + 1] = v;
            txBuffer[txIndex * 8 + 2] = v;
            txBuffer[txIndex * 8 + 3] = v;
            txBuffer[txIndex * 8 + 4] = v;
            txBuffer[txIndex * 8 + 5] = v;
            txBuffer[txIndex * 8 + 6] = v; // R (ring)
            txBuffer[txIndex * 8 + 7] = v; // L (tip)

#if 0
            // XXX: mute the R (ring) channel
            txBuffer[txIndex * 8 + 0] = 0;
            txBuffer[txIndex * 8 + 2] = 0;
            txBuffer[txIndex * 8 + 4] = 0;
            txBuffer[txIndex * 8 + 6] = 0; // R (ring)
#endif

#if 1
            // XXX: mute the L (ring) channel
            txBuffer[txIndex * 8 + 1] = 0;
            txBuffer[txIndex * 8 + 3] = 0;
            txBuffer[txIndex * 8 + 5] = 0;
            txBuffer[txIndex * 8 + 7] = 0; // L (ring)
#endif
            /*
             * Write full txBuffer to I2S
             */
            if (++txIndex >= DSP_BLOCK_LEN) {
                txIndex = 0;

                (void)i2s_write(I2S_NUM_0, (char *)txBuffer, sizeof (txBuffer),
                  &bytes_written, 1000);

                if (bytes_written != sizeof (txBuffer)) {
                    ESP_LOGE("CODEC", "short codec write: %d", bytes_written);
                }
            }

        }

        /*
         * PTT release is deferred from the transmit state machine
         * to allow closing flags to be sent. XXX: see how many flags
         * make it out without delay; looks like 3 is good
         */
        if (txReleasePTT) {
            txReleasePTT = false;
            gpio_set_level(GPIO_NUM_23, 0);
        }
    }
}
