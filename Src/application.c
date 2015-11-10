/**
  ******************************************************************************
  * @file    application.c 
  * @author  Gustavo Muro
  * @version V0.0.1
  * @date    30/05/2015
  * @brief   Archivo de aplicación.
  ******************************************************************************
  * @attention
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted provided that the following conditions are met:
  *
  * 1. Redistributions of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  *
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  *
  * 3. Neither the name of the copyright holder nor the names of its
  *    contributors may be used to endorse or promote products derived from this
  *    software without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
  * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  * POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "application.h"
#include "ff.h"
#include "waveplayer.h"
#include "waverecorder.h"
#include "ff.h"    
#include "ff_gen_drv.h"
#include "usbh_diskio.h"
#include "main.h"
#include "utils.h"
#include "audioFilter.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
  APPSTATE_IDLE = 0,
  APPSTATE_GEN_SINE,
  APPSTATE_MOUNT_FS,
  APPSTATE_UMOUNT_FS,
  APPSTATE_WRITE,
  APPSTATE_PLAY,
}appState_enum;

/* Private define ------------------------------------------------------------*/

#define SINE_GEN_AUDIO_SAMPLE_RATE    8000

#define SINE_GEN_DURATION             10

#define SINE_GEN_1KHZ_LENGTH          (SINE_GEN_AUDIO_SAMPLE_RATE/1000)

#define SINE_GEN_500HZ_LENGTH         (SINE_GEN_AUDIO_SAMPLE_RATE/500)

/* Private variables ---------------------------------------------------------*/
static FATFS USBDISKFatFs;           /* File system object for USB disk logical drive */
static char USBDISKPath[4];          /* USB Host logical drive path */
static appState_enum appState = APPSTATE_IDLE;
static audioFilter_filterSel_enum filterSel = AUDIO_FILTER_FILTER_SEL_BAND_PASS;
static uint8_t usbConnected = 0;
static uint32_t tickstart = 0; //Para temporizaciones junto con la variable tickcount
static uint32_t nmax = 0;      //Cantidad de buffers que superaron el umbral

/* Variable used by FatFs*/
static FIL FileRead;
static FIL FileWrite;

static const int16_t sine_1khz_FS8khz[SINE_GEN_1KHZ_LENGTH] =
{
  0, 23169, 32767, 23169, 0, -23169, 32767, -23169
};

static const int16_t sine_500hz_FS8khz[SINE_GEN_500HZ_LENGTH] =
{
  0,12539,23169,30272,32767,30272,23169,12539,0,-12539,-23169,-30272,-32767,-30272,-23169,-12539
};



/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
int32_t getDataCB(int16_t *pBuff, int32_t length)
{
  UINT bytesread = 0;
  uint32_t tickcount; //Para temporizaciones junto con la variable estatica tickstart
  uint32_t i;         //Indice para recorrer pBuff y buscar el maximo
	int16_t ymax = 0;   //Valor maximo de las muestras del buffer
	char bResetClk = 0; //Para sincronizar el temp de 1 minuto

//  TickTock_Stop();
//  TickTock_Start();

  f_read(&FileRead, pBuff, length*sizeof(int16_t), (void *)&bytesread); 

  audioFilter_filter(pBuff, pBuff, length);
  
	//Buscamos el maximo en pBuff
	for (i = 0 ; i < length ; i++)
	{
    if (ymax < pBuff[i]) ymax = pBuff[i];
	}
  // printf("ymax: %u \n",ymax);  //Utilizado para ver ymax y elegir el umbral

	//Comparo ymax con el umbral 1500, si es mayor esta presente un tono, incremento nmax
	if (ymax > 1500)
	{
    nmax++;
    BSP_LED_On(LED4);
   //Si nmax es mayor a 4 estoy en el tono largo
	  if (nmax > 4)
		{
			BSP_LED_On(LED3);
			bResetClk = 1;    //Para sincronizar el temp de 1 minuto
		}
	}
	else
	{
		nmax=0;
    bResetClk = 0;
    BSP_LED_Off(LED3);
    BSP_LED_Off(LED4);
	}

	//Uso del teclado para sincronizar el temp de 1 minuto
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) bResetClk = 1;
	
	//Temporizador de 1 minuto
	tickcount = HAL_GetTick();
	if (((tickcount - tickstart) > 60000) | bResetClk)
	{
		tickstart = tickcount;
	}
	//Si estoy dentro del 1er segundo enciendo LED5
	if ((tickcount - tickstart) < 1000) BSP_LED_On(LED5);
  else BSP_LED_Off(LED5);

  return bytesread;
}

int32_t getDataSineCB(int16_t *pBuff, int32_t length)
{
  static int8_t count = 0;
  int32_t ret = length * 2;
  
  TickTock_Start();
  
  while (length)
  {
    *pBuff = sine_500hz_FS8khz[count];
    count++;
    if (SINE_GEN_500HZ_LENGTH <= count)
    {
      count = 0;
    }
    pBuff++;
    length--;
  }
  
  TickTock_Stop();
  
  return ret;
}


/* Exported functions ------------------------------------------------------- */

extern void application_init(void)
{
  /*##-1- Link the USB Host disk I/O driver ##################################*/
  if(FATFS_LinkDriver(&USBH_Driver, USBDISKPath) != 0)
  {
    Error_Handler();
  }
  
  TickTock_Init();
  
  audioFilter_init();
}

extern void application_task(void)
{
  UINT bytesread = 0;
  WAVE_FormatTypeDef waveformat;
  
  switch (appState)
  {
    case APPSTATE_IDLE:
      if (usbConnected)
      {
        appState = APPSTATE_MOUNT_FS;
      }
      break;
    
    case APPSTATE_GEN_SINE:
      waveformat.SampleRate = SINE_GEN_AUDIO_SAMPLE_RATE;
      waveformat.FileSize = SINE_GEN_AUDIO_SAMPLE_RATE * SINE_GEN_DURATION * \
                            sizeof(int16_t) + sizeof(WAVE_FormatTypeDef);
      waveformat.NbrChannels = CHANNEL_MONO;
      WavePlayerStart(waveformat, getDataSineCB, 70);
      break;
    
    case APPSTATE_MOUNT_FS:
      if (f_mount(&USBDISKFatFs, (TCHAR const*)USBDISKPath, 0 ) != FR_OK ) 
      {
        /* FatFs initialization fails */
        Error_Handler();
      }
      else
      {
        appState = APPSTATE_PLAY;
      }
      break;
    
    case APPSTATE_UMOUNT_FS:
      f_mount(NULL, (TCHAR const*)"", 1);
      appState = APPSTATE_IDLE;
      break;
    
    case APPSTATE_WRITE:
      if (f_open(&FileWrite, WAVE_NAME_COMPLETO, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
      {
        Error_Handler();
      }
      else
      {
        waveformat.SampleRate = SINE_GEN_AUDIO_SAMPLE_RATE;
        waveformat.FileSize = SINE_GEN_AUDIO_SAMPLE_RATE * SINE_GEN_DURATION * \
                              sizeof(int16_t) + sizeof(WAVE_FormatTypeDef);
        waveformat.NbrChannels = WAVE_CHANNEL_MONO;
        waveformat.ByteRate = SINE_GEN_AUDIO_SAMPLE_RATE * WAVE_CHANNEL_MONO * sizeof(int16_t);
        waveformat.BitPerSample = __REV16(WAVE_16_BIT_PER_SAMPLE);
        waveformat.SubChunk2Size = SINE_GEN_AUDIO_SAMPLE_RATE * SINE_GEN_DURATION * sizeof(int16_t);
      
        WaveRecord(&FileWrite, waveformat, getDataSineCB);
        f_close(&FileWrite);
        appState = APPSTATE_PLAY;
      }
      break;

    case APPSTATE_PLAY:
      if (f_open(&FileRead, WAVE_NAME_COMPLETO, FA_READ) != FR_OK)
      {
        Error_Handler();
      }
      else
      {
				//Seleciono filtro
        //filterSel = AUDIO_FILTER_FILTER_SEL_HIGH_PASS;
        //audioFilter_filterSel(filterSel);

        /* Read sizeof(WaveFormat) from the selected file */
        f_read (&FileRead, &waveformat, sizeof(waveformat), &bytesread);
        WavePlayerStart(waveformat, getDataCB, 70);
        f_close(&FileRead);
        
			}
      break;
    
    default:
      appState = APPSTATE_IDLE;
      break;
  }
}

extern void application_conect(void)
{
  usbConnected = 1;
}
extern void application_disconect(void)
{
  usbConnected = 0;
}

/* End of file ---------------------------------------------------------------*/
