 /*
------------------------------------------------------------------------------
~ File   : pid.h
~ Author : Majid Derhambakhsh
~ Version: V1.0.0
~ Created: 02/11/2021 03:43:00 AM
~ Brief  :
~ Support:
		   E-Mail : Majid.do16@gmail.com (subject : Embedded Library Support)

		   Github : https://github.com/Majid-Derhambakhsh
------------------------------------------------------------------------------
~ Description:

~ Attention  :

~ Changes    :
------------------------------------------------------------------------------
*/

#ifndef __PIDSPEED_H_
#define __PIDSPEED_H_

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Include ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include <stdint.h>
#include <string.h>

/* ------------------------------------------------------------------ */

#ifdef __CODEVISIONAVR__  /* Check compiler */

#pragma warn_unref_func- /* Disable 'unused function' warning */

/* ------------------------------------------------------------------ */

#elif defined(__GNUC__) && !defined(USE_HAL_DRIVER)  /* Check compiler */

#pragma GCC diagnostic ignored "-Wunused-function" /* Disable 'unused function' warning */

/* ------------------------------------------------------------------ */

#elif defined(USE_HAL_DRIVER)  /* Check driver */

	#include "main.h"

	/* --------------- Check Mainstream series --------------- */

	#ifdef STM32F0
		#include "stm32f0xx_hal.h"       /* Import HAL library */
	#elif defined(STM32F1)
		#include "stm32f1xx_hal.h"       /* Import HAL library */
	#elif defined(STM32F2)
		#include "stm32f2xx_hal.h"       /* Import HAL library */
	#elif defined(STM32F3)
		#include "stm32f3xx_hal.h"       /* Import HAL library */
	#elif defined(STM32F4)
		#include "stm32f4xx_hal.h"       /* Import HAL library */
	#elif defined(STM32F7)
		#include "stm32f7xx_hal.h"       /* Import HAL library */
	#elif defined(STM32G0)
		#include "stm32g0xx_hal.h"       /* Import HAL library */
	#elif defined(STM32G4)
		#include "stm32g4xx_hal.h"       /* Import HAL library */

	/* ------------ Check High Performance series ------------ */

	#elif defined(STM32H7)
		#include "stm32h7xx_hal.h"       /* Import HAL library */

	/* ------------ Check Ultra low power series ------------- */

	#elif defined(STM32L0)
		#include "stm32l0xx_hal.h"       /* Import HAL library */
	#elif defined(STM32L1)
		#include "stm32l1xx_hal.h"       /* Import HAL library */
	#elif defined(STM32L5)
		#include "stm32l5xx_hal.h"       /* Import HAL library */
	#elif defined(STM32L4)
		#include "stm32l4xx_hal.h"       /* Import HAL library */
	#elif defined(STM32H7)
		#include "stm32h7xx_hal.h"       /* Import HAL library */
	#else
	#endif /* STM32F1 */

	/* ------------------------------------------------------- */

	#if defined ( __ICCARM__ ) /* ICCARM Compiler */

	#pragma diag_suppress=Pe177   /* Disable 'unused function' warning */

	#elif defined   (  __GNUC__  ) /* GNU Compiler */

//	#pragma diag_suppress 177     /* Disable 'unused function' warning */

	#endif /* __ICCARM__ */

/* ------------------------------------------------------------------ */

#else                     /* Compiler not found */

#error Chip or Library not supported  /* Send error */

#endif /* __CODEVISIONAVR__ */

/* ------------------------------------------------------------------ */

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Defines ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ------------------------ Library ------------------------ */
#define _PID_LIBRARY_VERSION    1.0.0

/* ------------------------ Public ------------------------- */
#define _PID_8BIT_PWM_MAX_Speed       UINT8_MAX
#define _PID_SAMPLE_TIME_MS_DEF_Speed 100

#ifndef _FALSE

	#define _FALSE 0

#endif

#ifndef _TRUE

	#define _TRUE 1

#endif

/* ---------------------- By compiler ---------------------- */
#ifndef GetTime

	/* ---------------------- By compiler ---------------------- */

	#ifdef __CODEVISIONAVR__  /* Check compiler */

		#define GetTime()   0

	/* ------------------------------------------------------------------ */

	#elif defined(__GNUC__) && !defined(USE_HAL_DRIVER)  /* Check compiler */

		#define GetTime()   0

	/* ------------------------------------------------------------------ */

	#elif defined(USE_HAL_DRIVER)  /* Check driver */

		#define GetTime()   HAL_GetTick()

	/* ------------------------------------------------------------------ */

	#else
	#endif /* __CODEVISIONAVR__ */
	/* ------------------------------------------------------------------ */

#endif

/* --------------------------------------------------------- */

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Types ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* PID Mode */
typedef enum
{
	
	_PID_MODE_MANUAL_SPEED    = 0,
	_PID_MODE_AUTOMATIC_SPEED = 1
	
}PIDMode_TypeDef_Speed;

/* PID P On x */
typedef enum
{
	
	_PID_P_ON_M_SPEED = 0, /* Proportional on Measurement */
	_PID_P_ON_E_SPEED = 1
	
}PIDPON_TypeDef_Speed;

/* PID Control direction */
typedef enum
{
	
	_PID_CD_DIRECT_SPEED  = 0,
	_PID_CD_REVERSE_SPEED = 1
	
}PIDCD_TypeDef_Speed;

/* PID Structure */
typedef struct
{
	
	PIDPON_TypeDef_Speed  POnESpeed;
	PIDMode_TypeDef_Speed InAutoSpeed;

	PIDPON_TypeDef_Speed  POnSpeed;
	PIDCD_TypeDef_Speed   ControllerDirectionSpeed;

	uint32_t        LastTimeSpeed;
	uint32_t        SampleTimeSpeed;

	double          DispKpSpeed;
	double          DispKiSpeed;
	double          DispKdSpeed;

	double          KpSpeed;
	double          KiSpeed;
	double          KdSpeed;

	double          *MyInputSpeed;
	double          *MyOutputSpeed;
	double          *MySetpointSpeed;

	double          OutputSumSpeed;
	double          LastInputSpeed;

	double          OutMinSpeed;
	double          OutMaxSpeed;
	
}PID_TypeDef_Speed;

/* :::::::::::::: Init ::::::::::::: */
void PID_Init_Speed(PID_TypeDef_Speed *uPID);

void PID_Speed(PID_TypeDef_Speed *uPID, double *Input, double *Output, double *Setpoint, double Kp, double Ki, double Kd, PIDPON_TypeDef_Speed POn, PIDCD_TypeDef_Speed ControllerDirection);
void PID2_Speed(PID_TypeDef_Speed *uPID, double *Input, double *Output, double *Setpoint, double Kp, double Ki, double Kd, PIDCD_TypeDef_Speed ControllerDirection);

/* ::::::::::: Computing ::::::::::: */
uint8_t PID_Compute_Speed(PID_TypeDef_Speed *uPID);

/* ::::::::::: PID Mode :::::::::::: */
void            PID_SetMode_Speed(PID_TypeDef_Speed *uPID, PIDMode_TypeDef_Speed Mode);
PIDMode_TypeDef_Speed PID_GetMode_Speed(PID_TypeDef_Speed *uPID);

/* :::::::::: PID Limits ::::::::::: */
void PID_SetOutputLimits_Speed(PID_TypeDef_Speed *uPID, double Min, double Max);

/* :::::::::: PID Tunings :::::::::: */
void PID_SetTunings_Speed(PID_TypeDef_Speed *uPID, double Kp, double Ki, double Kd);
void PID_SetTunings2_Speed(PID_TypeDef_Speed *uPID, double Kp, double Ki, double Kd, PIDPON_TypeDef_Speed POn);

/* ::::::::: PID Direction ::::::::: */
void          PID_SetControllerDirection_Speed(PID_TypeDef_Speed *uPID, PIDCD_TypeDef_Speed Direction);
PIDCD_TypeDef_Speed PID_GetDirection_Speed(PID_TypeDef_Speed *uPID);

/* ::::::::: PID Sampling :::::::::: */
void PID_SetSampleTime_Speed(PID_TypeDef_Speed *uPID, int32_t NewSampleTime);

/* ::::::: Get Tunings Param ::::::: */
double PID_GetKp_Speed(PID_TypeDef_Speed *uPID);
double PID_GetKi_Speed(PID_TypeDef_Speed *uPID);
double PID_GetKd_Speed(PID_TypeDef_Speed *uPID);

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End of the program ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#endif /* __PIDSPEED_H_ */
