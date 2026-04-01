 /*
------------------------------------------------------------------------------
~ File   : pid.c
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

#include "pidspeed.h"

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Functions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~ Initialize ~~~~~~~~~~~~~~~~ */
void PID_Init_Speed(PID_TypeDef_Speed *uPID)
{
	/* ~~~~~~~~~~ Set parameter ~~~~~~~~~~ */
	uPID->OutputSumSpeed = *uPID->MyOutputSpeed;
	uPID->LastInputSpeed = *uPID->MyInputSpeed;
	
	if (uPID->OutputSumSpeed > uPID->OutMaxSpeed)
	{
		uPID->OutputSumSpeed = uPID->OutMaxSpeed;
	}
	else if (uPID->OutputSumSpeed< uPID->OutMinSpeed)
	{
		uPID->OutputSumSpeed = uPID->OutMinSpeed;
	}
	else { }
	
}

void PID_Speed(PID_TypeDef_Speed *uPID, double *InputSpeed, double *OutputSpeed, double *SetpointSpeed, double KpSpeed, double KiSpeed, double KdSpeed, PIDPON_TypeDef_Speed POnSpeed, PIDCD_TypeDef_Speed ControllerDirectionSpeed)
{
	/* ~~~~~~~~~~ Set parameter ~~~~~~~~~~ */
	uPID->MyOutputSpeed   = OutputSpeed;
	uPID->MyInputSpeed    = InputSpeed;
	uPID->MySetpointSpeed = SetpointSpeed;
	uPID->InAutoSpeed     = (PIDMode_TypeDef_Speed)_FALSE;
	
	PID_SetOutputLimits_Speed(uPID, 0, _PID_8BIT_PWM_MAX_Speed);
	
	uPID->SampleTimeSpeed = _PID_SAMPLE_TIME_MS_DEF_Speed; /* default Controller Sample Time is 0.1 seconds */
	
	PID_SetControllerDirection_Speed(uPID, ControllerDirectionSpeed);
	PID_SetTunings2_Speed(uPID, KpSpeed, KiSpeed, KdSpeed, POnSpeed);
	
	uPID->LastTimeSpeed = GetTime() - uPID->SampleTimeSpeed;
	
}

void PID2_Speed(PID_TypeDef_Speed *uPID, double *InputSpeed, double *OutputSpeed, double *SetpointSpeed, double KpSpeed, double KiSpeed, double KdSpeed, PIDCD_TypeDef_Speed ControllerDirectionSpeed)
{
	PID_Speed(uPID, InputSpeed, OutputSpeed, SetpointSpeed, KpSpeed, KiSpeed, KdSpeed, _PID_P_ON_E_SPEED, ControllerDirectionSpeed);
}

/* ~~~~~~~~~~~~~~~~~ Computing ~~~~~~~~~~~~~~~~~ */
uint8_t PID_Compute_Speed(PID_TypeDef_Speed *uPID)
{
	
	uint32_t nowSpeed;
	uint32_t timeChangeSpeed;
	
	double inputSpeed;
	double errorSpeed;
	double dInputSpeed;
	double outputSpeed;
	
	/* ~~~~~~~~~~ Check PID mode ~~~~~~~~~~ */
	if (!uPID->InAutoSpeed)
	{
		return _FALSE;
	}
	
	/* ~~~~~~~~~~ Calculate time ~~~~~~~~~~ */
	nowSpeed        = GetTime();
	timeChangeSpeed = (nowSpeed - uPID->LastTimeSpeed);
	
	if (timeChangeSpeed >= uPID->SampleTimeSpeed)
	{
		/* ..... Compute all the working error variables ..... */
		inputSpeed   = *uPID->MyInputSpeed;
		errorSpeed   = *uPID->MySetpointSpeed - inputSpeed;
		dInputSpeed  = (inputSpeed - uPID->LastInputSpeed);
		
		uPID->OutputSumSpeed     += (uPID->KiSpeed * errorSpeed);
		
		/* ..... Add Proportional on Measurement, if P_ON_M is specified ..... */
		if (!uPID->POnESpeed)
		{
			uPID->OutputSumSpeed -= uPID->KpSpeed * dInputSpeed;
		}
		
		if (uPID->OutputSumSpeed > uPID->OutMaxSpeed)
		{
			uPID->OutputSumSpeed = uPID->OutMaxSpeed;
		}
		else if (uPID->OutputSumSpeed < uPID->OutMinSpeed)
		{
			uPID->OutputSumSpeed = uPID->OutMinSpeed;
		}
		else { }
		
		/* ..... Add Proportional on Error, if P_ON_E is specified ..... */
		if (uPID->POnESpeed)
		{
			outputSpeed = uPID->KpSpeed * errorSpeed;
		}
		else
		{
			outputSpeed = 0;
		}
		
		/* ..... Compute Rest of PID Output ..... */
		outputSpeed += uPID->OutputSumSpeed - uPID->KdSpeed * dInputSpeed;
		
		if (outputSpeed > uPID->OutMaxSpeed)
		{
			outputSpeed = uPID->OutMaxSpeed;
		}
		else if (outputSpeed < uPID->OutMinSpeed)
		{
			outputSpeed = uPID->OutMinSpeed;
		}
		else { }
		
		*uPID->MyOutputSpeed = outputSpeed;
		
		/* ..... Remember some variables for next time ..... */
		uPID->LastInputSpeed = inputSpeed;
		uPID->LastTimeSpeed = nowSpeed;
		
		return _TRUE;
		
	}
	else
	{
		return _FALSE;
	}
	
}

/* ~~~~~~~~~~~~~~~~~ PID Mode ~~~~~~~~~~~~~~~~~~ */
void PID_SetMode_Speed(PID_TypeDef_Speed *uPID, PIDMode_TypeDef_Speed Mode)
{
	
	uint8_t newAutoSpeed = (Mode == _PID_MODE_AUTOMATIC_SPEED);
	
	/* ~~~~~~~~~~ Initialize the PID ~~~~~~~~~~ */
	if (newAutoSpeed && !uPID->InAutoSpeed)
	{
		PID_Init_Speed(uPID);
	}
	
	uPID->InAutoSpeed = (PIDMode_TypeDef_Speed)newAutoSpeed;
	
}
PIDMode_TypeDef_Speed PID_GetMode_Speed(PID_TypeDef_Speed *uPID)
{
	return uPID->InAutoSpeed ? _PID_MODE_AUTOMATIC_SPEED : _PID_MODE_MANUAL_SPEED;
}

/* ~~~~~~~~~~~~~~~~ PID Limits ~~~~~~~~~~~~~~~~~ */
void PID_SetOutputLimits_Speed(PID_TypeDef_Speed *uPID, double Min, double Max)
{
	/* ~~~~~~~~~~ Check value ~~~~~~~~~~ */
	if (Min >= Max)
	{
		return;
	}
	
	uPID->OutMinSpeed = Min;
	uPID->OutMaxSpeed = Max;
	
	/* ~~~~~~~~~~ Check PID Mode ~~~~~~~~~~ */
	if (uPID->InAutoSpeed)
	{
		
		/* ..... Check out value ..... */
		if (*uPID->MyOutputSpeed > uPID->OutMaxSpeed)
		{
			*uPID->MyOutputSpeed = uPID->OutMaxSpeed;
		}
		else if (*uPID->MyOutputSpeed < uPID->OutMinSpeed)
		{
			*uPID->MyOutputSpeed = uPID->OutMinSpeed;
		}
		else { }
		
		/* ..... Check out value ..... */
		if (uPID->OutputSumSpeed > uPID->OutMaxSpeed)
		{
			uPID->OutputSumSpeed = uPID->OutMaxSpeed;
		}
		else if (uPID->OutputSumSpeed < uPID->OutMinSpeed)
		{
			uPID->OutputSumSpeed = uPID->OutMinSpeed;
		}
		else { }
		
	}
	
}

/* ~~~~~~~~~~~~~~~~ PID Tunings ~~~~~~~~~~~~~~~~ */
void PID_SetTunings_Speed(PID_TypeDef_Speed *uPID, double KpSpeed, double KiSpeed, double KdSpeed)
{
	PID_SetTunings2_Speed(uPID, KpSpeed, KiSpeed, KdSpeed, uPID->POnSpeed);
}
void PID_SetTunings2_Speed(PID_TypeDef_Speed *uPID, double KpSpeed, double KiSpeed, double KdSpeed, PIDPON_TypeDef_Speed POnSpeed)
{
	
	double SampleTimeInSecSpeed;
	
	/* ~~~~~~~~~~ Check value ~~~~~~~~~~ */
	if (KpSpeed < 0 || KiSpeed < 0 || KdSpeed < 0)
	{
		return;
	}
	
	/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
	uPID->POnSpeed    = POnSpeed;
	uPID->POnESpeed   = (PIDPON_TypeDef_Speed)(POnSpeed == _PID_P_ON_E_SPEED);

	uPID->DispKpSpeed = KpSpeed;
	uPID->DispKiSpeed = KiSpeed;
	uPID->DispKdSpeed = KdSpeed;
	
	/* ~~~~~~~~~ Calculate time ~~~~~~~~ */
	SampleTimeInSecSpeed = ((double)uPID->SampleTimeSpeed) / 1000;
	
	uPID->KpSpeed = KpSpeed;
	uPID->KiSpeed = KiSpeed * SampleTimeInSecSpeed;
	uPID->KdSpeed = KdSpeed / SampleTimeInSecSpeed;
	
	/* ~~~~~~~~ Check direction ~~~~~~~~ */
	if (uPID->ControllerDirectionSpeed == _PID_CD_REVERSE_SPEED)
	{
		
		uPID->KpSpeed = (0 - uPID->KpSpeed);
		uPID->KiSpeed = (0 - uPID->KiSpeed);
		uPID->KdSpeed = (0 - uPID->KdSpeed);
		
	}
	
}

/* ~~~~~~~~~~~~~~~ PID Direction ~~~~~~~~~~~~~~~ */
void          PID_SetControllerDirection_Speed(PID_TypeDef_Speed *uPID, PIDCD_TypeDef_Speed DirectionSpeed)
{
	/* ~~~~~~~~~~ Check parameters ~~~~~~~~~~ */
	if ((uPID->InAutoSpeed) && (DirectionSpeed !=uPID->ControllerDirectionSpeed))
	{
		
		uPID->KpSpeed = (0 - uPID->KpSpeed);
		uPID->KiSpeed = (0 - uPID->KiSpeed);
		uPID->KdSpeed = (0 - uPID->KdSpeed);
		
	}
	
	uPID->ControllerDirectionSpeed = DirectionSpeed;
	
}
PIDCD_TypeDef_Speed PID_GetDirection_Speed(PID_TypeDef_Speed *uPID)
{
	return uPID->ControllerDirectionSpeed;
}

/* ~~~~~~~~~~~~~~~ PID Sampling ~~~~~~~~~~~~~~~~ */
void PID_SetSampleTime_Speed(PID_TypeDef_Speed *uPID, int32_t NewSampleTimeSpeed)
{
	
	double ratioSpeed;
	
	/* ~~~~~~~~~~ Check value ~~~~~~~~~~ */
	if (NewSampleTimeSpeed > 0)
	{
		
		ratioSpeed = (double)NewSampleTimeSpeed / (double)uPID->SampleTimeSpeed;
		
		uPID->KiSpeed *= ratioSpeed;
		uPID->KdSpeed /= ratioSpeed;
		uPID->SampleTimeSpeed = (uint32_t)NewSampleTimeSpeed;
		
	}
	
}

/* ~~~~~~~~~~~~~ Get Tunings Param ~~~~~~~~~~~~~ */
double PID_GetKp_Speed(PID_TypeDef_Speed *uPID)
{
	return uPID->DispKpSpeed;
}
double PID_GetKi_Speed(PID_TypeDef_Speed *uPID)
{
	return uPID->DispKiSpeed;
}
double PID_GetKd_Speed(PID_TypeDef_Speed *uPID)
{
	return uPID->DispKdSpeed;
}
