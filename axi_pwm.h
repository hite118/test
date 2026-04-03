/* ---------------------------------------------------------------
 * Copyright(C) 2023, BLUTEK Co., LTD. All Right Reserved.
 * ---------------------------------------------------------------
 * Author     : ChongHan Park (E-mail: chpark@blutek.co.kr)
 * Filename   : axi_pwm.h
 * Created on : 2025. 07. 18.
 * Description: AXI PWM device driver & example
 * ---------------------------------------------------------------
 */

#ifndef __AXI_PWM_H__
#define __AXI_PWM_H__

#include <stdio.h>
#include <stdint.h>

#ifndef GET_BIT
#define GET_BIT(REG, BIT)	((REG) & (BIT))
#endif
#ifndef SET_BIT
#define SET_BIT(REG, BIT)	((REG) |= (BIT))
#endif
#ifndef CLR_BIT
#define CLR_BIT(REG, BIT)	((REG) &= ~(BIT))
#endif
#ifndef GET_REG
#define GET_REG(REG)		((REG))
#endif
#ifndef SET_REG
#define SET_REG(REG, VAL)	((REG) = (VAL))
#endif
#ifndef CLR_REG
#define CLR_REG(REG)		((REG) = (0x0))
#endif

#ifndef __IO
#define __IO volatile
#endif

#pragma pack(1)
typedef struct {
	__IO uint32_t CHIPID;
	__IO uint32_t CPOL;  /*0: Normal(Polarity High, Default), 1: Polarity High*/
	__IO uint32_t MODE;  /*0: Fast PWM(Default), 1: Center aligned PWM */
	__IO uint32_t EN; /* 0: Fast PWM(Default), 1: Center aligned PWM */
	__IO uint32_t PERIOD; /* 0: Disable, 1: Eanble */
	__IO uint32_t DUTY;
} PWM_DEVICE;
#pragma pack()

/*****************************************************************************/
/**
*
* PWM initialization function
* @param  PWM device structure address
*
******************************************************************************/
static inline int pwm_init(PWM_DEVICE *pwm)
{
	pwm->CPOL	= 0x0U;
	pwm->MODE	= 0x0U;
	pwm->EN 	= 0x0U;
	pwm->PERIOD = 0x0U;
	pwm->DUTY 	= 0x0U;
	return 0;
}

/*****************************************************************************/
/**
*
* PWM initialization function
* @param  PWM device structure address
* @param  Whether or not it is carried out
*
******************************************************************************/
static inline int pwm_ctrl(PWM_DEVICE *pwm, u32 value)
{
	pwm->EN = value;
	return 0;
}

/*****************************************************************************/
/**
*
* Set pwm
* @param PWM address
* @param pwm mode
* @param Start bit 1 or 0
* @param Whether or not it is carried out
* @param Frequency
* *Update(2024-08-26) 신뢰성검사 : division by zero의 발생을 방지하기 위해, 제수로 상수 0 또는 검증되지 않은 객체를 사용하면 안됨
******************************************************************************/
static inline int pwm_setup(PWM_DEVICE *pwm, u32 mode, u32 cpol, u32 en, u32 freq)
{
    float period_ns = 0.0;
    int ret = 0;  // 성공을 나타내는 값

	pwm->CPOL 	= cpol;
	pwm->MODE 	= mode;

	pwm_ctrl(pwm,en); //pwm->EN		= 	en;

	if (freq != 0)
	{
		period_ns 	= (1.0 / (float)freq) * 1000000000.0;
		pwm->PERIOD = (u32)(period_ns * 0.05);
	}
    else
    {
		period_ns = 0.0f; // 기본값 설정
		ret = -1; // 오류 발생 시 반환할 값 설정
    }
	return ret;
}

/*****************************************************************************/
/**
*
* setting pwm duty
* @param PWM address
* @param  Duty setting value
*
******************************************************************************/
static inline int pwm_duty(PWM_DEVICE *pwm, float duty)
{
	float duty_cycle = 0.0;
	duty_cycle = (float)duty * 0.01;
	pwm->DUTY = (u32)(pwm->PERIOD * duty_cycle);
	return 0;
}

/*    PWM Settings Description */
/*
PWM_DEVICE *PWM = 0x80000000;

pwm_init(PWM);      // 0hz,  pwm->EN = 0x0U, pwm->CPOL = 0x0U, pwm->MODE = 0x0U;
pwm_setup(PWM, 1, 1, 1, 1000); //____HHHH____ : mode -> enable,cpol ->1 ,Contor PWM, 1KHz,

pwm_duty(PWM, 0.0F);


pwm_duty(PWM, 50.0F); // Duty cycle 50%
pwm_ctrl(PWM, 0x1); <= enable
pwm_ctrl(PWM, 0x0); <= disable

*/

/*
 pwm configuration
	pwm_init (PWM);    // input clock 100MHz
	pwm_setup(PWM0, 100000U, 0); // ch0 period 100KHz, Fast PWM
	pwm_setup(PWM0, 1, 100000U, 1); // ch1 period 100KHz, Phase Correct PWM
	pwm_duty (PWM0, 0, 0.0F);       // ch0 duty cycle 0%
	pwm_ctrl (PWM0, 0x3);           // enable all channel
*/



#endif /* __AXI_PWM_H__ */
