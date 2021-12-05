#ifndef ROBOTMOTIONCONFIG_H
#define ROBOTMOTIONCONFIG_H

#include <Arduino.h>
#include "TimerThree.h"

// Robot
#define ROBOT_L 0.19133   // Distance between the robot's geometric center and the wheels (m)
const float 
        ROBOT_WH_D[]={    // Diameter of the wheels (m)
          0.09785 , 
          0.09667 , 
          0.09697 };
#define VBATT 28.00       // Battery level (V)

// Motors
#define MOT_NGEAR  12            // Gear reduction ratio (n:1)
#define MOT_ENCRES 1024          // Resolution of the encoder (ppr)
#define MOT_KP  2.737441777222   // Gain (rad.s⁻¹/V)
#define MOT_TAU 0.208736119092   // Time constant (s)
#define MOT_LAG 0.00             // Lag (s)
#define CTRL_TIME  10000         // Control period (us)
#define MOT_VMAX   24.00         // Maximum voltage applied to the motor
#define MOT_PWMMAX 1023          // Maximum value for the PWM
const int MOT_DIR_PIN[]   = { 8 , 6 , 4 };
const int MOT_TIMER_PIN[] = { TIMER3_C_PIN , TIMER3_A_PIN , TIMER3_B_PIN };
const int MOT_CURRA_PIN[] = { A8 , A10 , A12 };
const int MOT_CURRB_PIN[] = { A9 , A11 , A13 };
#define MOT_HAMM_V0 1.5
#define MOT_HAMM_VD 1.0

// PI controllers (IMC method)
#define PI_TAUCL (MOT_TAU / 1.5)                    // Closed-loop time constant (s)
#define PI_KCKP  (MOT_TAU / (PI_TAUCL + MOT_LAG))   // Division ratio of the closed-loop time constant
#define PI_KC    (PI_KCKP / MOT_KP)                 // Proportional gain (V/rad.s⁻¹)
#define PI_TI    (MOT_TAU)                          // Integrative time (s)

// Conversion constants
#define IMP2W (2 * PI * 1000000 / (1.0 * CTRL_TIME * MOT_NGEAR * MOT_ENCRES)) // Impulses per cycle (imp/cyc) >>> Angular velocity of the wheel (rad/s)
#define V2PWM ((MOT_PWMMAX*1.0) / VBATT)                                      // Input voltage (V) >>> PWM value (-1023..1023)

#endif