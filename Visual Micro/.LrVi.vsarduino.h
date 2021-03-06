/* 
	Editor: http://www.visualmicro.com
	        visual micro and the arduino ide ignore this code during compilation. this code is automatically maintained by visualmicro, manual changes to this file will be overwritten
	        the contents of the Visual Micro sketch sub folder can be deleted prior to publishing a project
	        all non-arduino files created by visual micro and all visual studio project or solution files can be freely deleted and are not required to compile a sketch (do not delete your own code!).
	        note: debugger breakpoints are stored in '.sln' or '.asln' files, knowledge of last uploaded breakpoints is stored in the upload.vmps.xml file. Both files are required to continue a previous debug session without needing to compile and upload again
	
	Hardware: Arduino Pro or Pro Mini (5V, 16 MHz) w/ ATmega328, Platform=avr, Package=arduino
*/

#ifndef _VSARDUINO_H_
#define _VSARDUINO_H_
#define __AVR_ATmega328p__
#define __AVR_ATmega328P__
#define ARDUINO 106
#define ARDUINO_MAIN
#define __AVR__
#define __avr__
#define F_CPU 16000000L
#define __cplusplus
#define __inline__
#define __asm__(x)
#define __extension__
#define __ATTR_PURE__
#define __ATTR_CONST__
#define __inline__
#define __asm__ 
#define __volatile__

#define __builtin_va_list
#define __builtin_va_start
#define __builtin_va_end
#define __DOXYGEN__
#define __attribute__(x)
#define NOINLINE __attribute__((noinline))
#define prog_void
#define PGM_VOID_P int
            
typedef unsigned char byte;
extern "C" void __cxa_pure_virtual() {;}

//
//
void motor_calc();
void motor_osc();
void bae_write();
void serial_mpu();
void serial_deg();
void serial_deg(int delay);
void calib();
void kalman_init();
void MPU_Getdata();
void MPU_Init();
void parse();
void serial_radio();
void radio_init();
void read_ppm();

#include "C:\Program Files (x86)\Arduino\hardware\arduino\cores\arduino\arduino.h"
#include "C:\Program Files (x86)\Arduino\hardware\arduino\variants\standard\pins_arduino.h" 
#include "C:\Users\Mingyu\Documents\Arduino\LrVi\LrVi.ino"
#include "C:\Users\Mingyu\Documents\Arduino\LrVi\Event.cpp"
#include "C:\Users\Mingyu\Documents\Arduino\LrVi\Event.h"
#include "C:\Users\Mingyu\Documents\Arduino\LrVi\I2Cdev.cpp"
#include "C:\Users\Mingyu\Documents\Arduino\LrVi\I2Cdev.h"
#include "C:\Users\Mingyu\Documents\Arduino\LrVi\Kalman.h"
#include "C:\Users\Mingyu\Documents\Arduino\LrVi\MPU6050.cpp"
#include "C:\Users\Mingyu\Documents\Arduino\LrVi\MPU6050.h"
#include "C:\Users\Mingyu\Documents\Arduino\LrVi\MPU6050_6Axis_MotionApps20.h"
#include "C:\Users\Mingyu\Documents\Arduino\LrVi\MPU6050_9Axis_MotionApps41.h"
#include "C:\Users\Mingyu\Documents\Arduino\LrVi\Servo.cpp"
#include "C:\Users\Mingyu\Documents\Arduino\LrVi\Servo.h"
#include "C:\Users\Mingyu\Documents\Arduino\LrVi\Timer.cpp"
#include "C:\Users\Mingyu\Documents\Arduino\LrVi\Timer.h"
#include "C:\Users\Mingyu\Documents\Arduino\LrVi\helper_3dmath.h"
#endif
