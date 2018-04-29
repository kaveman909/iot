/*
 * imu.h
 *
 *  Created on: Apr 22, 2018
 *      Author: David
 */

#ifndef IMU_H_
#define IMU_H_

// Configuration parameters
#define IMU_MOTION_THRESHOLD_MG    40
/*
Lposc_clksel Output Frequency (Hz)
0 0.24
1 0.49
2 0.98
3 1.95
4 3.91
5 7.81
6 15.63
7 31.25
8 62.50
9 125
10 250
11 500
12-15 RESERVED
*/
#define IMU_MOTION_UPDATE_RATE_HZ 5

// I2C Address
#define IMU_I2C_ADDRESS 0x68 // default

#define NUM_OF_CONFIG_CMDS    8
#define NUM_OF_GYRO_REGISTERS 6

// Registers
#define IMU_PWR_MGMT_1      0x6b
#define IMU_PWR_MGMT_2      0x6c
#define IMU_ACCEL_CONFIG_2  0x1d
#define IMU_INT_ENABLE      0x38
#define IMU_MOT_DETECT_CTRL 0x69
#define IMU_WOM_THR         0x1f
#define IMU_LP_ACCEL_ODR    0x1e

enum gyro_registers_e {
	GYRO_XOUT_H = 0x43,
	GYRO_XOUT_L,
	GYRO_YOUT_H,
	GYRO_YOUT_L,
	GYRO_ZOUT_H,
	GYRO_ZOUT_L
};

enum gyro_en_success_e {
	GYRO_EN_FAILURE = -1,
	GYRO_EN_SUCCESS
};

enum gyro_read_success_e {
	GYRO_READ_FAILURE = -1,
	GYRO_READ_SUCCESS
};

/* Wake-on-motion configuration using low-power accel mode.
 * reference MPU9250 datasheet, page 31.
 * https://cdn.sparkfun.com/assets/learn_tutorials/5/5/0/MPU9250REV1.0.pdf
 */

// make sure accel. is running; turn off gyro to save power
#define IMU_PWR_MGMT_1_CONFIG      0x01 // default
#define IMU_PWR_MGMT_2_CONFIG      0x07 // DIS_XG / YG / ZG
// set accel lpf to 184 hz bw
#define IMU_ACCEL_CONFIG_2_CONFIG  0x09 // ACCEL_FCHOICE_B = 1, A_DLPF_CFG = 1 (note: typo in register map)
// enable motion interrupt
#define IMU_INT_ENABLE_CONFIG      0x40 // only motion interrupt is enabled
// enable accel hardware intelligence
#define IMU_MOT_DETECT_CTRL_CONFIG 0xC0 // ACCEL_INTEL_EN / ACCEL_INTEL_MODE
// set motion threshold
#define IMU_WOM_THR_CONFIG         (IMU_MOTION_THRESHOLD_MG / 4)
// set freq of wakeup
#define IMU_LP_ACCEL_ODR_CONFIG    IMU_MOTION_UPDATE_RATE_HZ
// enable cycle mode (accel low power mode)
#define IMU_PWR_MGMT_1_CONFIG2     0x21 // CYCLE + default

/* Once motion is detected, gyroscope must be enabled */
#define IMU_PWR_MGMT_2_EN_GYRO     0x00 // Enable all axes of accelerometer / gyroscope

#endif /* IMU_H_ */

// public functions
void imu_init(void);
void imu_init_sched(void);
void imu_start_write(void);
void imu_config_addr(void);
void imu_config_data(void);
void imu_config_next(void);
void imu_interrupt_handler(void);

// enable gyro functions
int8_t imu_enable_gyro(bool enable);
void imu_en_gyro_addr(void);
void imu_en_gyro_data(void);
void imu_en_gyro_done(void);

// read gyro functions
int8_t imu_read_gyro_start(void);
void imu_read_gyro_addr(void);
void imu_read_gyro_data(void);
void imu_read_gyro_cont(void);
void imu_read_gyro_drdy(void);
