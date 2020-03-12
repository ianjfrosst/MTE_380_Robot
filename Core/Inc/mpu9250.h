/**
 * @file mpu9250.h
 * @author Ian Frosst
 * @brief MPU-9250 driver
 * @date 2020-01-20
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef _MPU9250_H
#define _MPU9250_H

#include <stdint.h>

typedef struct {
  uint8_t id;  // 9250 or 9255

  float Ax, Ay, Az;  // Units: g (9.81 m/s^2)
  float Gx, Gy, Gz;  // Units: rad/s
  float Mx, My, Mz;  // units: uT

  float A_res, G_res, M_res;
  float Mx_adj, My_adj, Mz_adj;

  int16_t Ax_raw, Ay_raw, Az_raw;
  int16_t Gx_raw, Gy_raw, Gz_raw;
  int16_t Mx_raw, My_raw, Mz_raw;
} MPU9250_t;

void MPU9250_Init(MPU9250_t* mpu);

void MPU9250_ReadAccel(MPU9250_t* mpu);
void MPU9250_ReadGyro(MPU9250_t* mpu);
void MPU9250_ReadMag(MPU9250_t* mpu);

int16_t MPU9250_ReadTemp();

#endif /* _MPU9250_H */
