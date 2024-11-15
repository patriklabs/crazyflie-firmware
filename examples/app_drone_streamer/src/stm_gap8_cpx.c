/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2023 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * App layer application that communicates with the GAP8 on an AI deck.
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "cpx.h"
#include "cpx_internal_router.h"

#include "FreeRTOS.h"
#include "task.h"
#include "commander.h"
#define DEBUG_MODULE "APP"
#include "debug.h"
#include "log.h"
#include "param.h"
#define CPX_F_MYAPP 6
#define CPX_F_CONTROL 7

// Callback that is called when a CPX packet arrives
static void cpxPacketCallback(const CPXPacket_t *cpxRx);

static CPXPacket_t txPacket;

typedef struct
{
  uint8_t magic;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
  int16_t acc_x;
  int16_t acc_y;
  int16_t acc_z;
  uint64_t timestamp;
} __attribute__((packed)) imu_data_t;

typedef struct
{
  uint8_t magic;
  int16_t vel_x;
  int16_t vel_y;
  int16_t vel_z;
  int16_t yaw_rate;
} __attribute__((packed)) velocity_setpoint_t;

void getIMUData(imu_data_t *data, logVarId_t idGyroX, logVarId_t idGyroY, logVarId_t idGyroZ, logVarId_t idAccX, logVarId_t idAccY, logVarId_t idAccZ)
{
  data->magic = 0xBE;
  // Get the logging data
  data->gyro_x = logGetFloat(idGyroX) * 500;
  // DEBUG_PRINT("gyro_x is now: %f raw\n", (double)data->gyro_x);

  data->gyro_y = logGetFloat(idGyroY) * 500;
  // DEBUG_PRINT("gyro_y is now: %f raw\n", (double)data->gyro_y);

  data->gyro_z = logGetFloat(idGyroZ) * 500;
  // DEBUG_PRINT("gyro_z is now: %f raw\n", (double)data->gyro_z);

  data->acc_x = logGetFloat(idAccX) * 1000;
  // DEBUG_PRINT("acc_x is now: %f raw\n", (double)data->acc_x);

  data->acc_y = logGetFloat(idAccY) * 1000;
  // DEBUG_PRINT("acc_y is now: %f raw\n", (double)data->acc_y);

  data->acc_z = logGetFloat(idAccZ) * 1000;
  // DEBUG_PRINT("acc_z is now: %f raw\n", (double)data->acc_z);

  data->timestamp = xTaskGetTickCount();
  // DEBUG_PRINT("timestamp is now: %lld\n", data->timestamp);
}

void createIMUPacket(CPXPacket_t *packet, imu_data_t *data)
{
  imu_data_t *imu_pack = (imu_data_t *)packet->data;
  imu_pack->magic = data->magic;
  imu_pack->gyro_x = data->gyro_x;
  imu_pack->gyro_y = data->gyro_y;
  imu_pack->gyro_z = data->gyro_z;
  imu_pack->acc_x = data->acc_x;
  imu_pack->acc_y = data->acc_y;
  imu_pack->acc_z = data->acc_z;
  imu_pack->timestamp = data->timestamp;
  packet->dataLength = sizeof(imu_data_t);
}

static float vel_x = 0;
static float vel_y = 0;
static float vel_z = 0;
static float yaw_rate = 0;
static setpoint_t setpoint;
static uint8_t positioningInit = 0;

static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate)
{
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;

  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = yawrate;

  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = vx;
  setpoint->velocity.y = vy;

  setpoint->velocity_body = true;
}

void appMain()
{
  DEBUG_PRINT("Hello! I am the stm_gap8_cpx app\n");
  logVarId_t idGyroX = logGetVarId("gyro", "x");
  logVarId_t idGyroY = logGetVarId("gyro", "y");
  logVarId_t idGyroZ = logGetVarId("gyro", "z");

  logVarId_t idAccX = logGetVarId("acc", "x");
  logVarId_t idAccY = logGetVarId("acc", "y");
  logVarId_t idAccZ = logGetVarId("acc", "z");
  paramVarId_t idPositioningDeck = paramGetVarId("deck", "bcFlow2");

  imu_data_t data;

  // Register a callback for CPX packets.
  // Packets sent to destination=CPX_T_STM32 and function=CPX_F_APP will arrive here
  cpxRegisterAppMessageHandler(cpxPacketCallback);

  TickType_t xLastWakeTime;
  const TickType_t xFrequency = M2T(10);

  xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
    // vTaskDelay(M2T(10));

    positioningInit = paramGetUint(idPositioningDeck);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    getIMUData(&data, idGyroX, idGyroY, idGyroZ, idAccX, idAccY, idAccZ);

    cpxInitRoute(CPX_T_STM32, CPX_T_GAP8, CPX_F_MYAPP, &txPacket.route);

    createIMUPacket(&txPacket, &data);

    cpxSendPacketBlocking(&txPacket);
  }
}

static void cpxPacketCallback(const CPXPacket_t *cpxRx)
{
  // DEBUG_PRINT("Got packet from GAP8 (%u)\n", cpxRx->data[0]);

  velocity_setpoint_t *velocity_data = (velocity_setpoint_t *)cpxRx->data;

  vel_x = velocity_data->vel_x / 1000.0;
  vel_y = velocity_data->vel_y / 1000.0;
  vel_z = velocity_data->vel_z / 1000.0;
  yaw_rate = velocity_data->yaw_rate / 500.0;

  DEBUG_PRINT("vel_x: %f \n", (double)vel_x);
  DEBUG_PRINT("vel_y: %f \n", (double)vel_y);
  DEBUG_PRINT("z: %f \n", (double)vel_z);
  DEBUG_PRINT("yaw_rate: %f \n", (double)yaw_rate);
  DEBUG_PRINT("positioningInit: %f \n", (double)positioningInit);

  setHoverSetpoint(&setpoint, vel_x, vel_y, vel_z, yaw_rate);

  if (positioningInit)
  {
    commanderSetSetpoint(&setpoint, 3);
  }
  /*
  imu_data_t *imu_pack = (imu_data_t *)cpxRx->data;

  DEBUG_PRINT("Got packet gyro_x is now: %f raw\n", (double)imu_pack->gyro_x);

  DEBUG_PRINT("Got packet gyro_y is now: %f raw\n", (double)imu_pack->gyro_y);

  DEBUG_PRINT("Got packet gyro_z is now: %f raw\n", (double)imu_pack->gyro_z);

  DEBUG_PRINT("Got packet acc_x is now: %f raw\n", (double)imu_pack->acc_x);

  DEBUG_PRINT("Got packet acc_y is now: %f raw\n", (double)imu_pack->acc_y);

  DEBUG_PRINT("Got packet acc_z is now: %f raw\n", (double)imu_pack->acc_z);

  DEBUG_PRINT("Got packet timestamp is now: %lld\n", imu_pack->timestamp);*/
}
