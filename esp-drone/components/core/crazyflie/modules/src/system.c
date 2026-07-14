/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * system.c - Top level module implementation
 */

#include <stdbool.h>
#include <inttypes.h>
#include <string.h>
/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "version.h"
#include "config.h"
#include "param.h"
#include "log.h"
#include "ledseq.h"
#include "adc_esp32.h"
#include "pm_esplane.h"
#include "config.h"
#include "system.h"
#include "platform.h"
//#include "storage.h"
#include "configblock.h"
#include "worker.h"
#include "freeRTOSdebug.h"
//#include "uart_syslink.h"
//#include "uart1.h"
//#include "uart2.h"
#include "wifi_esp32.h"
#include "comm.h"
#include "stabilizer.h"
#include "commander.h"
#include "ble_control.h"
#include "espnow_control.h"
#include "console.h"
#include "wifilink.h"
#include "mem.h"
//#include "proximity.h"
//#include "watchdog.h"
#include "queuemonitor.h"
#include "buzzer.h"
#include "sound.h"
#include "sysload.h"
#include "estimator_kalman.h"
//#include "deck.h"
//#include "extrx.h"
#include "app.h"
#include "stm32_legacy.h"
#define DEBUG_MODULE "SYS"
#include "debug_cf.h"
#include "static_mem.h"
//#include "peer_localization.h"
#include "cfassert.h"

#ifndef START_DISARMED
#define ARM_INIT true
#else
#define ARM_INIT false
#endif

/* Private variable */
static bool selftestPassed;
static bool canFly;
static bool armed = ARM_INIT;
static bool forceArm;
static bool isInit;

STATIC_MEM_TASK_ALLOC(systemTask, SYSTEM_TASK_STACKSIZE);

/* System wide synchronisation */
xSemaphoreHandle canStartMutex;
static StaticSemaphore_t canStartMutexBuffer;

/* Private functions */
static void systemTask(void *arg);

/* Public functions */
void systemLaunch(void)
{
  STATIC_MEM_TASK_CREATE(systemTask, systemTask, SYSTEM_TASK_NAME, NULL, SYSTEM_TASK_PRI);
}

// This must be the first module to be initialized!
void systemInit(void)
{
  if(isInit)
    return;

  DEBUG_PRINT_LOCAL("----------------------------\n");
  DEBUG_PRINT_LOCAL("%s is up and running!\n", platformConfigGetDeviceTypeName());

  canStartMutex = xSemaphoreCreateMutexStatic(&canStartMutexBuffer);
  xSemaphoreTake(canStartMutex, portMAX_DELAY);

  wifilinkInit();
  sysLoadInit();

  /* Initialized here so that DEBUG_PRINT (buffered) can be used early */
  debugInit();
  crtpInit();
  consoleInit();

  /* DEBUG_PRINT("----------------------------\n");
  DEBUG_PRINT("%s is up and running!\n", platformConfigGetDeviceTypeName());

  if (V_PRODUCTION_RELEASE) {
    DEBUG_PRINT("Production release %s\n", V_STAG);
  } else {
    DEBUG_PRINT("Build %s:%s (%s) %s\n", V_SLOCAL_REVISION,
                V_SREVISION, V_STAG, (V_MODIFIED)?"MODIFIED":"CLEAN");
  }
  DEBUG_PRINT("I am 0x%08X%08X%08X and I have %dKB of flash!\n",
              *((int*)(MCU_ID_ADDRESS+8)), *((int*)(MCU_ID_ADDRESS+4)),
              *((int*)(MCU_ID_ADDRESS+0)), *((short*)(MCU_FLASH_SIZE_ADDRESS)));*/

  configblockInit();
  //storageInit();
  workerInit();
  adcInit();
  ledseqInit();
  pmInit();
  buzzerInit();
//  peerLocalizationInit();

#ifdef APP_ENABLED
  appInit();
#endif

  isInit = true;
}

bool systemTest()
{
  bool pass=isInit;

  pass &= ledseqTest();
  pass &= pmTest();
  DEBUG_PRINTI("pmTest = %d", pass);
  pass &= workerTest();
  DEBUG_PRINTI("workerTest = %d", pass);
  pass &= buzzerTest();
  return pass;
}

/* Private functions implementation */

/* ===========================================================================
 * RADIO SELECT — pick exactly ONE control radio (running two 2.4GHz radios at
 * once browns out the board). Set ONE of these to 1 (or both 0 = Wi-Fi AP):
 *   both 0                  -> Wi-Fi AP control (drive_blimp.py / web panel). STABLE.
 *   BLE_CONTROL_ENABLED 1   -> BLE only (Wi-Fi AP off).  drive_blimp_ble.py
 *   ESPNOW_CONTROL_ENABLED 1-> ESP-NOW only (Wi-Fi AP off, via C6 bridge).
 *                              drive_blimp_espnow.py
 * Change here and reflash. ESP-NOW mode also forces passthrough mixing so the
 * client sends motor-domain values (matches drive_blimp_espnow.py / the panel).
 * =========================================================================== */
#define BLE_CONTROL_ENABLED 0
#define ESPNOW_CONTROL_ENABLED 1

/* blimp mixer scales (defined in power_distribution_stock.c) — set for passthrough
 * in ESP-NOW mode since there is no live CRTP param link over ESP-NOW. */
extern float blimpFwdScale, blimpVertGain, blimpTurnGain, blimpVertScale;

/* Turns BLE control floats into a setpoint and feeds the commander, mirroring
 * what the Wi-Fi commander delivers so BLIMP_MODE in the stabilizer is unchanged.
 * pitch -> vertical (up/down), yaw -> turn, thrust -> forward. */
#if ESPNOW_CONTROL_ENABLED
void blimpGuidanceClearAuto(void);   /* drop the autonomous latch on real manual input */
#endif
static void bleSetpointHandler(float roll, float pitch, float yaw, float thrust)
{
#if ESPNOW_CONTROL_ENABLED
  /* A non-trivial manual command means the operator wants manual control now;
   * clear the ESP-NOW autonomous latch so manual setpoints aren't ignored by the
   * stabilizer's auto-priority branch after a prior autonomous session. Zero
   * heartbeats (all idle) are left alone so they can't kick out of autonomy. */
  if (thrust != 0.0f || pitch != 0.0f || yaw != 0.0f) {
    blimpGuidanceClearAuto();
  }
#endif
  setpoint_t sp;
  memset(&sp, 0, sizeof(sp));
  sp.mode.x = modeDisable;
  sp.mode.y = modeDisable;
  sp.mode.z = modeDisable;
  sp.mode.roll = modeAbs;
  sp.mode.pitch = modeAbs;
  sp.mode.yaw = modeVelocity;
  sp.attitude.roll = roll;
  sp.attitude.pitch = pitch;
  sp.attitudeRate.yaw = yaw;
  sp.thrust = thrust;
  commanderSetSetpoint(&sp, COMMANDER_PRIORITY_CRTP);
}

#if ESPNOW_CONTROL_ENABLED
#include "blimp_guidance.h"
/* Forward a 32-byte ESP-NOW mocap pose frame to the on-board guidance, which
 * runs the decoupled PID itself. Manual (16-byte) frames are unaffected. */
static void espnowMocapHandler(const float pose8[8])
{
  blimpGuidanceSetMocap(pose8);
}
/* Link-loss: drop the autonomous latch so the on-board guidance can't keep
 * flying after the ESP-NOW stream stops (motors are also zeroed by the failsafe
 * task). Pairs with the bridge no longer sending an idle heartbeat, so a silent
 * Mac now propagates all the way to a hard stop on the drone. */
static void espnowFailsafeHandler(void)
{
  blimpGuidanceClearAuto();
}
/* Live gain-tuning frame (56 bytes) from the panel -> update the controller. */
static void espnowGainsHandler(const float gains[BLIMP_NUM_GAINS])
{
  blimpGuidanceSetGains(gains);
}
#endif

void systemTask(void *arg)
{
  bool pass = true;

  ledInit();
  ledSet(CHG_LED, 1);
#if BLE_CONTROL_ENABLED
  /* BLE mode: no Wi-Fi radio at all. */
#elif ESPNOW_CONTROL_ENABLED
  /* ESP-NOW mode: STA-only base init, NO SoftAP. Running an AP alongside the
   * motors browned the board out (worked "for a split second" then reset); the
   * AP keeps the PA hot. STA-idle draws far less and matches the proven
   * BlimpSwarm blimp. Still supplies the base init + UDP queues boot needs. */
  wifiInitEspnowBase();
#else
  /* Normal Wi-Fi AP (web panel / drive_blimp.py / mocap-over-Wi-Fi). */
  wifiInit();
#endif
  vTaskDelay(M2T(500));

#ifdef DEBUG_QUEUE_MONITOR
  queueMonitorInit();
#endif

#ifdef ENABLE_UART1
  uart1Init(9600);
#endif
#ifdef ENABLE_UART2
  uart2Init(115200);
#endif

  //Init the high-levels modules
  systemInit();
  commInit();
  commanderInit();

#if BLE_CONTROL_ENABLED
  /* Bring up the BLE control link (Wi-Fi AP is off in this mode). Registered
   * AFTER commanderInit so the handler can safely call commanderSetSetpoint(). */
  bleControlSetHandler(bleSetpointHandler);
  bleControlInit();
#elif ESPNOW_CONTROL_ENABLED
  /* ESP-NOW control (Wi-Fi AP off). Force passthrough mixing: the ESP-NOW client
   * sends motor-domain values (no live param link over ESP-NOW). */
  blimpFwdScale = 1.0f; blimpVertGain = 1.0f; blimpTurnGain = 1.0f; blimpVertScale = 2.0f;
  espnowControlSetHandler(bleSetpointHandler);
  /* Autonomous over ESP-NOW: a 32-byte mocap frame -> the on-board guidance.
   * Manual 16-byte frames still go to bleSetpointHandler above, so manual flight
   * is unaffected; autonomy only engages when the mocap script streams poses. */
  espnowControlSetMocapHandler(espnowMocapHandler);
  espnowControlSetGainsHandler(espnowGainsHandler);
  espnowControlSetFailsafeHandler(espnowFailsafeHandler);
  /* Safe to init inline now: wifiInit() above already brought up the radio, so
   * espnowControlInit() only adds esp_now on top (no wifi re-init, no boot hang). */
  espnowControlInit();
#else
  (void)bleSetpointHandler;   /* compiled but unused in Wi-Fi mode */
#endif

  StateEstimatorType estimator = anyEstimator;
  estimatorKalmanTaskInit();
  //deckInit();
  //estimator = deckGetRequiredEstimator();
  stabilizerInit(estimator);
  //if (deckGetRequiredLowInterferenceRadioMode() && platformConfigPhysicalLayoutAntennasAreClose())
  //{
  //  platformSetLowInterferenceRadioMode();
  //}
  soundInit();
  memInit();

#ifdef PROXIMITY_ENABLED
  proximityInit();
#endif

	/* Test each modules */
  pass &= wifiTest();
  DEBUG_PRINTI("wifilinkTest = %d ", pass);
  pass &= systemTest();
  DEBUG_PRINTI("systemTest = %d ", pass);
  pass &= configblockTest();
  DEBUG_PRINTI("configblockTest = %d ", pass);
  //pass &= storageTest();
  pass &= commTest();
  DEBUG_PRINTI("commTest = %d ", pass);
  pass &= commanderTest();
  DEBUG_PRINTI("commanderTest = %d ", pass);
  pass &= stabilizerTest();
  DEBUG_PRINTI("stabilizerTest = %d ", pass);
  pass &= estimatorKalmanTaskTest();
  DEBUG_PRINTI("estimatorKalmanTaskTest = %d ", pass);
  //pass &= deckTest();
  //pass &= soundTest();
  //pass &= memTest();
  DEBUG_PRINTI("memTest = %d ", pass);
  //pass &= watchdogNormalStartTest();
  pass &= cfAssertNormalStartTest();
//  pass &= peerLocalizationTest();

  //Start the firmware
  if(pass)
  {
    selftestPassed = 1;
    systemStart();
    DEBUG_PRINTI("systemStart ! selftestPassed = %d", selftestPassed);
    soundSetEffect(SND_STARTUP);
    ledseqRun(&seq_alive);
    ledseqRun(&seq_testPassed);
  }
  else
  {
    selftestPassed = 0;
    if (systemTest())
    {
      while(1)
      {
        ledseqRun(&seq_testFailed);
        vTaskDelay(M2T(2000));
        // System can be forced to start by setting the param to 1 from the cfclient
        if (selftestPassed)
        {
	        DEBUG_PRINT("Start forced.\n");
          systemStart();
          break;
        }
      }
    }
    else
    {
      ledInit();
      ledSet(SYS_LED, true);
    }
  }
  DEBUG_PRINT("Free heap: %"PRIu32" bytes\n", xPortGetFreeHeapSize());

  workerLoop();

  //Should never reach this point!
  while(1)
    vTaskDelay(portMAX_DELAY);
}


/* Global system variables */
void systemStart()
{
  xSemaphoreGive(canStartMutex);
#ifndef DEBUG_EP2
  //watchdogInit();
#endif
}

void systemWaitStart(void)
{
  //This permits to guarantee that the system task is initialized before other
  //tasks waits for the start event.
  while(!isInit)
    vTaskDelay(2);

  xSemaphoreTake(canStartMutex, portMAX_DELAY);
  xSemaphoreGive(canStartMutex);
}

void systemSetCanFly(bool val)
{
  canFly = val;
}

bool systemCanFly(void)
{
  return canFly;
}

void systemSetArmed(bool val)
{
  armed = val;
}

bool systemIsArmed()
{

  return armed || forceArm;
}
// void vApplicationIdleHook( void )
// {
//   static uint32_t tickOfLatestWatchdogReset = M2T(0);

//   portTickType tickCount = xTaskGetTickCount();

//   if (tickCount - tickOfLatestWatchdogReset > M2T(WATCHDOG_RESET_PERIOD_MS))
//   {
//     tickOfLatestWatchdogReset = tickCount;
//     watchdogReset();
//   }

//   // Enter sleep mode. Does not work when debugging chip with SWD.
//   // Currently saves about 20mA STM32F405 current consumption (~30%).
// #ifndef DEBUG
//   { __asm volatile ("wfi"); }
// #endif
// }

/*System parameters (mostly for test, should be removed from here) */
/*PARAM_GROUP_START(cpu)
PARAM_ADD(PARAM_UINT16 | PARAM_RONLY, flash, MCU_FLASH_SIZE_ADDRESS)
PARAM_ADD(PARAM_UINT32 | PARAM_RONLY, id0, MCU_ID_ADDRESS+0)
PARAM_ADD(PARAM_UINT32 | PARAM_RONLY, id1, MCU_ID_ADDRESS+4)
PARAM_ADD(PARAM_UINT32 | PARAM_RONLY, id2, MCU_ID_ADDRESS+8)
PARAM_GROUP_STOP(cpu)*/

PARAM_GROUP_START(system)
PARAM_ADD(PARAM_INT8 | PARAM_RONLY, selftestPassed, &selftestPassed)
PARAM_ADD(PARAM_INT8, forceArm, &forceArm)
PARAM_GROUP_STOP(sytem)

/* Loggable variables */
LOG_GROUP_START(sys)
LOG_ADD(LOG_INT8, canfly, &canFly)
LOG_ADD(LOG_INT8, armed, &armed)
LOG_GROUP_STOP(sys)
