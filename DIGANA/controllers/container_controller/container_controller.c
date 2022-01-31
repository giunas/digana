/*
 * File:          container_controller.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */
#define TIME_STEP 32

#include <webots/robot.h>
#include <webots/gps.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main() {
  wb_robot_init();
  while (wb_robot_step(TIME_STEP) != -1) {
    WbDeviceTag gps = wb_robot_get_device("gps");
  }
  wb_robot_cleanup();
  return 0;
} 
    

