/*
 * File:         void.c
 * Description:  This is an empty robot controller, the robot does nothing.
 * Author:       www.cyberbotics.com
 * Note:         !!! PLEASE DO NOT MODIFY THIS SOURCE FILE !!!
 *               This is a system file that Webots needs to work correctly.
 */

#include <webots/robot.h>
#include <webots/gps.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TIME_STEP 32


int main() {
  wb_robot_init();
  while (wb_robot_step(TIME_STEP) != -1) {
    WbDeviceTag gps = wb_robot_get_device("gps");
  }
  wb_robot_cleanup();
  return 0;
} 
    
