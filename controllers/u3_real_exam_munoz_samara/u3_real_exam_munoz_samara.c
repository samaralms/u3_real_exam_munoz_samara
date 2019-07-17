/*
 * File:          u3_real_exam_munoz_samara.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/differential_wheels.h>, etc.
 */
#include <webots/robot.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/distance_sensor.h>
#include <webots/keyboard.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv)
{
  /* necessary to initialize webots stuff */
  wb_robot_init();

  WbDeviceTag wheel_1 = wb_robot_get_device("wheel1");
  WbDeviceTag wheel_2 = wb_robot_get_device("wheel2");
  WbDeviceTag wheel_3 = wb_robot_get_device("wheel3");
  WbDeviceTag distanceMotor = wb_robot_get_device("ds_motor");
  WbDeviceTag gunMotor = wb_robot_get_device("gun_motor");
   
  WbDeviceTag pos1 = wb_robot_get_device("ps_1");
  WbDeviceTag pos2 = wb_robot_get_device("ps_2");
  WbDeviceTag pos3 = wb_robot_get_device("ps_3");
  WbDeviceTag distancePos = wb_robot_get_device("ds_position");
  WbDeviceTag gunPos = wb_robot_get_device("gun_position");

   
   
   wb_position_sensor_enable(pos1,TIME_STEP);
   wb_position_sensor_enable(pos2,TIME_STEP);
   wb_position_sensor_enable(pos3,TIME_STEP);
   wb_position_sensor_enable(distancePos,TIME_STEP);
   wb_position_sensor_enable(gunPos,TIME_STEP);
   
   wb_motor_set_position(wheel_1,INFINITY);
   wb_motor_set_position(wheel_2,INFINITY);
   wb_motor_set_position(wheel_3,INFINITY);
   wb_motor_set_position(distanceMotor,INFINITY);
   wb_motor_set_position(gunMotor,INFINITY);

   wb_motor_set_velocity(wheel_1,-3);
   wb_motor_set_velocity(wheel_2,3);
   wb_motor_set_velocity(wheel_3,0); 
   wb_motor_set_velocity(distanceMotor,3); 
   wb_motor_set_velocity(gunMotor,-3);


  while (wb_robot_step(TIME_STEP) != -1) {





  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
