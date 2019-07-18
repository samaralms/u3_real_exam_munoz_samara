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


#define TIME_STEP 64
#define PI 3.14159

#define OBSTACLE_DISTANCE 40.0

enum {
  GO,
  TURN,
  FREEWAY,
  OBSTACLE
};

int counter=0;

double initial_angle_wheel1;

//int dismob_value;
//int disgun_value;

float ReadMobileSensor=0;
float ReadGunSensor=0;

double down_distance=0;
double mobile_distance=0;

double  MobileMotor=0;
double GunMotor=0;



int checkForObstacles(WbDeviceTag ds_down) {
  double distance = wb_distance_sensor_get_value(ds_down);

  if (distance > OBSTACLE_DISTANCE)
    return FREEWAY;
  else
    return OBSTACLE;
}

void forwardRobot(WbDeviceTag *wheels) {
  wb_motor_set_position(wheels[0], INFINITY); 
  wb_motor_set_velocity(wheels[0], 0);
  wb_motor_set_position(wheels[1], INFINITY); 
  wb_motor_set_velocity(wheels[1], -3);
  wb_motor_set_position(wheels[2], INFINITY); 
  wb_motor_set_velocity(wheels[2], 3);
}
void stopRobot(WbDeviceTag *wheels) { 
  wb_motor_set_velocity(wheels[0], 0); 
  wb_motor_set_velocity(wheels[1], 0); 
  wb_motor_set_velocity(wheels[2], 0);
  printf("Detecto Objeto\n");
}
void turnRobot(WbDeviceTag *wheels) { 
  wb_motor_set_velocity(wheels[0], 5); 
  wb_motor_set_velocity(wheels[1], 5); 
  wb_motor_set_velocity(wheels[2], 3);
}

void onRadar(WbDeviceTag *mobmotor) { 
  wb_motor_set_position(mobmotor[0], INFINITY);
  wb_motor_set_velocity(mobmotor[0], 1); 
}
void offRadar(WbDeviceTag *mobmotor) { 
  wb_motor_set_position(mobmotor[0], INFINITY);
  wb_motor_set_velocity(mobmotor[0], 0); 
}
void onGun(WbDeviceTag *gunmotor) { 
  wb_motor_set_position(gunmotor[0], INFINITY);
  wb_motor_set_velocity(gunmotor[0], 1); 
}
void offGun(WbDeviceTag *gunmotor) {
  wb_motor_set_position(gunmotor[0], INFINITY); 
  wb_motor_set_velocity(gunmotor[0], 0); 
}

double getAngleRobot(WbDeviceTag pos_sensor) {
  printf("Calculando angulo\n");
  double angle, angle_wheel1, angle_wheel2;

  angle_wheel1 = wb_position_sensor_get_value(pos_sensor);
  //printf("Angle Wheel1: %lf\n", angle_wheel1);
  angle = fabs(angle_wheel1 - initial_angle_wheel1);
  //printf("Angle: %lf\n", angle);

  return angle;
}

float clearAngleRobot() {
  printf("Clearing angle\n");
}


int main(int argc, char **argv)
{
  /* necessary to initialize webots stuff */
  wb_robot_init();

  WbDeviceTag wheel_1 = wb_robot_get_device("wheel1");
  WbDeviceTag wheel_2 = wb_robot_get_device("wheel2");
  WbDeviceTag wheel_3 = wb_robot_get_device("wheel3");
  
  WbDeviceTag wheels[3];
  wheels[0] = wheel_1;
  wheels[1] = wheel_2;
  wheels[2] = wheel_3;
  
  WbDeviceTag mobMotor = wb_robot_get_device("mobile_motor");
  WbDeviceTag mobmotor[2];
  mobmotor[0] = mobMotor; 
  
  
  WbDeviceTag gunMotor = wb_robot_get_device("gun_motor");
  WbDeviceTag gunmotor[2];
  gunmotor[0] = gunMotor; 
  
 // Encoder devices
  WbDeviceTag encoder = wb_robot_get_device("ps_1");
  wb_position_sensor_enable(encoder, TIME_STEP);

  // Distance sensor devices
  WbDeviceTag dis_down = wb_robot_get_device("ds_down");
  wb_distance_sensor_enable(dis_down, TIME_STEP);
  
  WbDeviceTag dis_mobile = wb_robot_get_device("ds_mobile");
  wb_distance_sensor_enable(dis_mobile, TIME_STEP);
  
  
  down_distance = wb_distance_sensor_get_value(dis_down);
  mobile_distance =  wb_distance_sensor_get_value(dis_mobile);
  
 
  MobileMotor = wb_position_sensor_get_value(mobMotor);
  GunMotor = wb_position_sensor_get_value(gunMotor);
  
  
  
  //double ps_value;
  short int ds_state, robot_state = GO;
  //float velocity = 0.5;
  float angle;
  float counter=0;
 
 
  while (wb_robot_step(TIME_STEP) != -1) {

  
  
  if (robot_state == GO) {
      ds_state = checkForObstacles(dis_down);

      if (ds_state == FREEWAY) {
        forwardRobot(wheels);
        onRadar(mobmotor);
        onGun(gunmotor);
        angle = wb_position_sensor_get_value(encoder);
        printf("Angle: %lf\n", angle);
      } 
      else if (ds_state == OBSTACLE) {
        robot_state = TURN;
        stopRobot(wheels);
        printf("Obstacle Detected\n");
        initial_angle_wheel1 = wb_position_sensor_get_value(encoder);
      }
    } else if (robot_state == TURN) {
        turnRobot(wheels);
        angle = getAngleRobot(encoder);

      if (angle >= 0.4*PI) {
        robot_state = GO;
        stopRobot(wheels);
        clearAngleRobot();
      }
    }
    
   
    
    
            
  printf("La distancia es = %f\n", dis_down);
  fflush(stdout); 
   
     //counter ++;
    
    

  


  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
