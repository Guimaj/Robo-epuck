#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/accelerometer.h>
#include <webots/motor.h>
#include <webots/supervisor.h>
#include <webots/supervisor.h>
#include <stdio.h>
#include <stdlib.h>
#include <webots/led.h>
#include <unistd.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28
#define LEDS 10


typedef struct ObstacleDetector{
  bool right;
  bool left;
} ObstacleDetector;


static int get_time_step() {
    static int time_step = -1;
    if (time_step == -1)
        time_step = (int)wb_robot_get_basic_time_step();
    return time_step;
}

static void step() {
    if (wb_robot_step(get_time_step()) == -1) {
        wb_robot_cleanup();
        exit(EXIT_SUCCESS);
    }
}

static void passive_wait(double sec) {
    double start_time = wb_robot_get_time();
    do {
        step();
    } while (start_time + sec > wb_robot_get_time());
}

void checkBoxCollision(WbDeviceTag* proximitySensors, WbFieldRef* boxFields, double** boxInitialPositions, WbDeviceTag* ledsVector){
  
  bool isCollision = false;
  bool lightBox = false;
  
  for(int i = 0; i < 8; i++){
  
    float sensorValue = wb_distance_sensor_get_value(proximitySensors[i]);
    
    if(sensorValue > 100){
        isCollision = true;
        break; 
    }
  
  }
  
  if(isCollision){
     
     for(int i = 0; i < 9; i++){
    
        double* posicaoBox = wb_supervisor_field_get_sf_vec3f(boxFields[i]);
             
        for(int j = 0; j < 3; j++){
    
            if(posicaoBox[j] != boxInitialPositions[i][j]){        
              lightBox = true;
              updateBoxPosition(boxInitialPositions[i],posicaoBox);
              break;
            }  
        } 
     }
     
     if(lightBox){
       activeLed(ledsVector);
     }            
  }
}

void updateBoxPosition(double* boxInitialPosition, double* boxCurrentPosition){

  for(int i = 0; i < 3; i++){ 
     boxInitialPosition[i] = boxCurrentPosition[i];  
  }

}

// inicializa sensores de proximidade
void initializeProximitySensors(WbDeviceTag* proximitySensors){

  char proximitySensorsNames[8][4] = {
    "ps0", "ps1", "ps2", "ps3",
    "ps4", "ps5", "ps6", "ps7"
  };

  
  for (int i = 0; i < 8 ; i++) {
    proximitySensors[i] = wb_robot_get_device(proximitySensorsNames[i]);
    wb_distance_sensor_enable(proximitySensors[i], TIME_STEP);
  }
  
}

// inicializa o motor
void initializeMotor(const WbDeviceTag* leftMotor, const WbDeviceTag* rightMotor){

  wb_motor_set_position(*leftMotor, INFINITY);
  wb_motor_set_position(*rightMotor, INFINITY);
  
  wb_motor_set_velocity(*leftMotor, 0.0);
  wb_motor_set_velocity(*rightMotor, 0.0);

}

// inicializa as caixas
void assignBoxes(WbNodeRef* boxes, WbFieldRef* boxFields){
  
  char boxNames[9][5] = {
    "box1", "box2", "box3", "box4",
    "box5", "box6", "box7", "box8","box9"
  };
  
  for(int i = 0; i < 9; i++){
    boxes[i] = wb_supervisor_node_get_from_def(boxNames[i]);
    boxFields[i] = wb_supervisor_node_get_field(boxes[i], "translation");
  }
}

// pega posições das caixas 
void assignBoxInitialPositions(WbFieldRef* boxFields, double** boxInitialPositions){

  for(int i = 0; i < 9; i++){
    double* posicao_box = wb_supervisor_field_get_sf_vec3f(boxFields[i]);
    
    for(int j = 0; j < 3; j++){
    
       boxInitialPositions[i][j] = posicao_box[j];
 
    }
  }
}

// detecta obstaculos na arena
void detectObstacles(WbDeviceTag* proximitySensors, ObstacleDetector* obstacleDetector){

  double proximitySensorValues[8];
 
  for (int i = 0; i < 8 ; i++){
      proximitySensorValues[i] = wb_distance_sensor_get_value(proximitySensors[i]);
  }
  
  bool right =
      proximitySensorValues[0] > 80.0 ||
      proximitySensorValues[1] > 80.0 ||
      proximitySensorValues[2] > 80.0;
      
  bool left =
      proximitySensorValues[5] > 80.0 ||
      proximitySensorValues[6] > 80.0 ||
      proximitySensorValues[7] > 80.0;
      
  obstacleDetector->right = right;
  obstacleDetector->left = left;
       
}


double** createMatrix(int l, int c){

  double** matrix = malloc(sizeof(double*) * l);
  
  for(int i = 0; i < l; i++){
      matrix[i] = malloc(sizeof(double) * c);
  }
  
  return matrix;
}

void initializeLeds(WbDeviceTag* ledsVector){

  static const char* leds_names[LEDS] = { "led0", "led1", "led2", "led3", "led4", 
  "led5", "led6", "led7", "led8", "led9" };

  for (int i = 0; i < LEDS; i++)
    ledsVector[i] = wb_robot_get_device(leds_names[i]);
}


void activeLed(WbDeviceTag* ledsVector) {
  for (int i = 0; i < LEDS; i++)
    wb_led_set(ledsVector[i], true);
    passive_wait(0.5);
}

void deactiveLed(WbDeviceTag* ledsVector) {
  for (int i = 0; i < LEDS; i++)
    wb_led_set(ledsVector[i], false);
}




// seta velocidade do robo
void setRobotVelocity(ObstacleDetector* obstacle, WbDeviceTag* leftMotor, WbDeviceTag* rightMotor){
  
  double leftSpeed  = 0.5 * MAX_SPEED;
  double rightSpeed = 0.5 * MAX_SPEED;
  
   if (obstacle->left) {
      leftSpeed  += 0.5 * MAX_SPEED;
      rightSpeed -= 0.5 * MAX_SPEED;
   }
  
   else if (obstacle->right) {
      leftSpeed  -= 0.5 * MAX_SPEED;
      rightSpeed += 0.5 * MAX_SPEED;
   }
   
   wb_motor_set_velocity(*leftMotor, leftSpeed);
   wb_motor_set_velocity(*rightMotor, rightSpeed);

}

int main(int argc, char **argv) {
 
  wb_robot_init();
  ObstacleDetector* obstacleDetector = malloc(sizeof(obstacleDetector));
  
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  WbDeviceTag proximitySensors[8];
  WbDeviceTag leds[LEDS];
   
  bool leds_values[LEDS];
  
  WbNodeRef boxes[9];
  WbFieldRef boxFields[9];
  double **boxInitialPositions = createMatrix(9,3);
  
  WbNodeRef robot_node = wb_supervisor_node_get_from_def("ePuck");
  WbFieldRef robot_field = wb_supervisor_node_get_field(robot_node, "translation");
  
  initializeProximitySensors(proximitySensors);
  initializeMotor(&left_motor, &right_motor);
  assignBoxes(boxes, boxFields);
  assignBoxInitialPositions(boxFields, boxInitialPositions);
  initializeLeds(leds);
  
  while (wb_robot_step(TIME_STEP) != -1) {
    
    deactiveLed(leds);
    detectObstacles(proximitySensors, obstacleDetector);
    setRobotVelocity(obstacleDetector, &left_motor, &right_motor); 
    checkBoxCollision(proximitySensors,boxFields,boxInitialPositions,leds);
   
  }
 
  wb_robot_cleanup();
  return 0;
}


