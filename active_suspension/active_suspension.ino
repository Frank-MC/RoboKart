//include:
#include <Servo.h>
#include <Timer.h>
Timer t;

//servo attackments
Servo Servo1;
Servo Servo2;
Servo Servo3;
Servo Servo4;
Servo Servo5;

//define existing states
enum State {                    //define the state
  State_Active,                 //actieve suspension on
  State_Test,                   //Test state for all servo's
} state;

//Pin defenitions:              // -1 is not yet defined
//axceleration&gyro senor pins:
int accel_x_pin = -1;
int accel_y_pin = -1;
int accel_z_pin = -1;
int ang_accel_x_pin = A0;
int ang_accel_y_pin = A2;
int ang_accel_z_pin = -1;

//steering input (rc)
int steering_input_pin = -1;

//servo actuator pins
//suspension                      //location servo's
int servo1_pin = 7;             //  1-2   x       1&2 voorkant wagen
int servo2_pin = 3;             //  | |   ^
int servo3_pin = 6;             //  3-4   |
int servo4_pin = 2;             //  y<- axis
//steering
int steering_output_pin = -1;   //servo for steering

//setup
//multipliers
float multiplier_steer = 1;       //make the suspension react more or less to steering
float multiplier_accel = 1;       //make the suspension react more or less to acceleration
float multiplier_rot = 1;         //make the suspension react more or less to rotations

int multiplier_accel_steering_conversion = 0.2;   //ang acceleration to steering
int multiplier_steering_correction = 0.2;   //stuurgevoeligheid correctie for active steeringcorrection

//rate conversion
float VtoG = 1;                   //amount of volt measured for 1 m/s^2
float VtoAngleA = 1;              //amount of volt measrued for 1 rad/s^2

//additional:
//active suspension servo angle limit
int Min_Servo_Angle = -90;          //minimum servo angle from center
int Max_Servo_Angle = 90;           //maximum servo angle from center
int Servo_Center_Angle = 90;        //servo default angle
//active steering settings
int steering_correction = 0;        //0 is off 1 is on
int steering_threshold = 100;       // active steering off until threshold is surpassed

//store / temp_store / bridge
float accel_x;
float accel_y;
float accel_z;
float rot_accel_x;
float rot_accel_y;
float rot_accel_z;
float steer_angle;
float steering_corrected_vallue;
float steerring_correction;

float tilt1;
float tilt2;
float mapmin;
float mapmax;

float action1;
float action2;
float action3;
float action4;

void setup() {                                //setup
  Serial.begin(9600);
  while (! Serial);
  Serial.println("Wake up");

  pinMode(accel_x_pin, INPUT);
  pinMode(accel_y_pin, INPUT);
  pinMode(accel_z_pin, INPUT);
  pinMode(ang_accel_x_pin, INPUT);
  pinMode(ang_accel_y_pin, INPUT);
  pinMode(ang_accel_z_pin, INPUT);
  pinMode(servo1_pin, OUTPUT);
  pinMode(servo2_pin, OUTPUT);
  pinMode(servo3_pin, OUTPUT);
  pinMode(servo4_pin, OUTPUT);

  Servo1.attach(servo1_pin);
  Servo2.attach(servo2_pin);
  Servo3.attach(servo3_pin);
  Servo4.attach(servo4_pin);
  Servo5.attach(steering_output_pin);

  state = State_Active;      //State_Test;          //Begin state

  switch (state) {
    case State_Active:
      init_Active();
      break;
    case State_Test:
      init_Test();
      break;
  }
}

void loop() {
  t.update();                   //update clock
  t.every(10, update_sensor);  //update sensor readings
  t.every(10, calcuate);       //update servo angels and apply
  //  t.every(1000, report_Back_input);
  t.every(2000, report_back_servo);

  if (steering_correction = 1) {                                //steering correction
    steer_angle = digitalRead(steering_input_pin);

    if ((steer_angle - rot_accel_z * multiplier_accel_steering_conversion) < steering_threshold) {
      steerring_correction = 0;
    } else {
      steerring_correction = multiplier_steering_correction;
    }

    steering_corrected_vallue = steer_angle + (steer_angle - rot_accel_z) * steerring_correction;
    Servo5.write(steering_corrected_vallue);
  }

 // Actuate the suspension servo's -> verplaatst naar switch state
//  Servo1.write(Servo_Center_Angle + action1);
//  Servo2.write(Servo_Center_Angle + action2);
//  Servo3.write(Servo_Center_Angle + action3);
//  Servo4.write(Servo_Center_Angle + action4);

}

void update_sensor() {                                        //analogRead is 10bit ADC
  accel_x = analogRead(accel_x_pin) / VtoG;
  accel_y = analogRead(accel_y_pin) / VtoG;
  accel_z = analogRead(accel_z_pin) / VtoG;
  rot_accel_x = analogRead(ang_accel_x_pin) / VtoAngleA;
  rot_accel_y = analogRead(ang_accel_y_pin) / VtoAngleA;
  rot_accel_z = analogRead(ang_accel_z_pin) / VtoAngleA;

  if (steering_correction = 0) {
    steer_angle = digitalRead(steering_input_pin);
  }
}

void calcuate() {

  tilt1 = multiplier_rot * rot_accel_x + multiplier_steer * steer_angle;                    // around x-axis
  tilt2 = multiplier_rot * rot_accel_y + multiplier_accel * accel_x;                        // around y-axis

  if (tilt1 + tilt2 < 1023)  {
    mapmin = -1023/2;
    mapmax = 1023/2;
  } else {
    mapmin = -(tilt1 + tilt2)/2;
    mapmax = (tilt1 + tilt2)/2;
  }

  action1 = map( - tilt1 + tilt2 , mapmin , mapmax, Min_Servo_Angle , Max_Servo_Angle);
  action2 = map(   tilt1 + tilt2 , mapmin , mapmax, Min_Servo_Angle , Max_Servo_Angle);
  action3 = map( - tilt1 - tilt2 , mapmin , mapmax, Min_Servo_Angle , Max_Servo_Angle);
  action4 = map(   tilt1 - tilt2 , mapmin , mapmax, Min_Servo_Angle , Max_Servo_Angle);

}

void init_Active()  {
 // Actuate the suspension servo's
  Servo1.write(Servo_Center_Angle + action1);
  Servo2.write(Servo_Center_Angle + action2);
  Servo3.write(Servo_Center_Angle + action3);
  Servo4.write(Servo_Center_Angle + action4);
}

void init_Test() {
  Serial.println("test");
  Servo1.write(Max_Servo_Angle);
  Servo2.write(Max_Servo_Angle);
  Servo3.write(Max_Servo_Angle);
  Servo4.write(Max_Servo_Angle);
  delay(1000);
  Servo1.write(Min_Servo_Angle);
  Servo2.write(Min_Servo_Angle);
  Servo3.write(Min_Servo_Angle);
  Servo4.write(Min_Servo_Angle);
  delay(1000);
  Servo1.write(Max_Servo_Angle);
  Servo2.write(Max_Servo_Angle);
  Servo3.write(Max_Servo_Angle);
  Servo4.write(Max_Servo_Angle);
  delay(1000);
  Servo1.write(Min_Servo_Angle);
  Servo2.write(Min_Servo_Angle);
  Servo3.write(Min_Servo_Angle);
  Servo4.write(Min_Servo_Angle);
  delay(1000);
  Serial.println("test over");
  state = State_Active;
}

void report_Back_input() {
  Serial.print(rot_accel_x);
  Serial.println(" rad/s x-axis");

  Serial.print(rot_accel_y);
  Serial.println(" rad/s y-axis");

  Serial.print(rot_accel_z);
  Serial.println(" rad/s z-axis");

  Serial.println();

  Serial.println(tilt1);
  Serial.println(tilt2);

  Serial.println();
}

void report_back_servo() {
  Serial.print(action1);
  Serial.println(" angle1");

  Serial.print(action2);
  Serial.println(" angle2");

  Serial.print(action3);
  Serial.println(" angle3");

  Serial.print(action4);
  Serial.println(" angle4");

  Serial.println();
}
