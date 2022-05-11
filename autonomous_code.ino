int mode = 2; //for PHW 2

const float Ki = 0.0; 


const float Kp_up = 7.5; //Proportional parameter
const float Kd_up = 2.5;  //Derivative parameter


//const float Kp_flat[12] = {0, 1,3,5,10, 15, 20, 25,30,35,40,45}//Proportional parameter
//const float Kd_flat[12] = {0, 1,3,5,10, 15, 20, 25,30,35,40,45};  //Derivative parameter


const float Kp_flat = 7.5;
const float Kd_flat = 2.5;

const float Kp_down = 10.0; //Proportional parameter
const float Kd_down = 5.0;  //Derivative parameter




//LINE FOLLWING SENSOR VALUES TESTING:
int pin_arr[5] = {A8, A9, A10, A11, A12}; //Analog pins assignments for each sensor cell
float y_prime; //current y_val ranging from 0 to 5 (target y_val = 3)
float y_arr[5];
const float minim = 50; //min raw output of sensors
const float maxim = 1000; //max raw output of sensors
const float non_black_threshold = 0.85; //y value after which bot is confident that given cell is above non-black surface
const float black_threshold = 0.65; //confidently black below this value
int n; //number of sensors in black line

int black_cross_num;

//PID Variables:
float y = 3.0; //target value

float integral;
float previousError;


unsigned long currentTime, previousTime;
double elapsedTime;




//LED:
#define LED 42

//MOTION CONTROL:
//const int Speed = 70; 0-4

int Speed = 80;

#include <Adafruit_MotorShield.h>
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorLeft = AFMS.getMotor(1);
Adafruit_DCMotor *motorRight = AFMS.getMotor(2);
float frac = 0.95; //left_speed = frac* right_speed







//IMU:
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55);




//Ultrasonic:
#include <HCSR04.h>
HCSR04 hc(3, 2); //initialisation class HCSR04 (trig pin , echo pin)



int cnt;
void setup() {
  Serial.begin(9600);
  
  initialize_IMU();
  

  previousError = 0;
  integral = 0;
  black_cross_num = 0;

  //Line following sensor:
 
  for (int i = 0; i < 5; i++) {
    pinMode(pin_arr[i], INPUT);
  }
  //LED:
  pinMode(LED , OUTPUT);

  //Motors:
  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
    // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  delay(3000);

}






void loop() {
  y_prime = get_yprime(); //get current y-value 
  
  if (hc.dist() <= 10.0) { //barrier
    cnt++;
    drive_backward(80);
    delay(400);
    brake();
    delay(500);
    rotate_angle_usingIMU(170.0);
    drive_forward(60);
    delay(4000);
        
  }
  
//  if (n >=3) { //if all sensors detect black surface
//    communicate_status(); //blink LED as bot crosses horizontal lines
//  }

  PID(y_prime);
}

//void loop () {
//  rotate_angle_usingIMU(180.0);
//}



void PID(float yp) {
  /*
     Implement PID controller given current y_value
  */
  currentTime = millis();                //get current time
  elapsedTime = (double)(currentTime - previousTime);
//  Serial.print("elapsed time = ");  Serial.println(elapsedTime); Serial.println();
  float error = yp - y;

  integral += error * elapsedTime ; //right reimann sum or right rectangle piece-wise approx.
  float derivative = (error - previousError) / elapsedTime; //forward difference or forward Euler

  float Kp; 
  float Kd;
  
  if (get_pitch() >5) {
    Speed = 80.0;
    Kp = Kp_up; Kd = Kd_up; 
  }
  else if (abs(get_pitch()) <=5) {
    Speed = 60.0;
    Kp = Kp_flat; Kd = Kd_flat; 
  }

  else if (get_pitch() < -5) {
    Speed = 40.0;
    Kp = Kp_down; Kd = Kd_down;   
  }
  
  float del = Kp * error + Ki * integral + Kd * derivative;
  previousError = error;
  previousTime = currentTime;
  
  drive(Speed + del, Speed - del); //move wheel to adjust y value

}


float get_pitch() {
  sensors_event_t event;
  bno.getEvent(&event);
  return event.orientation.z;
}

float get_yaw() {
  sensors_event_t event;
  bno.getEvent(&event);
  return event.orientation.x;
}





float get_yprime() {
  /*
     returns number between 0.0 to 5.0
     0.0 means bot is NOT in black line
     1.0 to 5.0 indicates position of line following sensor in black line
  */
  float y_prime = 0.0; //output from sensor (to be updated)
  float sum = 0.0;
  float y_raw;  //raw analog output values from sensors



  n = 0;
  for (int i = 0; i < 5; i++) {
    //    if (i == 0) {y_raw = (float) analogRead(pin_arr[i]); }  //adjust offset on sensor 1
    //    else{y_raw = (float) analogRead(pin_arr[i]);}
    y_raw = (float) analogRead(pin_arr[i]);

//    Serial.print("y_raw = ");  Serial.println(y_raw);
    float yi = (y_raw - minim) / (maxim - minim);
    y_arr[i] = yi;
    if (yi >= non_black_threshold) {
      yi = 1;
    }
    else if (yi <= black_threshold) {
      yi = 0;
      n++;
    }

    sum += yi;
    y_prime += (1 - yi) * i;
  }

  y_prime = y_prime / (5 - sum) + 1;

  if (isnan(y_prime)) {
//    Serial.println("Bot in non-black region!");
    y_prime = previousError + y; //new
  }
  return y_prime;
}





void communicate_status() { //change
  brake();
  black_cross_num++; //increase counter for number of black lines crossed
  //blink_led(1); 
  blink_led(black_cross_num);
  
  if (mode == 1) { //a1 to a2
    if (black_cross_num < 7) {
      drive_forward(Speed);
    }
    else {
      brake();
      delay(5000);
    }
  }

  else if (mode == 2) { //a3 to a4
    if (black_cross_num <= 3 || black_cross_num == 16) {
      drive_forward(Speed);
    }
    
    else if (black_cross_num >3 && black_cross_num <= 15) {
      drive_forward(150);
    }   
    else if (black_cross_num == 17){
      brake();
      delay(5000);
    }
    else {
      drive_forward(120);     
    }
    
  }

  delay(200); //  //to cross that line without using PID

}



//IMU:

void initialize_IMU() {
  /* Initialise the IMU sensor */
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  bno.setExtCrystalUse(true);
}


void print_YPR() {
  sensors_event_t event;
  bno.getEvent(&event);

  Serial.print("Yaw angle: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\t Roll angle: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\t Pitch angle: ");
  Serial.print(event.orientation.z, 4);
  Serial.println(""); 
}




void rotate_angle_usingIMU(float angle) {

  if (angle >= 0 ) { //CCW'
    motorLeft->run(FORWARD); motorRight->run(BACKWARD);    
  }
  
  else if (angle < 0 ) { //CW'
    motorLeft->run(BACKWARD); motorRight->run(FORWARD);  
  }   

  
  
  motorLeft->setSpeed(Speed * frac);
  motorRight->setSpeed(Speed);
  
  for (int i = 0; i < 1000000; i++) {
    Serial.println(i);
    sensors_event_t event;
    bno.getEvent(&event);
    if (event.orientation.x >= angle ) {    
      break;       
    }  
  }
  brake();
  delay(1000);  
}











//MOTION CONTROL FUNCTIONS:
void rotate(int delay_time, int dir) { 
  /*
   * dir = 1 is CCW 
   * dir = 2 is CW
   */
  if (dir == 1) { //CCW'
    motorLeft->run(BACKWARD); motorRight->run(FORWARD);  
  }
  else if  (dir == 2) { //CCW'
    motorLeft->run(FORWARD); motorRight->run(BACKWARD);  
  }  
  motorLeft->setSpeed(Speed * frac);
  motorRight->setSpeed(Speed);
  //TODO
}



void drive(int left_speed, int right_speed) {
  if (left_speed >= 0) {
    motorLeft->run(FORWARD);
  }
  else {
    motorLeft->run(BACKWARD);
  }

  if (right_speed >= 0) {
    motorRight->run(FORWARD);
  }
  else {
    motorRight->run(BACKWARD);
  }

  motorLeft->setSpeed(left_speed * frac);
  motorRight->setSpeed(right_speed);
}


void brake() {
  motorLeft->run(RELEASE);
  motorRight->run(RELEASE);
}


void drive_forward(int motorSpeed) {
  /*
     Runs forward with given motorspeed until stopped
  */
  motorLeft->run(FORWARD); motorRight->run(FORWARD);

  motorLeft->setSpeed(motorSpeed*0.90);
  motorRight->setSpeed(motorSpeed);
}

void drive_backward(int motorSpeed) {
  /*
     Runs backward with given motorspeed until stopped
  */
  motorLeft->run(BACKWARD); motorRight->run(BACKWARD);

  motorLeft->setSpeed(motorSpeed * 0.70);
  motorRight->setSpeed(motorSpeed);
}


//LED blink count black crossings:
void blink_led(int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(LED , HIGH);//turn the LED On by making the voltage HIGH
    delay(300);                       // wait half a second
    digitalWrite(LED , LOW);// turn the LED Off by making the voltage LOW
    delay(300);
  }
}
