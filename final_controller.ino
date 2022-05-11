#define NRF_CE 38
#define NRF_CSN 39
#define JOYSTICK_DEADZONE 4

#include "NEET_RF24.h"
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>

// Declare the radio and motors
NEET_RF24 radio(NRF_CE, NRF_CSN, 63);

int cnt = 1;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x60); // solder address jumpers on any stacked boards
Adafruit_MotorShield AFMS_DC = Adafruit_MotorShield(0x61); // solder address jumpers on any stacked boards

Adafruit_DCMotor *motor_left_front = AFMS.getMotor(2);
Adafruit_DCMotor *motor_right_front = AFMS.getMotor(1);
Adafruit_DCMotor *motor_left_back = AFMS.getMotor(3);
Adafruit_DCMotor *motor_right_back = AFMS.getMotor(4);


Adafruit_DCMotor *DCMotor = AFMS_DC.getMotor(2);
Adafruit_DCMotor *encDCMotor = AFMS_DC.getMotor(4);

long previousMillis = 0;
long currentMillis = 0;

// Min and max values for setSPeed (PWM for DC motors)
const int minSpeed = -255; // Minimum motor speed. (-) range used to reverse motor direction
const int maxSpeed = 255;  // Maximum motor speed. (+) range used to forward motor direction





Servo servo_base;
Servo servo_grippers;
Servo servo_buttons;



// Holds the input from controller
ControlInput input;

int8_t last_servo = 0;

int t_twist = 1000;







void setup(){
  Serial.begin(115200);
  if (!radio.begin()){
    Serial.println("Radio not started");
  } else {
    Serial.println("Radio started");
  }
  
  AFMS.begin();
  AFMS_DC.begin();
  

  servo_base.attach(10);
  servo_grippers.attach(9);
  servo_buttons.attach(11);

  servo_base.write(0);
  servo_grippers.write(45);
  servo_buttons.write(180);
  radio.rxSendTelemetry("Hello world! " + String(analogRead(A0)));
}



void loop(){
  // If radio input is valid
  if (radio.rxUpdate()){
    // Gets value from radio
    input = radio.rxGetInput();


    //Drive:
    int left = 0;
    int right = 0;
    // Don't process unless the joysticks are more than 5 away from center
    if (abs(input.j1PotY) > JOYSTICK_DEADZONE){
      left = input.j1PotY*1.5;
    }
    if (abs(input.j2PotX) > JOYSTICK_DEADZONE){
      right = input.j2PotX;
    }
    // Arcade drive
    drive(left + right , left - right);





    

//tswitch:
    if (input.tSwitch == true) {
      servo_buttons.write(0);    
    }
    else if (input.tSwitch == false) {
      servo_buttons.write(180);
    }

  
  
    
 


//Potentiometer:

    if (abs(input.pot - last_servo) > 5){
      servo_base.write(map(input.pot, -127, 127, 0, 180));
      last_servo = input.pot;
    }




//1
    //Linear Motor UP: keep pressing  1
    if (input.button1 == true) {
        drive_encDC(255,50);     
    } 
 
//2
    //Linear Motor DOWN: 2
    if (input.button2 == true) {  
        drive_encDC(-255,50);       
    } 
    




 //3   
    if (input.button3 == true) {  
        driveDC(150,15);       
    } 
  
  
    


  
//4
    //Tgrip: 4
    if (input.button4 == true) {

        servo_grippers.write(15);
           
    } 
    else if (input.button4 == false) {

        servo_grippers.write(45);
           
    }

   



  } else {
    drive(0, 0);
  }
}

/**
 * @brief Drives the robot by interfacing with the motor controller shield
 * 
 * @param left integer value between -255 and 255 for left motor
 * @param right integer value between -255 and 255 for right motor
 */




//linear actuator:
void drive_encDC(int pwm, unsigned int MS) {
    
    previousMillis = millis();
    while (millis() - previousMillis < MS) {
        int constrained_pwm = constrain(pwm, minSpeed, maxSpeed);
        if (pwm > 0) {
            encDCMotor->run(FORWARD);
        } else if (pwm < 0) {
            encDCMotor->run(BACKWARD);
        }
        encDCMotor->setSpeed(abs(constrained_pwm));
    }
    encDCMotor->setSpeed(0);
    encDCMotor->run(RELEASE);
}



//TWISTER:
void driveDC(int pwm, unsigned int MS) {
    previousMillis = millis();
    while (millis() - previousMillis < MS) {
        int constrained_pwm = constrain(pwm, minSpeed, maxSpeed);
        if (pwm > 0) {
            DCMotor->run(FORWARD);
        } else if (pwm < 0) {
            DCMotor->run(BACKWARD);
        }
        DCMotor->setSpeed(abs(constrained_pwm));
    }
    DCMotor->setSpeed(0);
    DCMotor->run(RELEASE);
}



//WHEELS:
void drive(int left, int right){
//  right = -right;
//  left = -left;
  
  motor_left_front->setSpeed(abs(left));
  motor_left_back ->setSpeed(abs(left));
  
  motor_right_front->setSpeed(abs(right));
  motor_right_back ->setSpeed(abs(right));

  if (left > 0){
    motor_left_front->run(FORWARD);
    motor_left_back->run(FORWARD); 
  } else if (left < 0){
    motor_left_front->run(BACKWARD);
    motor_left_back->run(BACKWARD);
  } else {
    motor_left_front->run(RELEASE);
    motor_left_back->run(RELEASE);
  }




  if (right > 0){
    motor_right_front->run(FORWARD);
    motor_right_back->run(FORWARD);
  } else if (right < 0){
    motor_right_front->run(BACKWARD);
    motor_right_back->run(BACKWARD);
  } else {
    motor_right_front->run(RELEASE);
    motor_right_back->run(RELEASE);
  }
}





