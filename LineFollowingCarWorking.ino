// ANCH150 HW#8- Line Following Automated Car - 2017

//****DO NOT CHANGE****
// Pin Declarations
int LeftMotorDir = 12, LeftMotorPower = 3; 
int RightMotorDir = 13, RightMotorPower = 11; 

//****DO NOT CHANGE****
// Decleration for two analog input light sensors
int left_light = A0, right_light = A1;  

//***CHANGE LIGHT SENSOR CALIBRATION VALUES***
// Light Sensor Calibration Values
// These you need to set manually (use the LCD to read the values).  Make sure to give yourself a buffer
//  If line reads 999, may use something like ~925 for your dark value (so you don't miss it).
int dark = 600;  

//***CHANGE MOTOR SPEEDS***
// Motor maximum and minimum speeds
// Adjust these within the 0-255 range to try and make your bot faster
//***CHANGE MOTOR MIN AND MAX SPEEDS***
int motor_max = 225, motor_min = 5;  

//****DO NOT CHANGE****
// Milliseconds per cycle -> 10ms would yield loop speed of 100Hz
int ms_per_cycle = 10;  
// This is updated each time the motors are updated to make sure everything runs exactly at desired loop speed
unsigned long previous_time = 0;

/* 
The motor speeds are often not matched meaning that the car won't travel straight. 
You will need to adjust so the faster motor goes the same speed as the slower motor. 
The range of speed is from 0 to 255. 255 is the maximum speed.
Change the values of leftMotor and rightMotor until the car will travel straight. 
To find the values that make the car go straight, determine the radial velocity of each wheel and adjust
proportionally. That or it can be accomplished by trial and error. 
Try lowering the duty cycle of the faster motor, little by little.
Be patient!
*/

// Change these values to CALIBRATE MOTORS SEPARATELY
int leftMotor = 240, rightMotor = 205;
#include <LiquidCrystal.h>
#include <Time.h>
time_t elapsedTime;   //to restart the time counter to Zero:  setTime(0);
LiquidCrystal lcd(12,11,5,4,3,2);

float input_voltage = 0.0;
float temp=0.0;


void setup() {
  pinMode(LeftMotorDir, OUTPUT);
  pinMode(RightMotorDir, OUTPUT);
  pinMode(LeftMotorPower, OUTPUT);
  pinMode(RightMotorPower, OUTPUT);
  Serial.begin(9600);
  lcd.begin(16, 2);
  lcd.clear();
  //lcd.print("DIGITAL VOLTMETER");
}

//**Car Logic - Tread Carefully**
void loop() {

 /*  int analog_value = analogRead(A0);
   input_voltage = (analog_value * 5.0) / 1024.0; 

   
   if (input_voltage < 0.1) 
   {
     input_voltage=0.0;
   } 
    Serial.print("v= ");
    Serial.println(input_voltage);
    lcd.setCursor(0, 1);
    lcd.print("Voltage= ");
    lcd.print(input_voltage);
    delay(300);
   */ 
   if (millis() - previous_time >= ms_per_cycle)  // if time since last motor update is >= desired delta t then update motors
   {
     previous_time = millis(); // Update the cycle timer
     
     int right_reading = analogRead(right_light); // Read from the two light sensors
     int left_reading = analogRead(left_light);

      //lcd.clear();
      //lcd.setCursor (0,0);
      //lcd.print("Hi! Im TimTim :D");
      
      //elapsedTime = now();
      //lcd.setCursor (0,1);               // sets cursor to 2nd line
      //lcd.print("Min: ");
      //lcd.print(minute(elapsedTime));
      //lcd.setCursor(9,1);               // sets cursor to 2nd line
      //lcd.print("Sec: ");
      //lcd.print(second(elapsedTime));
     
     Serial.begin(9600);
     //Serial.println(right_reading); Reading of Right Line Sensor
     //Serial.println(left_reading); Reading of Left Line Sensor
     
     if (right_reading >= dark && left_reading < dark)  // See the line on the right side of the robot.
     {                                                  // Need to turn the robot right to keep the line over the robot.
       digitalWrite(LeftMotorDir, HIGH);  // Left motor forward 
       analogWrite(LeftMotorPower, motor_min);  // Max power for left motor
       digitalWrite(RightMotorDir, HIGH); // Right motor forward
       analogWrite(RightMotorPower, motor_max); // Minimum power for right motor
     }
     else if (right_reading < dark && left_reading >= dark) // See line on the left side of the robot
     {
       digitalWrite(LeftMotorDir, HIGH);  // Left motor forward 
       analogWrite(LeftMotorPower, motor_max);  // Min power for left motor
       digitalWrite(RightMotorDir, HIGH); // Right motor forward
       analogWrite(RightMotorPower, motor_min); // Max power for right motor
     }
     else if (right_reading < dark && left_reading < dark) // Neither sensor sees line, must be straddled... Go straight
     {
       digitalWrite(LeftMotorDir, HIGH);  // Left motor forward 
       analogWrite(LeftMotorPower, motor_max);  // Max power for left motor
       digitalWrite(RightMotorDir, HIGH); // Right motor forward
       analogWrite(RightMotorPower, motor_max); // Max power for right motor
     }
     else 
     {
       // Both sensors must see black or there is a problem.  Turning motors off
       digitalWrite(LeftMotorDir, HIGH);  // Left motor forward 
       analogWrite(LeftMotorPower, 50);  // Turning motor off
       digitalWrite(RightMotorDir, HIGH); // Right motor forward
       analogWrite(RightMotorPower, 50); // Turning motor off
     }
   } // End of cycle timer if statement 
   
    //delay(1000);
   // If the if statement is false (hasn't been enough time since last motor update) then the program will simply skip the if statement
   // and loop around very very quickly.
   



//lcd.setCursor(6, 0);

//lcd.setCursor(0, 1);
}
