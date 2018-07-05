#include "Arduino.h"
#include "Robot.h"
#include <avr/EEPROM.h>

// Analog pin reference
#define left_claw_servo    0
#define right_claw_servo   1
#define left_bridge_servo  2
#define right_bridge_servo 3
#define left_QRD           4
#define right_QRD          5
#define gyro_SCL           6
#define gyro_SDA           7

// Digital pin reference
#define left_encoder       0
#define right_encoder      1
#define gyro_INT           2
#define sonar_trig         3
#define sonar_echo         4
#define IR_signal_10kHz    5
#define IR_signal_1kHz     6
#define left_limit_switch  7
#define right_limit_switch 8
#define QSD                9
#define IR_LED             10 

const int delay_time = 400;
bool captured = false;

// angle position of the arm and clas servos for various modes
const int arm_raised = 0;
const int arm_lowered = 135;
const int claw_open = 0;
const int claw_closed = 90;

/* Purpose: Initialize the robot before activating
 * Input: none
 * Output: none
 */
void Robot::Initialize() {
    RCServo0.write(arm_raised);
    RCServo1.write(arm_raised);
    
    pinMode(left_limit_switch,INPUT);
    pinMode(right_limit_switch,INPUT);
}


/* Purpose: Pick up the Ewok or Chewbacca using the claw and place into the bin
 * Input: bool left_side (true to activate leftwing, false to activate rightwing
 * Output: none
 */

void Robot::pickup(bool left_side){
    motor.stop_all();
    
    // activate either the left or right pickup system base on the input
    if(left_side) {
        // open the claw
        analogWrite(left_claw_servo, claw_open);
        delay(delay_time);
        
        // lower the arm
        RCServo0.write(arm_lowered);
        delay(delay_time);
        
        // close the claw
        analogWrite(left_claw_servo, claw_closed);
        delay(delay_time);
        
        // confirm whether the item is captured via the limit switch
        if (digitalRead(left_limit_switch) == HIGH) {captured = true;}
        else{pickuped = false;}
        
        // raise the arm
        RCServo0.write(arm_raised);
        delay(delay_time);
        
        // release the item into the bin
        analogWrite(left_claw_servo, claw_open);
        delay(delay_time);
    }
    
    else{
        // open the claw
        analogWrite(right_claw_servo, claw_open);
        delay(delay_time);
        
        // lower the arm
        RCServo1.write(arm_lowered);
        delay(delay_time);
        
        // close the claw
        analogWrite(right_claw_servo, claw_closed);
        delay(delay_time);
        
        // confirm whether the item is captured via the limit switch
        if (digitalRead(right_limit_switch) == HIGH) {captured = true;}
        else{pickuped = false;}
        
        // raise the arm
        RCServo1.write(arm_raised);
        delay(delay_time);
        
        // release the item into the bin
        analogWrite(right_claw_servo, claw_open);
        delay(delay_time);
    }
    
    
    /* Purpose: Print a message on the TINAH LCD screen
     * Input: char* message (the message to be displayed)
     * Output: none
     */
    
    void Robot::displayLCD(char* message){
        LCD.clear(); LCD.home();
        LCD.setCursor(0, 0); LCD.print(message);
    }
}


