#include <phys253.h>
#include <LiquidCrystal.h>

// Claw Arm Servo (HobbyKing) Angle Position Constants: 
#define bin_drop_mode 0
#define default_mode  45
#define pick_up_mode  135


// Claw Servo (Tower) Angle Position Constants: 
#define open_claw_mode 0
#define close_claw_mode 90

// TINAH Analog Pins Reference
#define left_claw_servo 0
#define right_claw_servo 1

// TINAH Digital Pins Reference
#define left_limit_switch 7
#define right_limit_switch 8

// other constants
#define delay_time 300
#define QSD_left false
#define QSD_right true
bool pickuped;

void setup() {
  #include <phys253setup.txt>
  Serial.begin(9600);
  pinMode(left_limit_switch, OUTPUT);
  pinMode(right_limit_switch, OUTPUT);

}

void loop() {
  motor.stop_all();
  if(QSD_left){
    // make the claw wide open before lowering for pick up t prevent a collison
    analogWrite(left_claw_servo, open_claw_mode);
    delay(delay_time);
    
    // Lower the arm 
    RCServo0.write(pick_up_mode);
    delay(delay_time);
    
    // pick up 
    analogWrite(left_claw_servo, close_claw_mode);
    delay(delay_time);

    // confirm whether picked up via the limit switch 
    if (digitalRead(left_limit_switch) == HIGH) {pickuped = true;}
    else{pickuped = false;}

    // Raise the arm 
    RCServo0.write(bin_drop_mode);
    delay(delay_time);

    // release into the bin
    analogWrite(left_claw_servo, open_claw_mode);
    delay(delay_time);
  }

  else if (QSD_right){
    // make the claw wide open before lowering for pick up t prevent a collison
    analogWrite(right_claw_servo, open_claw_mode);
    delay(delay_time);
    
    // Lower the arm 
    RCServo1.write(pick_up_mode);
    delay(delay_time);
    
    // pick up 
    analogWrite(right_claw_servo, close_claw_mode);
    delay(delay_time);

    // confirm whether picked up via the limit switch 
    if (digitalRead(right_limit_switch) == HIGH) {pickuped = true;}
    else{pickuped = false;}

    // Raise the arm 
    RCServo1.write(bin_drop_mode);
    delay(delay_time);

    // release into the bin
    analogWrite(right_claw_servo, open_claw_mode);
    delay(delay_time);
  }

  else{}
}
