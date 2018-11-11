#include "DualMC33926MotorShield.h"
#include "math.h"

DualMC33926MotorShield md;
volatile long enc_count_right = 0;
volatile long enc_count_left = 0;
int pwmL = 175;
int pwmR = 175;
int error = 0;
int k = -1;
int b = -1;
unsigned long timeRunning = 0;
int turn = 0;
double sleft = 0;
double sright = 0;
int turnInt = 0;
double circ = .22225;
int total_enc_count_right = 0;
int total_enc_count_left = 0;
double x = 0;
double y = 0;
double theta = 0;
double delta_theta = 0;
double delta_d = 0;
int last_enc_count_right = 0;
int last_enc_count_left = 0;

void setup() {
  Serial.begin(115200);
  attachInterrupt(0,encoder_isr_right,CHANGE);
  attachInterrupt(1,encoder_isr_left,CHANGE);
  Serial.println("Dual MC33926 Motor Shield");
  md.init();
}

void stopIfFault() {
  if (md.getFault()) {
    Serial.println("fault");
    while(1);
  }
}

void location() {
  delta_d = (sleft + sright)/2;
  delta_theta = atan2((sleft - sright)/2, 0.1651/2);
  theta += delta_theta;
  y += delta_d*cos(theta);
  x += delta_d*sin(theta);
  //Serial.println(x);
  //Serial.print("Y: ");
  //Serial.println(y);
  //Serial.print("Theta: ");
  //Serial.println(theta);
  //Serial.print("Delta Theta: ");
  //Serial.println(delta_theta);
}

void encoder_isr_right() {
  static int8_t lookup_table[] = {0,0,0,1,0,0,-1,0,0,-1,0,0,1,0,0,0};
  static uint8_t enc_val_right = 0;
  //pin 2
  uint8_t interrupt = (PIND & 0b100) >> 2;
  //pin 6
  uint8_t second = (PIND & 0b1000000) >> 5;
  enc_val_right = enc_val_right << 2;
  enc_val_right = enc_val_right | (interrupt | second);
  enc_count_right = enc_count_right + lookup_table[enc_val_right & 0b1111];
  if (enc_count_right % 25 == 0 && turn == 0) {
    odo_close_loop();
  }
  total_enc_count_right += lookup_table[enc_val_right & 0b1111];
  turnInt += 1;
  if (enc_count_right % 8 == 0) {
     int diff_count_right = total_enc_count_right - last_enc_count_right;
     sright = diff_count_right * circ/32;
     last_enc_count_right = total_enc_count_right;
     //location();
  }
  // odo_close_loop();
  //Serial.print("MR: ");

  //  Serial.println(enc_count_right);
}

void encoder_isr_left() {
  static int8_t lookup_table[] = {0,0,0,1,0,0,-1,0,0,-1,0,0,1,0,0,0};
  static uint8_t enc_val_left = 0;
  //pin 3
  uint8_t interrupt = (PIND & 0b1000) >> 3;
  //pin 5
  uint8_t second = (PIND & 0b100000) >> 4;
  enc_val_left = enc_val_left << 2;
  enc_val_left = enc_val_left | (interrupt | second);
  enc_count_left = enc_count_left + lookup_table[enc_val_left & 0b1111];
  total_enc_count_left += lookup_table[enc_val_left & 0b1111];
  if (enc_count_left % 8 == 0) {
    int diff_count_left = total_enc_count_left - last_enc_count_left;
    sleft = diff_count_left * circ/32;
    last_enc_count_left = total_enc_count_left;
    //location();
  }
  //  if (enc_count_left % 20 == 0) {
  //   odo_close_loop();
  //  }
  //Serial.print("ML: ");
  //  Serial.println(enc_count_left);
}

void odo_close_loop() {
    int error_last = error;
    int delta_error = 0;
    int delta_v;
    if (enc_count_right > enc_count_left) {
      error = enc_count_right - enc_count_left;
      delta_error = error - error_last;
      delta_v = -k*error - b*delta_error;
      pwmR -= delta_v;
      pwmL += delta_v;
    }
    else if (enc_count_left > enc_count_right) {
      error = enc_count_left - enc_count_right;
      delta_error = error - error_last;
      delta_v = -k*error - b*delta_error;
      pwmR += delta_v;
      pwmL -= delta_v;
    }

    //Serial.print("Error: ");
    //Serial.println(delta_error);
    //Serial.print("Right PWM: ");
    //Serial.println(pwmR);
    //Serial.print("Left PWM: ");
    //Serial.println(pwmL);

}

void loop() {
   unsigned long timeRunning = millis();
  
   if (turnInt % 20 == 0){
     location();
   } 
   if (turnInt >= 245) {
     if (turn == 0) {
       Serial.print("X: ");
       Serial.println(x);
       Serial.print("Y: ");
       Serial.println(y);
       Serial.print("Theta: ");
       Serial.println(theta);
     }
     turn = 1;
     md.setM1Speed(0);  // Right Motor
     md.setM2Speed(0);  // Left Motor
   }
   else if (turnInt >= 192) {
     if (turn == 1) {
       enc_count_right = 0;
       enc_count_left = 0;
     }
     turn = 0;
     md.setM1Speed(pwmR);
     md.setM2Speed(pwmL);
   }
   else if (turnInt >= 160) {
     if (turn == 0) {
       enc_count_right = 0;
       enc_count_left = 0;
     }
     turn = 1;
     md.setM1Speed(pwmR - 20);
     md.setM2Speed(pwmL + 230 - 20);
   }
   else {
     turn = 0;
     md.setM1Speed(pwmR);
     md.setM2Speed(pwmL);
   }
   stopIfFault();
   delay(1);
 }
