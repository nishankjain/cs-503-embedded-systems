#include "DualMC33926MotorShield.h"
#include "math.h"

DualMC33926MotorShield md;
volatile long enc_count_right = 0;
volatile long enc_count_left = 0;
long pwmL = 100;
long pwmR = 100;
float v_error = 0;
float k = -0.5;
float b = 0.05;
long turn = 0;
float delta_sleft = 0;
float delta_sright = 0;
float circ = .22225;
volatile long total_enc_count_right = 0;
volatile long total_enc_count_left = 0;
float x = 0;
float y = 0;
float theta = 0;
float delta_theta = 0;
float delta_d = 0;
volatile long last_enc_count_right = 0;
volatile long last_enc_count_left = 0;
float previousTime = 0;
float currentTime = 0;
float interval = 1000;
float actualInterval = 0;
long last_interrupt_left_time = 0;
long last_interrupt_right_time = 0;
float interrupt_right_interval = 0;
float interrupt_left_interval = 0;
float wheel_base = 0.1651;
float vref = 0.12;
const float pi = 3.1415926;
float delta_theta_degrees = 0;
static int8_t lookup_table[] = {0,0,0,1,0,0,-1,0,0,-1,0,0,1,0,0,0};

void setup() {
  Serial.begin(115200);
  attachInterrupt(0, encoder_isr_right, CHANGE);
  attachInterrupt(1, encoder_isr_left, CHANGE);
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
  delta_sright = float(total_enc_count_right - last_enc_count_right) * circ / 32;
  delta_sleft = float(total_enc_count_left - last_enc_count_left) * circ / 32;
  last_enc_count_right = total_enc_count_right;
  last_enc_count_left = total_enc_count_left;
  delta_d = (delta_sleft + delta_sright) / 2;
  delta_theta = atan2((delta_sright - delta_sleft) / 2, wheel_base / 2);
  delta_theta_degrees = delta_theta * (180 / pi);
  theta += delta_theta;
  x += delta_d * cos(theta);
  y += delta_d * sin(theta);
//  Serial.print("Delta Theta: ");
//  Serial.println(delta_theta);
//  Serial.print("X: ");
//  Serial.println(x);
//  Serial.print("Y: ");
//  Serial.println(y);
  if (turn == 0) {
    odo_close_loop();
  }
  //Serial.print("Theta: ");
  //Serial.println(theta);
  Serial.print("Right: ");
  Serial.print(delta_sright);
  Serial.print(", Left: ");
  Serial.print(delta_sleft);
  Serial.print(", Angle: ");
  Serial.print(delta_theta_degrees);
  Serial.print(", X: ");
  Serial.print(x);
  Serial.print(", Y: ");
  Serial.print(y);
  Serial.print(", Delta Dist: ");
  Serial.println(delta_d);
}

void encoder_isr_right() {
  static uint8_t enc_val_right = 0;
  //pin 2
  uint8_t interrupt = (PIND & 0b100) >> 2;
  //pin 6
  uint8_t second = (PIND & 0b1000000) >> 5;
  enc_val_right = enc_val_right << 2;
  enc_val_right = enc_val_right | (interrupt | second);
  enc_count_right = lookup_table[enc_val_right & 0b1111];
  total_enc_count_right += enc_count_right;
}

void encoder_isr_left() {
  static uint8_t enc_val_left = 0;
  //pin 3
  uint8_t interrupt = (PIND & 0b1000) >> 3;
  //pin 5
  uint8_t second = (PIND & 0b100000) >> 4;
  enc_val_left = enc_val_left << 2;
  enc_val_left = enc_val_left | (interrupt | second);
  enc_count_left = lookup_table[enc_val_left & 0b1111];
  total_enc_count_left += enc_count_left;
}

void odo_close_loop() {
  float delta_v = 0;
  float v_error_last = v_error;
  float delta_v_error = 0;
  float vright = delta_sright / (actualInterval / 1000);
  float vleft = delta_sleft / (actualInterval / 1000);
  
//  Serial.print("Right Vel: ");
//  Serial.println(vright);
//  Serial.print("Left Vel: ");
//  Serial.println(vleft);

  v_error = vright - vleft;

//  if (v_error < 0) {
//    vright = vref + v_error / 2;
//    vleft = vref - v_error / 2;
//  }
//  else {
//    vright = vref - v_error / 2;
//    vleft = vref + v_error / 2;
//  }

//  vright = vref - v_error / 2;
//  vleft = vref + v_error / 2;

  delta_v_error = v_error - v_error_last;
  delta_v = -k * v_error - (b * delta_v_error);
  vright = vref - delta_v;
  vleft = vref + delta_v;
//  vright -= delta_v;
//  vleft += delta_v;
  
//  Serial.print("Right Vel: ");
//  Serial.println(vright);
//  Serial.print("Left Vel: ");
//  Serial.println(vleft);

//  On Ground
//  pwmR = long((vright + 0.0776) / 0.0016);
//  pwmL = long((vleft + 0.0907) / 0.0016);

//  In Air
  pwmR = long((vright + 0.0471) / 0.0017);
  pwmL = long((vleft + 0.0261) / 0.0016);

//  Serial.print("Error: ");
//  Serial.println(v_error);
//  Serial.print("Right PWM: ");
//  Serial.println(pwmR);
//  Serial.print("Left PWM: ");
//  Serial.println(pwmL);
}

void loop() {
  currentTime = millis();
   
  // if (y >= 1.1684 && x < 0.2032) {
  //   turn = 1;
  //   md.setM1Speed(pwmR - 20);
  //   md.setM2Speed(pwmL + 230 - 20);
  // }
  // else if (y >= 1.1684 && x >= 0.2032 && x < 0.5588) {
  //   turn = 0;
  //   md.setM1Speed(pwmR);
  //   md.setM2Speed(pwmL);
  // }
  // else if (y >= 1.1684 && x >= 0.5588) {
  //   md.setM1Speed(0);
  //   md.setM2Speed(0);
  // }
  // else if (y < 1.1684 && x < 0.2032) {
  //   md.setM1Speed(pwmR);
  //   md.setM2Speed(pwmL);
  //   if (currentTime - previousTime > interval) {
  //     location();
  //     previousTime = currentTime;
  //     last_enc_count_right = total_enc_count_right;
  //     last_enc_count_left = total_enc_count_left;
  //   }
  // }

  md.setM1Speed(pwmR);
  md.setM2Speed(pwmL);
  actualInterval = currentTime - previousTime;
  if (actualInterval >= interval) {
    previousTime = currentTime;
    location();
//    Serial.print("Delta Theta: ");
//    Serial.println(delta_theta);
//    Serial.print("Theta: ");
//    Serial.println(theta);
//    Serial.print("X: ");
//    Serial.println(x);
//    Serial.print("Y: ");
//    Serial.println(y);
  }

  // if (currentTime - previousTime > interval) {
  //   location();
  //   previousTime = currentTime;
  //   last_enc_count_right = total_enc_count_right;
  //   last_enc_count_left = total_enc_count_left;
  // }

  // if (currentTime < 30000) {
  //   md.setM1Speed(pwmR);
  //   md.setM2Speed(pwmL);
  // }
  // else {
  //   md.setM1Speed(0);
  //   md.setM2Speed(0);
  // }
//  stopIfFault();
//  delay(1);
}
