#include "DualMC33926MotorShield.h"
#include "math.h"

DualMC33926MotorShield md;

// Interrupt Counters
volatile long enc_count_right = 0;
volatile long enc_count_left = 0;
volatile long total_enc_count_right = 0;
volatile long total_enc_count_left = 0;
volatile long last_enc_count_right = 0;
volatile long last_enc_count_left = 0;

// Constants
const float pi = 3.1415926;
const float circ = .22225;
const float wheel_base = 0.1651;
const float innerRadiusRight = 0.0635;
const float outerRadiusRight = 0.2286;
const float innerRadiusLeft = 0.314325;
const float outerRadiusLeft = 0.47625;
const float turnAngleTolerance = 0.1; // 5.73 degrees
static int8_t lookup_table[] = {0,0,0,1,0,0,-1,0,0,-1,0,0,1,0,0,0};

// Initializations
float v_error = 0;
float k = -0.5;
float b = 0.05;
float delta_sleft = 0;
float delta_sright = 0;
float x = 0;
float y = 0;
float theta = 0;
float delta_theta = 0;
float delta_d = 0;
float previousTime = 0;
float currentTime = 0;
float interval = 100;
float actualInterval = 0;
float vref = 0.2;
float theta_degrees = 0;
int turn = 0;
int right = 0;

// Initial PWMs
long pwmR = long((vref + 0.0776) / 0.0016);
long pwmL = long((vref + 0.0907) / 0.0016);

// Values for Right Turn
float innerDistRight = (pi * innerRadiusRight) / 2; // In meters
float outerDistRight = (pi * outerRadiusRight) / 2; // In meters
float rightTurnTime = 1;  // In seconds
float rightTurnVelInner = innerDistRight / rightTurnTime;
float rightTurnVelOuter = outerDistRight / rightTurnTime;
float pwmRRightTurn = long((rightTurnVelInner + 0.0776) / 0.0016);
float pwmLRightTurn = long((rightTurnVelOuter + 0.0907) / 0.0016);

// Values for Left Turn
float innerDistLeft = (pi * innerRadiusLeft) / 2; // In meters
float outerDistLeft = (pi * outerRadiusLeft) / 2; // In meters
float leftTurnTime = 1;  // In seconds
float leftTurnVelInner = innerDistLeft / leftTurnTime;
float leftTurnVelOuter = outerDistLeft / leftTurnTime;
float pwmRLeftTurn = long((leftTurnVelInner + 0.0776) / 0.0016);
float pwmLLeftTurn = long((leftTurnVelOuter + 0.0907) / 0.0016);

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
  theta += delta_theta;
  theta_degrees = theta * (180 / pi);
  y += delta_d * cos(theta);
  x += delta_d * sin(theta);
  if (turn == 0) {
    odo_close_loop();
  }

  Serial.print("X: ");
  Serial.print(x);
  Serial.print(", Y: ");
  Serial.print(y);
  Serial.print(", Angle: ");
  Serial.print(theta_degrees);
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

  v_error = vright - vleft;

  delta_v_error = v_error - v_error_last;
  delta_v = -k * v_error - (b * delta_v_error);
  vright = vref - delta_v;
  vleft = vref + delta_v;

//  On Ground
  pwmR = long((vright + 0.0776) / 0.0016);
  pwmL = long((vleft + 0.0907) / 0.0016);

//  In Air
//  pwmR = long((vright + 0.0471) / 0.0017);
//  pwmL = long((vleft + 0.0261) / 0.0016);
}

/*
World Coordinate System
        Y
        ↑
        ↑
        ↑
        ↑
X←←←←←
Right turns always have shorter curvature than left turns
*/

//  Update location after a fixed interval continuously
void updateLocation() {
  actualInterval = currentTime - previousTime;
  if (actualInterval >= interval) {
    previousTime = currentTime;
    location();
  }  
}

void loop() {
  currentTime = millis();

  while(right == 1) {
  //  Start moving straight for 46 inches
    if (y < 1.1684) {
      md.setM1Speed(pwmR);
      md.setM2Speed(pwmL);
      updateLocation();
    }
  //  Start turning right after 46 inches of straight path
    else if (theta > (-pi / 2)) {
      if (turn == 0) {
        Serial.print("X: ");
        Serial.print(x);
        Serial.print(", Y: ");
        Serial.print(y);
        Serial.print(", Angle: ");
        Serial.print(theta_degrees);
        Serial.print(", Delta Dist: ");
        Serial.println(delta_d);
      }
      turn = 1;
      md.setM1Speed(pwmRRightTurn);
      md.setM2Speed(pwmLRightTurn);
      updateLocation();
    }
  //  Stop turning right after 90 degrees and start moving straight
    else if (theta < (-pi / 2) && x < 0.63) {
      if (turn == 1) {
        Serial.print("X: ");
        Serial.print(x);
        Serial.print(", Y: ");
        Serial.print(y);
        Serial.print(", Angle: ");
        Serial.print(theta_degrees);
        Serial.print(", Delta Dist: ");
        Serial.println(delta_d);
      }
      turn = 0;
      md.setM1Speed(pwmR);
      md.setM2Speed(pwmL);
      updateLocation();
    }
  //  Stop moving after 24.5 inches of straight path
    else {
      if (turn == 0) {
        Serial.print("X: ");
        Serial.print(x);
        Serial.print(", Y: ");
        Serial.print(y);
        Serial.print(", Angle: ");
        Serial.print(theta_degrees);
        Serial.print(", Delta Dist: ");
        Serial.println(delta_d);
      }
      turn = 1;
      md.setM1Speed(0);
      md.setM2Speed(0);
    }
  }

  while(right == 0) {
  //  Start moving straight for 46 inches
    if (x < 0.53975) {
      md.setM1Speed(pwmR);
      md.setM2Speed(pwmL);
      updateLocation();
    }
  //  Start turning right after 46 inches of straight path
    else if (theta < (pi / 2)) {
      if (turn == 0) {
        Serial.print("X: ");
        Serial.print(x);
        Serial.print(", Y: ");
        Serial.print(y);
        Serial.print(", Angle: ");
        Serial.print(theta_degrees);
        Serial.print(", Delta Dist: ");
        Serial.println(delta_d);
      }
      turn = 1;
      md.setM1Speed(pwmRLeftTurn);
      md.setM2Speed(pwmLLeftTurn);
      updateLocation();
    }
  //  Stop turning right after 90 degrees and start moving straight
    else if (theta > (pi / 2) && y < 1.6002) {
      if (turn == 1) {
        Serial.print("X: ");
        Serial.print(x);
        Serial.print(", Y: ");
        Serial.print(y);
        Serial.print(", Angle: ");
        Serial.print(theta_degrees);
        Serial.print(", Delta Dist: ");
        Serial.println(delta_d);
      }
      turn = 0;
      md.setM1Speed(pwmR);
      md.setM2Speed(pwmL);
      updateLocation();
    }
  //  Stop moving after 24.5 inches of straight path
    else {
      if (turn == 0) {
        Serial.print("X: ");
        Serial.print(x);
        Serial.print(", Y: ");
        Serial.print(y);
        Serial.print(", Angle: ");
        Serial.print(theta_degrees);
        Serial.print(", Delta Dist: ");
        Serial.println(delta_d);
      }
      turn = 1;
      md.setM1Speed(0);
      md.setM2Speed(0);
    }
  }

//  Motors testing code
//  if (currentTime < 30000) {
//    md.setM1Speed(pwmR);
//    md.setM2Speed(pwmL);
//  }
//  else {
//    md.setM1Speed(0);
//    md.setM2Speed(0);
//  }
//  stopIfFault();
//  delay(1);
}
