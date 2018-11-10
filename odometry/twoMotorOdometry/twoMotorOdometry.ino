#include "DualMC33926MotorShield.h"

DualMC33926MotorShield md;
volatile long enc_count1 = 0;
volatile long enc_count2 = 0;
int pwmL = 150;
int pwmR = 150;
int error = 0;
int k = 1;
int b = 1;
unsigned long timeRunning = 0;
int turn = 0;

void setup() {
  Serial.begin(115200);
  attachInterrupt(0,encoder_isr,CHANGE);
  attachInterrupt(1,encoder_isr2,CHANGE);
  Serial.println("Dual MC33926 Motor Shield");
  md.init();
}

void stopIfFault() {
  if (md.getFault()) {
    Serial.println("fault");
    while(1);
  }
}

void encoder_isr() {
  static int8_t lookup_table[] = {0,0,0,1,0,0,-1,0,0,-1,0,0,1,0,0,0};
  static uint8_t enc_val1 = 0;
  //pin 2
  uint8_t interrupt = (PIND & 0b100) >> 2;
  //pin 6
  uint8_t second = (PIND & 0b1000000) >> 5;
  enc_val1 = enc_val1 << 2;
  enc_val1 = enc_val1 | (interrupt | second);
  enc_count1 = enc_count1 + lookup_table[enc_val1 & 0b1111];
  if (enc_count1 % 10 == 0 && turn == 0) {
    odo_close_loop();
  }
  // odo_close_loop();
  //  Serial.print("MR: ");

  //  Serial.println(enc_count1);
}

void encoder_isr2() {
  static int8_t lookup_table[] = {0,0,0,1,0,0,-1,0,0,-1,0,0,1,0,0,0};
  static uint8_t enc_val2 = 0;
  //pin 3
  uint8_t interrupt = (PIND & 0b1000) >> 3;
  //pin 5
  uint8_t second = (PIND & 0b100000) >> 4;
  enc_val2 = enc_val2 << 2;
  enc_val2 = enc_val2 | (interrupt | second);
  enc_count2 = enc_count2 + lookup_table[enc_val2 & 0b1111];
  //  if (enc_count2 % 20 == 0) {
  //   odo_close_loop();
  //  }
  //  Serial.print("ML: ");
  //  Serial.println(enc_count2);
}

void odo_close_loop() {
  // timeRunning = millis();
  // Serial.print(timeRunning);
  // if (timeRunning > 7579) {
  //   Serial.print("Time to stop");
  //   pwmR = 0;  // Right Motor
  //   pwmL = 0; // Left Motor
  // }
  // else {
    Serial.print("Turn: ");
    Serial.println(turn);
    int error_last = error;
    int delta_error = 0;
    int delta_v;
    if (enc_count1 > enc_count2) {
      // Serial.println("Right was greater");
      // Serial.print("Right Count: ");
      // Serial.println(enc_count1);
      // Serial.print("Left Count: ");
      // Serial.println(enc_count2);
      error = enc_count1 - enc_count2;
      delta_error = error - error_last;
      delta_v = -k*error - b*delta_error;
      pwmR += delta_v;
      pwmL -= delta_v;
    }
    else if (enc_count2 > enc_count1) {
      // Serial.println("Left was greater");
      // Serial.print("Right Count: ");
      // Serial.println(enc_count1);
      // Serial.print("Left Count: ");
      // Serial.println(enc_count2);
      error = enc_count2 - enc_count1;
      delta_error = error - error_last;
      delta_v = -k*error - b*delta_error;
      pwmR -= delta_v;
      pwmL += delta_v;
    }

    // Serial.print("Error: ");
    // Serial.println(delta_error);
    Serial.print("Right PWM: ");
    Serial.println(pwmR);
    Serial.print("Left PWM: ");
    Serial.println(pwmL);
  // }

}

void loop() {
  unsigned long timeRunning = millis();
  if (timeRunning > 12719) {
    turn = 0;
    md.setM1Speed(0);  // Right Motor
    md.setM2Speed(0);  // Left Motor
  }
  else if (timeRunning > 8929) {
    if (turn == 1) {
      enc_count1 = 0;
      enc_count2 = 0;
    }
    turn = 0;
    md.setM1Speed(pwmR);
    md.setM2Speed(pwmL);
  }
  else if (timeRunning > 7579) {
    turn = 1;
    md.setM1Speed(pwmR);
    md.setM2Speed(pwmL + 260);
  }
  else {
    turn = 0;
    md.setM1Speed(pwmR);
    md.setM2Speed(pwmL);
  }
   stopIfFault();
   delay(1);
}
