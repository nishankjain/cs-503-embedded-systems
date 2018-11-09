#include "DualMC33926MotorShield.h"

DualMC33926MotorShield md;
volatile long enc_count1 = 0;
volatile long enc_count2 = 0;

void stopIfFault()
{
 if (md.getFault())
 {
   Serial.println("fault");
   while(1);
 }
}

void setup()
{
 Serial.begin(115200);
 attachInterrupt(0,encoder_isr,CHANGE);
 attachInterrupt(1,encoder_isr2,CHANGE);
 Serial.println("Dual MC33926 Motor Shield");
 md.init();
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

  Serial.print("MR : ");
  Serial.println(enc_count1);
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

  Serial.print("ML : ");
  Serial.println(enc_count2);
}

  void loop() {
   unsigned long timeRunning = millis();
//   md.setM1Speed(150);  // Right Motor
//   md.setM2Speed(150); // Left Motor
   stopIfFault();
   delay(1);
   if (timeRunning > 15000) {
      md.setM1Speed(0);  // Right Motor
      md.setM2Speed(0); // Left Motor
   }
   else {
      md.setM1Speed(150);  // Right Motor
      md.setM2Speed(150); // Left Motor
   }
}
