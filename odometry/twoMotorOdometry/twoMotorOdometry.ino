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
const float innerRadiusLeft = 0.36;  //13.25  = 0.33655
const float outerRadiusLeft = 0.65;   //19.625 0.498475;
const float turnAngleTolerance = 0.1; // 5.73 degrees
static int8_t lookup_table[] = {0,0,0,1,0,0,-1,0,0,-1,0,0,1,0,0,0};

// Initializations
float v_error = 0;
float k = 0.004; // 4; //.0001 good//005;  //-.4  /0.004 good for vref = 0.2
float b = 0.001;//0.00015;//0.05;          / 0.001 good
float delta_sleft = 0;
float delta_sright = 0;
float total_delta_sright = 0;
float total_delta_sleft = 0;
float x = 0;
float y = 0;
float theta = 0; //pi/2 //0; //0;
float delta_theta = 0;
float delta_d = 0;
float previousTime = 0;
float currentTime = 0;
float interval = 50;
float errorCorrectInterval = 50;
float previousCorrTime = 0;
float actualInterval = 0;
float vref = 0.2;//0.2;
float theta_degrees = 0;
int turn = 0;  //3
float cur_x = 0;
float cur_y = 0;
float cur_theta = 0;

//int right = 0;
const int pingPin = 11;

int right = 1;
int startTurning = 0;
int hello = 1;
int q[50] ; //= new q[50]; 
int qCount = 0;


// Initial PWMs
long pwmR = long((vref + 0.0776) / 0.0016);
long pwmL = long((vref + 0.0907) / 0.0016);


// Values for Right Turn
float innerDistRight = (pi * innerRadiusRight) / 2; // In meters
float outerDistRight = (pi * outerRadiusRight) / 2; // In meters
float rightTurnTime = 1.5;  // In seconds
float rightTurnVelInner = innerDistRight / rightTurnTime;
float rightTurnVelOuter = outerDistRight / rightTurnTime;
float pwmRRightTurn = long((rightTurnVelInner + 0.0776) / 0.0016);
float pwmLRightTurn = long((rightTurnVelOuter + 0.0907) / 0.0016);

// Values for Left Turn
float innerDistLeft = (pi * innerRadiusLeft) / 2; // In meters
float outerDistLeft = (pi * outerRadiusLeft) / 2; // In meters
float leftTurnTime = 2;  // In seconds
float leftTurnVelInner = innerDistLeft / leftTurnTime;
float leftTurnVelOuter = outerDistLeft / leftTurnTime;
float pwmRLeftTurn = long((leftTurnVelOuter + 0.0776) / 0.0016) + long(35);
float pwmLLeftTurn = long((leftTurnVelInner + 0.0907) / 0.0016) + long(35);

//flags
boolean start_turn  = false;
boolean done_turn  = false;
int turnTowards = 0;
boolean S = true;
boolean R = true;
boolean L = true;


void setup() {
  Serial.begin(115200);
  //right = Serial.read();
  //theta = pi/2 * Serial.read();
  //turnTowards = Serial.read();
  //Serial.println(turnTowards);
  //theta = pi/2 * Serial.read();
  attachInterrupt(0, encoder_isr_right, CHANGE);
  attachInterrupt(1, encoder_isr_left, CHANGE);
  Serial.println("Dual MC33926 Motor Shield");
  md.init();
  //readSM();
  qCount = 0;
  turn = q[0];
}

void encoder_isr_right() {
  static uint8_t enc_val_right = 0;
  //pin 2
  uint8_t interrupt = (PIND & 0b100) >> 2;
  //pin 6
  uint8_t second = (PIND & 0b1000000) >> 5;
  enc_val_right = enc_val_right << 2;
  enc_val_right = enc_val_right | (interrupt | second);
  enc_count_right += lookup_table[enc_val_right & 0b1111];
  total_enc_count_right += lookup_table[enc_val_right & 0b1111];
}

void encoder_isr_left() {
  static uint8_t enc_val_left = 0;
  //pin 3
  uint8_t interrupt = (PIND & 0b1000) >> 3;
  //pin 5
  uint8_t second = (PIND & 0b100000) >> 4;
  enc_val_left = enc_val_left << 2;
  enc_val_left = enc_val_left | (interrupt | second);
  enc_count_left += lookup_table[enc_val_left & 0b1111];
  total_enc_count_left += lookup_table[enc_val_left & 0b1111];
}

void stopIfFault() {
  if (md.getFault()) {
    Serial.println("fault");
    while(1);
  }
}

void readSM(){
  for (qCount;qCount <50;qCount++){
  if (Serial.available() > 0) {
  q[qCount] = Serial.parseInt();
  //Serial.println(turnTowards);
  }
  else{
   qCount--;
  }
  }
}
void correctRotation() {
  if (Serial.available() > 0) {
  turnTowards = Serial.parseInt();
  //Serial.println(turnTowards);
  }
  else{
   turnTowards = 0;
  } 
  /*if (turnDirection == 1) {
    pwmR = -75;
    pwmL = 75
  }
  else if (turnDirection == 2) {
    pwmR = 75;
    pwmL = -75;
  }
  else {
    startTurning = 0;
  }
  */
}


/*void location() {
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
  if (turn == 0 && currentTime - previousCorrTime > errorCorrectInterval) {
    odo_close_loop();
    previousCorrTime = currentTime;
  }


}
*/



void location() {
  delta_sright = float(total_enc_count_right - last_enc_count_right) * circ / 32;
  delta_sleft = float(total_enc_count_left - last_enc_count_left) * circ / 32;
  last_enc_count_right = total_enc_count_right;
  last_enc_count_left = total_enc_count_left;
  delta_d = (delta_sleft + delta_sright) / 2;
  delta_theta = atan2((delta_sright - delta_sleft) / 2, wheel_base / 2);
  /*
  if(!start_turn){
    if(right==1){
      theta = 0;
    }
    else{
      theta = pi/2;
    }
  }
  else{
    if(!done_turn){
      theta +=delta_theta;
    }
    else{
      if(right==1){
        theta = -pi/2;
      }
      else{
        theta = pi;
      }
    }
  }
  */
  theta += delta_theta;
  theta_degrees = theta * (180 / pi);
  y += delta_d * cos(theta);
  x += delta_d * sin(theta);
  total_delta_sright += delta_sright;
  total_delta_sleft += delta_sleft;
  if (currentTime - previousCorrTime > errorCorrectInterval) {    //turn!=1
    error_correction();
    previousCorrTime = currentTime;
    total_delta_sright = 0;
    total_delta_sleft = 0;
  }
}


/*
void odo_close_loop() {
  float delta_v = 0;
  float v_error_last = v_error;
  float delta_v_error = 0;
  float vright = enc_count_right *circ/32 / (errorCorrectInterval /1000); //total_delta_sright / (errorCorrectInterval / 1000);
  float vleft = enc_count_left *circ/32 / (errorCorrectInterval /1000);  //total_delta_sleft / (errorCorrectInterval / 1000);
  
  Serial.println(enc_count_right);
  Serial.println(enc_count_left);
  v_error = vright - vleft;
  Serial.println(v_error);

  delta_v_error = v_error - v_error_last;
  delta_v = -k * v_error - (b * delta_v_error);
  vright = vref - delta_v;
  vleft = vref + delta_v;

  //  On Ground
  pwmR = long((vright + 0.0776) / 0.0016);
  pwmL = long((vleft + 0.0907) / 0.0016);
  //pwmL = long((vleft + 0.0776) / 0.0016);
  
  Serial.println("PWM R, L");
  Serial.println(pwmR);
  Serial.println(pwmL);
  
  
  //0.0907
  //  In Air
  //  pwmR = long((vright + 0.0471) / 0.0017);
  //  pwmL = long((vleft + 0.0261) / 0.0016);
}
*/
void error_correction(){
  //if (turnTowards > 10 ){
  float delta_v = 0;
  float v_error_last = v_error;
  float delta_v_error = 0;
  v_error = turnTowards;// / 200;
  //Serial.println(turnTowards);
  //Serial.println(v_error);

  delta_v_error = v_error - v_error_last;
  delta_v = -k * v_error - (b * delta_v_error);
  float vright = vref - delta_v;
  float vleft = vref + delta_v;

  //  On Ground
  //pwmR += turnTowards; 
  //pwmL -= turnTowards;
  pwmR = long((vright + 0.0776) / 0.0016);
  pwmL = long((vleft + 0.0907) / 0.0016);
    
  //}
 /* else if (turnTowards < -10 ){
  float delta_v = 0;
  float v_error_last = v_error;
  float delta_v_error = 0;
  v_error = -turnTowards / 200;
 // Serial.println(v_error);

  delta_v_error = v_error - v_error_last;
  delta_v = -k * v_error - (b * delta_v_error);
  float vright = vref - delta_v;
  float vleft = vref + delta_v;

  //  On Ground
  //pwmR += turnTowards; 
  //pwmL -= turnTowards;
  pwmR = long((vright + 0.0776) / 0.0016);
  pwmL = long((vleft + 0.0907) / 0.0016);
    
  }*/
  
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
  if (currentTime - previousTime >= interval) {
    previousTime = currentTime;
    location();
  }
}
void ping(){
  //if (int(currentTime) % 50 == 0){
  long duration, inches;



  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.

  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:

  pinMode(pingPin, OUTPUT);

  digitalWrite(pingPin, LOW);

  delayMicroseconds(2);


  digitalWrite(pingPin, HIGH);

  delayMicroseconds(5);

  digitalWrite(pingPin, LOW);



  // The same pin is used to read the signal from the PING))): a HIGH pulse

  // whose duration is the time (in microseconds) from the sending of the ping

  // to the reception of its echo off of an object.

  pinMode(pingPin, INPUT);

  duration = pulseIn(pingPin, HIGH, 5000);
  // convert the time into a distance
  //Serial.println("--------------");
  //Serial.println(duration);
  //Serial.println("--------------");
  if(duration!=0 ){
    inches = duration / 74 / 2;
  }
  else{
   inches = 100; 
  }
 
  if(inches < 6){
    vref = 0.1;
    if(inches < 3){
     vref = 0; 
    }
  } 
  else{
   vref = 0.2; 
  }
//}
  
}
void rightTurn (){
 // if (R) {
    // cur_theta = theta;
   //  cur_x = x;
    // done_turn = false;
   //  start_turn = true;
   //  }
   //  R = false;
     done_turn = false;
     start_turn = true;
     cur_theta = theta;
     cur_x = x;
     cur_y = y;
     while (abs(theta - cur_theta) < pi/2 - pi/90 ) {//&& abs(x-cur_x) < 0.35 && abs(y-cur_y) < 0.35 ) {
        //Serial.println(theta - cur_theta);
        //turn = 1;
        md.setM1Speed(pwmRRightTurn);
        md.setM2Speed(pwmLRightTurn);
        currentTime = millis();
        //correctRotation();
        updateLocation();
      }
    // turn = 0;
     done_turn = true;
     start_turn = false;
     cur_x = x;
     cur_y = y;
     while(abs(x - cur_x) < 0.25 && abs(y - cur_y) < 0.25){        //adjust values to make work
       md.setM1Speed(pwmR);
       md.setM2Speed(pwmL);
      // Serial.println(total_enc_count_right);
     //  Serial.println(total_enc_count_left);
       currentTime = millis();
      correctRotation();
       updateLocation(); //look for bright not green as primary filter
      }
     //md.setM1Speed(0);
     //md.setM2Speed(0);
     turn = q[++qCount];
    // turn = 4;
      
}

void leftTurn () {
     cur_x = x;
     cur_y = y;
     while(abs(x - cur_x) < 0.06 && abs(y - cur_y) < 0.06){        //adjust values to make work
       md.setM1Speed(pwmR);
       md.setM2Speed(pwmL);
      // Serial.println(total_enc_count_right);
     //  Serial.println(total_enc_count_left);
      currentTime = millis();
      correctRotation();
      //updateLocation();
      updateLocation(); //look for bright not green as primary filter
      }
      
     //done_turn = false;
     //start_turn = true;
     cur_theta = theta;
     cur_x = x;
     cur_y = y;
     while (abs(theta - cur_theta) < (pi/2 + pi/180) ) {//&& abs(x-cur_x) < 0.35 && abs(y-cur_y) < 0.35 ) {
        //Serial.println(theta - cur_theta);
        //turn = 1;
        md.setM1Speed(pwmRLeftTurn);
        md.setM2Speed(pwmLLeftTurn);
        currentTime = millis();
        //correctRotation();
        //updateLocation();
        updateLocation();
      }
    // turn = 0;
     //done_turn = true;
     //start_turn = false;
     cur_x = 0;
     cur_y = 0;
     /*
     while(abs(x - cur_x) < 0.5 && abs(y - cur_y) < 0.5){        //adjust values to make work
       md.setM1Speed(pwmR);
       md.setM2Speed(pwmL);
      // Serial.println(total_enc_count_right);
     //  Serial.println(total_enc_count_left);
       currentTime = millis();
      correctRotation();
      //updateLocation();
       updateLocation(); //look for bright not green as primary filter
      }
     md.setM1Speed(0);
     md.setM2Speed(0);
     turn = q[++qCount];
      */
} 

void straight() {
      //if(S){
      //cur_x = x;
      //cur_y = y;
      //}
      //S = false;
      cur_x = x;
      cur_y = y;
      while(true){ //abs(x - cur_x) < 5 && abs(y - cur_y) < 5){ //0.68){ //0.6){        //adjust values to make work  //true
       md.setM1Speed(pwmR);
       md.setM2Speed(pwmL);
      // Serial.println(total_enc_count_right);
     //  Serial.println(total_enc_count_left);
       currentTime = millis();
       correctRotation();
       ping();
       updateLocation(); //look for bright not green as primary filter
       
       if (turnTowards == 350){
         Serial.println("mde it");
        turn = 3;
         break; 
       }
       else if (turnTowards == 400){
        turn = q[++qCount];
        break; 
       }
       
       
      }
         //S = true;
        // md.setM1Speed(0);
       //  md.setM2Speed(0);
         
       //  Serial.print(turn);
        // turn = 1;
        // q[qCount] = 3;
     // cur_x = 0;
     // cur_y = 0;
      
}

void blindS(){
 
      cur_x = x;
      cur_y = y;
  while(abs(x - cur_x) < 0.55 && abs(y - cur_y) < 0.55){ //0.6){        //adjust values to make work
       pwmR = long((vref + 0.0776) / 0.0016) + long(16);
       pwmL = long((vref + 0.0907) / 0.0016);
       md.setM1Speed(pwmR);
       md.setM2Speed(pwmL);
      // Serial.println(total_enc_count_right);
     //  Serial.println(total_enc_count_left);
       currentTime = millis();
       //correctRotation();
       updateLocation(); //look for bright not green as primary filter
       
      }
      Serial.print("here");
      cur_x = 0;
      cur_y = 0;
}

void loop() {
  //currentTime = millis();
  
 // if (turnTowards !=0){
 // qCount++;
 // Serial.println(qCount);
  
 // turn = q[qCount];
 // Serial.println(turn);
 // if(qCount == 10){
 //   qCount = 0; 
 // }
 // }
  
  //turn = q[qCount];
  //turn = 0;
  if(turn == 0){
    correctRotation();
    if (turnTowards!=0){
       //Serial.println("started moving");
       straight();
    } 
  }
 else if (turn == 1 && turnTowards!=0) {
   leftTurn();
   straight();
   //Turnstraight();
 }
 else if (turn == 2 && turnTowards!=0){
  blindS();
  straight(); 
 }
 else if (turn == 3){
   while(turnTowards < 400){//create new array
     md.setM1Speed(0);
     md.setM2Speed(0);
     correctRotation();
   }
   //turn = q[++qCount];
 }
 
  
/*  
///if (int(currentTime) % 50 == 0){
  long duration, inches;



  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.

  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:

  pinMode(pingPin, OUTPUT);

  digitalWrite(pingPin, LOW);

 // delayMicroseconds(2);


  digitalWrite(pingPin, HIGH);

 // delayMicroseconds(5);

  digitalWrite(pingPin, LOW);



  // The same pin is used to read the signal from the PING))): a HIGH pulse

  // whose duration is the time (in microseconds) from the sending of the ping

  // to the reception of its echo off of an object.

  pinMode(pingPin, INPUT);

  duration = pulseIn(pingPin, HIGH);
  // convert the time into a distance

  inches = duration / 74 / 2;
 
  if(inches < 6){
    vref = vref/2;
  } 
  else{
   vref = 0.2; 
  }
///}
*/
//if (right == 1) {

  //while(startTurning) {
   // correctRotation();
 // }

//  if (int(currentTime) % 1000 == 0) {
//    correctRotation();
//  }
/*
  if (right == 1 && startTurning == 0) {
    //  Start moving straight for 46 inches
      if (y < 1.14) {
        md.setM1Speed(pwmR);
        md.setM2Speed(pwmL);
        updateLocation();
      }
    //  Start turning right after 46 inches of straight path
      else if (theta > ((-pi / 2)) && x > -0.35) {
        if (turn == 0) {
          start_turn = true;
        }
        turn = 1;
        md.setM1Speed(pwmRRightTurn);
        md.setM2Speed(pwmLRightTurn);
        updateLocation();
      }
    //  Stop turning right after 90 degrees and start moving straight
      else if (x > -0.64) {  //theta < (-pi / 2) &&
        if (turn == 1) {
          done_turn = true;
          enc_count_right = 0;
          enc_count_left = 0;
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
  else if (right == 0 && startTurning == 0) {
  //  Start moving straight for 46 inches
    if (x < 0.53975) {
      md.setM1Speed(pwmR);
      md.setM2Speed(pwmL);
      updateLocation();
    }
  //  Start turning left after 22 inches of straight path
    else if (theta < pi && y > -0.5) {
      if (turn == 0) {
        start_turn = true;
      }
      turn = 1;
      md.setM1Speed(pwmRLeftTurn);
      md.setM2Speed(pwmLLeftTurn);
      updateLocation();
    }
  //  Stop turning right after 90 degrees and start moving straight
    else if (y > -1.50) {
      if (turn == 1) {
       done_turn = true;
       enc_count_right = 0;
       enc_count_left = 0;
      }
      turn = 0;
      md.setM1Speed(pwmR);
      md.setM2Speed(pwmL);
      updateLocation();
    }
  //  Stop moving after 24.5 inches of straight path
    else {
      if (turn == 0) {
      }
      turn = 1;
      md.setM1Speed(0);
      md.setM2Speed(0);
    }
  }

*/
}
