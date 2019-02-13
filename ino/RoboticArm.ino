/* 
 *  ************ TO DO *************
 *  - find exact range, limit to that (instead of relatively arbitrary one rn)
 *    - should possibly just have an is valid position fxn
 *  - further limit allowable positions
 *    - have a min allowable x,y norm
 *    - make sure nothing bumps into the base
 *  - change xb op print debugger, so that it's actually printing useful things
 *  - in XB_operate, currently stops if disconnected
 *    - maybe if disconnected for more than a certain amount of time, have position reset
 *  - current set_position() might be super jittery
 *    - if so do smthing smoother than just incrementing position
 *  - change speed range in set_position after some testing
 *    - should probs time ~how long each loop takes, and base off of that
 *      - say max is maybe a2+a3 dist travelled in any axis in 2s
 *    - basically just want smooth movement
 *      - speed up loop && make set_position() slow enough that things loop smooth
 *    
 *  - change PWM range and angle range if we change servos
 *  
 *  
 *  
 *  ************* DONE *************
 *  - look at the ranges of all the trig functions vs. wat angles are actually valid for us
 *  - setup xbox
 *  - xbox motion (set_position)
 *  - servo direction of rotation
 *  - add in fxn to test print arm vals (angles, dx/dy/dz, PWMs)
 *  - in xb op maybe change current reset fxn to a stop or zero fxn since it's all velocity now
 *  - change reset(); in xbox operations
 *    - would rather have |*** (link angle zeros) as reset point
 *    - maybe also call at startup (or just have a different startup pos fxn if want smthing else)
 */

#include <Servo.h>
#include <math.h>
#include <XBOXRECV.h>
#define DEGREE (M_PI/180.0)

#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

Servo serv1;
Servo serv2;
Servo serv3;
Servo serv4;

#define NUM_SERVO 4
int     servo_pins__[NUM_SERVO]           = {2,3,4,5};
int     servo_dir[NUM_SERVO]              = {1,1,1,1}; // Servo direction of rotation 1 -> (ang++, PWM++) || 0 -> (ang++, PWM--)
Servo   servos__[NUM_SERVO]               = {serv1, serv2, serv3, serv4};
double  servo_angles__[NUM_SERVO]         = {0, 0, 0, 0};      // Angles corresponding to servos
double  servo_directions__[NUM_SERVO]     = {1, 1, 1, 1};      // Servo Direction
double  servo_gearratios__[NUM_SERVO]     = {1, 1, 1, 1};  // Potential gear ratios for servos to actuators. ratio as <Servo>/<Actuator>
double  link_angle_zero__[NUM_SERVO]      = {0,-45,45,0};      // Link angle relative to servo angle (ie link is attached at <value> relative to servo 0)
double  link_angles__[NUM_SERVO]          = {0,0,0,0};          // Angles corresponding to links
double d1 = 3;
double a2 = 3.5;
double a3 = 3.5;
double d5 = 4.15;

/* Servo range */
#define PWM_MIN 600 // Servo min/max signal
#define PWM_MAX 2200
double  ANG_MIN[NUM_SERVO]                 = {-70.0, -70.0, -70.0, -70.0}; //servos can only have 140DEG range x.x
double  ANG_MAX[NUM_SERVO]                 = {70.0, 70.0, 70.0, 70.0};

double  position_[3]                       = {0, 0, 0}; // x y z

/* ======================= SETUP ======================== */
void setup() {
  for(int i = 0; i < NUM_SERVO; i++) {
    servos__[i].attach(servo_pins__[i]);
  }
  Serial.begin(115200);
  XB_setup();
  XB_operate();
  set_position_home();
}

/* ========================== LOOP ======================= */
void loop() {
  XB_operate();
  set_position();
  position2angles();
  link2servo();
  write_angles();
  //XB_print();
  print_vals(2); // 0 - Pos | 1 - link | 2 - servo | 3 - pwm
}

/* ================= FUNCTIONS ================= */
/* 
 * Sets the position according to some method
 * Currenty using XB to control velocity
  */
int set_position() {
  int reset = XB_getA();
  if (reset) {
    set_position_home();
    return 0;
  }

  double dx = XB_getLHatY()/500.0 * (a2 + a3); // currently going for max per axis is a2+a3 dist in 1000 loops
  double dy = XB_getLHatX()/500.0 * (a2 + a3); // RELATIVELY ARBITRARY VELOCITIES RN see todo for things to improve
  double dz = XB_getRHatY()/500.0 * (a2 + a3);
  
  double tempx = position_[0]+dx;
  double tempy = position_[1]+dy;
  double tempz = position_[2]+dz;

  /* check if position is valid, if invalid, don't update*/
  tempz = constrain(tempz,0,d1+a2+a3); // cut z off at 0
  if (sqrt(sq(tempx) + sq(tempy) + sq(tempz - (d1 - d5))) > 0.95 * (a2 + a3)) { // spherical range about z = +d1
    Serial.print("Out of bounds"); 
    return 1;
  } 

  // Saves values
  position_[0] = tempx; //x
  position_[1] = tempy; //y
  position_[2] = tempz; //z

  return 0;
}
//------------------------------------------------------------

/* Calculates the link angles required to move the tool to a position/rotation */
int position2angles() {
  double thet[NUM_SERVO];
  double x = position_[0]; double y = position_[1]; double z = position_[2]; 
  double arg1; double arg2; double arg3;
  
  /* kinematic equations*/
  thet[0] = atan2(y,x); // RANGE : [-Pi,Pi]
  arg1 = (sq(x)+sq(y)+sq(z-d1+d5) - sq(a2) - sq(a3)) / (2*sq(a3));  // ensure that w/in acos [-1,1] range
  if (abs(arg1) > 1) {return 1;} // Out of range, do not update
  thet[2] = acos(arg1); // RANGE : [0,Pi]
  /*~~~~~~~~~~~~~~~~~~~~~~~~~~~Equation below may be buggy depending on link values and desired position FIX THIS~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
  arg3 = sq((a2+a3*cos(thet[2]))) + sq(a3*sin(thet[2]))- sq(x/cos(thet[0]));
  if (arg3 < 0) {arg3 = 0;}
  arg2 = (-a3*sin(thet[2]) - sqrt(arg3)) / (a2+a3*cos(thet[2])+x/cos(thet[0]));
  thet[1] = 2*atan(arg2); // RANGE : [-Pi/2,Pi/2]
  thet[3] = -(thet[1]+thet[2]); // RANGE : [-2*Pi,2*Pi]

  // Save the values
  for (int i = 0; i < NUM_SERVO; i++){
    link_angles__[i] = thet[i]/DEGREE;
  }
  return 0;
}
//------------------------------------------------------------

/* Converts link angles to servo angles */
int link2servo() {
  double temp_angles__[NUM_SERVO];

  // Converts values
  for (int i = 0; i < NUM_SERVO; i++) {
    temp_angles__[i] = servo_directions__[i]*(link_angles__[i] - link_angle_zero__[i])/servo_gearratios__[i];
  }
  
  // Checks if values within range
  for (int i = 0; i < NUM_SERVO; i++) {
    if ((temp_angles__[i]<ANG_MIN[i]) || (temp_angles__[i]>ANG_MAX[i])) {
      Serial.print("Out of Angle");
      return 1;
    }
  }
  
  // Saves values
  for (int i = 0; i < NUM_SERVO; i++) {
    servo_angles__[i] = temp_angles__[i];
  }

  return 0;
}
//------------------------------------------------------------

/* Converts a servo angle to the corresponding PWM signal */
int angle2signal(double angle, int serv_ind) {
  double signal = map_double(angle, ANG_MIN[serv_ind], ANG_MAX[serv_ind], PWM_MIN, PWM_MAX);
  return int(signal);
}
//------------------------------------------------------------

/* Actuates all servos to the desired angles */
void write_angles() {
  for (int i = 0; i < NUM_SERVO; i++) {
    servos__[i].writeMicroseconds(angle2signal(servo_angles__[i], i));
  }
}
//------------------------------------------------------------

/* Prints link, servo angles, PWM values */
void print_vals(int x) {
  if (x == 0) {
    Serial.print("Position: "); // links
    for (int i = 0; i < 3; i++){
      Serial.print(position_[i]);
      Serial.print(" | ");
    }
  } else if (x == 1) {
    Serial.print("Link Angles: "); // links
    for (int i = 0; i < NUM_SERVO; i++){
      Serial.print(link_angles__[i]);
      Serial.print(" | ");
    }
  } else if (x == 2) {
    Serial.print("Servo Angles: "); // links
    for (int i = 0; i < NUM_SERVO; i++){
      Serial.print(servo_angles__[i]);
      Serial.print(" | ");
    }
  } else if (x == 3) {
    Serial.print("Servo PWMs: "); // PWM
    for (int i = 0; i < NUM_SERVO; i++){
      Serial.print(angle2signal(servo_angles__[i], i));
      Serial.print(" | ");
    }
  }
  Serial.println(" || ");

}
//------------------------------------------------------------

/* Resets arm to link angles 1:3 at zero */
void set_position_home() {
  position_[0] = a2*sqrt(2)/2 + a3; //x
  position_[1] = 0; //y
  position_[2] = d1+a2*sqrt(2)/2-d5; //z
}
//------------------------------------------------------------
