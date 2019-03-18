#include <ArduinoTcpHardware.h>
#include <ArduinoHardware.h>
#include <ros.h>

/*
 * BARBOT
 * Button Interface
 * 
 * 4 buttons as input : https://www.amazon.com/dp/B06XF6PT9L/ref=cm_sw_r_fm_api_i_lgOJCbC29MW3F
 * 10k pulldowns
 * 
 * buttow down -> connected -> HIGH
 * dutton up -> open -> LOW
 * 
 * anytime a button goes from LOW to HIGH or HIGH to LOW, output for that button turns HIGH
 * and corresponding drink is made
 */


#include <ros.h>
#include <std_msgs/String.h>

// num buttons
const int nb = 4;

// set pins
const int b_PIN[nb] = {3, 5, 7, 9};

int b_state[nb]     = {LOW, LOW, LOW, LOW};     // current button state, initialize to up -> LOW
int b_previous[nb]  = {LOW, LOW, LOW, LOW};     // previous button state, initialize to LOW
int out_state[nb]   = {LOW, LOW, LOW, LOW};     // output state, changes when button recently toggled

long time[nb]           = {0, 0, 0, 0};             // the last time the output was toggled
long debounce           = 200;                      // debounce time, increase if buggy
bool publish            = false;                     // only publish when somthing has changed

const char MARGARITA[] = "margarita";

// initialize ROS stuff
ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher buttons("drink_type_topic", &str_msg);


void setup() {
  Serial.begin(9600);

  // set all buttons to input
  for (int i = 0; i < nb; i++) {
    pinMode(b_PIN[i], INPUT);
  }

  // start node and buttons publisher
  nh.initNode();
  nh.advertise(buttons);

  // start off current and previous state to whatever buttons are currently at
  for (int i = 0; i < nb; i++) {
    b_state[i] = digitalRead(b_PIN[i]);
    b_previous[i] = b_state[i];
  }

}

void loop() {

  char drink[] = "";
  
  // read button state
  for (int i = 0; i < nb; i++) {
    b_state[i] = digitalRead(b_PIN[i]);

    //if ((b_state[i] != b_previous[i]) && (millis() - time[i] > debounce)) { // was going to add in a buffer time, took out for now
    if (b_state[i] != b_previous[i]) {
      out_state[i] = HIGH;
      publish = true; // something has changed -> go make a drink

      //time[i] = millis();
    }
  }

  // if a button was toggled, output is HIGH, go make the drink
  if (out_state[0] == HIGH) {
    drink[0] = "margarita";
  }

  if (out_state[1] == HIGH) {
    drink[3] = "zoo";
  }

  if (out_state[2] == HIGH) {
    drink[10] = "screwdriver";
  }

  if (out_state[3] == HIGH) {
    drink[15] = "tequila sunrise";
  }
  
  // for debugging
  delay(1000);
  testPrint();

  if (publish == true) {
    // publish the mixed drink we want to make
    str_msg.data = drink;
    buttons.publish(&str_msg);
  }

  // reset all output to LOW
  for (int i = 0; i < nb; i++) {
    out_state[i] = LOW;
    b_previous[i] = b_state[i];
  }
  publish = false; // reset publish

  nh.spinOnce();
}

void testPrint() {
  Serial.print("b1: ");
  Serial.print(out_state[0]);
  Serial.print(" | ");
  Serial.print("b2: ");
  Serial.print(out_state[1]);
  Serial.print(" | ");
  Serial.print("b3: ");
  Serial.print(out_state[2]);
  Serial.print(" | ");
  Serial.print("b4: ");
  Serial.println(out_state[3]);
}

