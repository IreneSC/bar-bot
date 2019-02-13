// =======================================
// ========== CONSTANTS ================
// =======================================
// Default parameters for XBox, determined by feed data
#define XB_STICK_NEG -32768
#define XB_STICK_MAX 32767
#define XB_STICK_LOWFILTER 8000

#define XB_TRIGGER_MAX 255

// ============================================
// ========== SYSTEM VARIABLES ================
// ============================================
USB Usb;
XBOXRECV Xbox(&Usb);
double XB_LHatY; double XB_RHatY;                       // values for stick input
double XB_LHatX; double XB_RHatX;

double XB_L1; double XB_R1;   int XB_L1_c; int XB_R1_c; // click/press values for trigger buttons
double XB_L2; double XB_R2;   int XB_L2_c; int XB_R2_c;
double XB_A; double XB_X;     int XB_A_c; int XB_X_c;   // click/press values for letter buttons
double XB_B; double XB_Y;     int XB_B_c; int XB_Y_c;




// ====================================================
// =============== MAIN FUNCTIONS ==================
// ====== functions that are called by main INO ======
// =====================================================
// Goes into setup
void XB_setup() {
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nXbox Wireless Receiver Library Started"));

  Serial.println("XBox Startup");
}



// Goes into loop
int XB_operate() {
  // Resets all values to 0;
  XB_stop();

  // Checks Connection and Obtains Values
  Usb.Task();
  if (Xbox.XboxReceiverConnected) {

    if (Xbox.Xbox360Connected[0]) {
      //Serial.print("[c]");
      XB_obtain_signal();
      XB_process_signal();  // Norms all signals to [-1,1] for stick, [0,1] for button
    } else {
      return 0;
    }
  } else {
    Serial.println("Not connected");
    return 0;
  }
}

// Debugging: Prints XB values
void XB_print() {
  Serial.print(" XB_x: ");
  Serial.print(XB_LHatX);
  Serial.print(" XB_y: ");
  Serial.print(XB_LHatY);
  Serial.print(" XB_z: ");
  Serial.print(XB_RHatY);
  Serial.println(" || ");

}

// ====================================================
// =============== EXTERNAL FUNCTIONS ==================
// ====== functions that are called by other INOs ======
// =====================================================

double XB_getLHatY() {
  return XB_LHatY;
}
double XB_getLHatX() {
  return XB_LHatX;
}

double XB_getRHatY() {
  return XB_RHatY;
}
/*
double XB_getRHatX() {
  return XB_RHatX;
}

double XB_getR2() {
  return XB_R2;
}
double XB_getL2() {
  return XB_L2;
}
*/
double XB_getA() {
  return XB_A;
}
double XB_getY_c() {
  return XB_Y_c;
}



// ====================================================
// ===============  FUNCTIONS =========================
// =====================================================
void XB_obtain_signal() {
  // XB_R2 = Xbox.getButtonPress(R2, 0);
  // XB_L2 = Xbox.getButtonPress(L2, 0);

  XB_LHatY = Xbox.getAnalogHat(LeftHatY, 0);
  XB_LHatX = Xbox.getAnalogHat(LeftHatX, 0);
  XB_RHatY = Xbox.getAnalogHat(RightHatY, 0);
  XB_RHatX = Xbox.getAnalogHat(RightHatX, 0);

  XB_A = Xbox.getButtonPress(A, 0);
  //  XB_Y_c = Xbox.getButtonClick(Y, 0);
}

void XB_process_signal() {
  // XB_R2 = map_double(XB_R2, 0, XB_TRIGGER_MAX, 0, 1);
  // XB_L2 = map_double(XB_L2, 0, XB_TRIGGER_MAX, 0, 1);

  if (!HP_filter_stick(XB_LHatY, XB_LHatX, XB_STICK_LOWFILTER)) {
    XB_LHatY = 0;
    XB_LHatX = 0;
  }
  XB_LHatY = map_double(XB_LHatY, XB_STICK_NEG, XB_STICK_MAX, -1, 1);
  XB_LHatX = map_double(XB_LHatX, XB_STICK_NEG, XB_STICK_MAX, -1, 1);

  if (!HP_filter_stick(XB_RHatY, XB_RHatX, XB_STICK_LOWFILTER)) {
    XB_RHatY = 0;
    XB_RHatX = 0;
  }
  XB_RHatY = map_double(XB_RHatY, XB_STICK_NEG, XB_STICK_MAX, -1, 1);
  // XB_RHatX = map_double(XB_RHatX, XB_STICK_NEG, XB_STICK_MAX, -1, 1);
}

/* sets all velocities to 0 */
void XB_stop() {

  XB_LHatY = 0; // y
  XB_LHatX = 0; // x

  XB_RHatY = 0; // z
}
