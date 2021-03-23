#include <Wire.h>
#include <LIDARLite.h>

#include "Arduino.h"
#include "heltec.h"
#include "scanbot_images.h"
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

LIDARLite lidar;

BluetoothSerial SerialBT;

#define MAX_STACK_STEPS 801

int button1Pin = 18;
int button2Pin = 5;
int button3Pin = 17;

int button1OldState = 0;
int button2OldState = 0;
int button3OldState = 0;
int button1NewState;
int button2NewState;
int button3NewState;

// polling time for buttons
int dt = 50;

int menuState = 1; // begin at main menu screen
int tiltOffset = 0; // manual tilt adjustment
int baseOffset = 0; // manual base adjustment
short int scanResolution = 1;
short int oldScanResolution = scanResolution;
short int scanPrecision = 3;

short int tiltStartRange = 0;
short int tiltEndRange = 180;
short int baseStartRange = 0;
short int baseEndRange = 360;

const int baudRate = 500000;

// setting the total steps according to microstep setting
int BASE_STEPS = 1600;
int ARM_STEPS = 1600;
int POINT_STACK_STEPS = ARM_STEPS / 2 + 1; // adding 1 because we want the very bottom point as well
int TOTAL_POINTS = BASE_STEPS * POINT_STACK_STEPS;

// setting up variables for spherical coordinates
float thetaPerStep = (2.0 * PI) / ARM_STEPS;
float phiPerStep = (2.0 * PI) / BASE_STEPS;

float phi = 0.0; // we are at 0 phi base rotation at scan start
float theta = 0.0; // we are PI or 180 degrees down from 0 in theta at scan start

const int lidarOffset = 7; // the lidar is roughly 7cm away from the origin

// main array for storing x, y, z coordinates
float pointStackCartesian[MAX_STACK_STEPS][3];

// PIN SETUP
const int piezoPin = 2;
const int relayPin = 12;

const int driverDIR1 = 26; // base motor DIR
const int driverPUL1 = 25; // base motor PUL
const int driverDIR2 = 27; // arm motor DIR
const int driverPUL2 = 14; // arm motor PUL
// END PIN SETUP

// where we are in the rotation
int currentBaseStep = 0;
int currentArmStep = 0;

// interrupt flag for when interrupt button is pressed
volatile int interrupt = false;

// all notes from C4 to B4 as frequencies
float notes[12] = {261.63, 277.18, 293.66, 311.13, 329.63, 349.23, 369.99, 392.00, 415.30, 440.00, 466.16, 493.88};

const int armStepDelay = 125; // 125 worked great with 1600 steps
const int baseStepDelay = armStepDelay * 2; // this ratio seems to work good

bool started = false; // main scan loop start

double d; // float for storing distance from radar

long int timeAtScanStart;

portMUX_TYPE synch = portMUX_INITIALIZER_UNLOCKED; // not sure what this is but guy in interrupt tutorial said to do this

// recalculate step variables
void recalculateSteps() {
  BASE_STEPS = 1600 / scanResolution;
  ARM_STEPS = 1600 / scanResolution;
  POINT_STACK_STEPS = ARM_STEPS / 2 + 1; // adding 1 because we want the very bottom point as well
  TOTAL_POINTS = BASE_STEPS * POINT_STACK_STEPS;

  thetaPerStep = (2.0 * PI) / ARM_STEPS;
  phiPerStep = (2.0 * PI) / BASE_STEPS;
}

// this function gets called when the interrupt switch is toggled
void IRAM_ATTR isr() {
  portENTER_CRITICAL(&synch);
  digitalWrite(relayPin, LOW);
  interrupt = true;
  portEXIT_CRITICAL(&synch);
}

void setup() {
  pinMode(driverPUL1, OUTPUT);
  pinMode(driverDIR1, OUTPUT);
  pinMode(driverPUL2, OUTPUT);
  pinMode(driverDIR2, OUTPUT);
  pinMode(relayPin, OUTPUT);
  pinMode(button1Pin, INPUT_PULLDOWN);
  pinMode(button2Pin, INPUT_PULLDOWN);
  pinMode(button3Pin, INPUT_PULLDOWN);

  Heltec.begin(true /*DisplayEnable Enable*/, false /*LoRa Disable*/, false /*Serial Enable*/);
  Heltec.display->setContrast(255);
  Heltec.display->clear();

  lidar.begin(0, true); // Set configuration to default and I2C to 400 kHz
  lidar.configure(3); // Change this number to try out alternate configuration

  //      configuration:  Default 0.
  //      0: Default mode, balanced performance.
  //      1: Short range, high speed. Uses 0x1d maximum acquisition count.
  //      2: Default range, higher speed short range. Turns on quick termination
  //          detection for faster measurements at short range (with decreased
  //          accuracy)
  //      3: Maximum range. Uses 0xff maximum acquisition count.
  //      4: High sensitivity detection. Overrides default valid measurement detection
  //          algorithm, and uses a threshold value for high sensitivity and noise.
  //      5: Low sensitivity detection. Overrides default valid measurement detection
  //          algorithm, and uses a threshold value for low sensitivity and noise.
  //    lidarliteAddress: Default 0x62. Fill in new address here if changed. See
  //      operating manual for instructions.
  //  lidar.write(0x02, 0xff); // Set acquisition count to maximum
  //  lidar.write(0x04, 0b00000100); // Use non-default reference acquisition count
  //  lidar.write(0x12, 0x05); // Reference acquisition count of 5 (default is 5)

  // Serial.begin(baudRate);
  SerialBT.begin("SCANBOT");

  playMelody(piezoPin, 5, 100); // startup sound
}

void toggleStarted() {
  if (started) {
    started = false;
  } else {
    started  = true;
  }
}

// following 2 functions allow us to skip multiple steps for lower scan resolution
void moveArm(int n = 0) {
  if (n > 0)
    digitalWrite(driverDIR2, HIGH);
  else if (n < 0)
    digitalWrite(driverDIR2, LOW);
  for (int i = 0; i < abs(n * scanResolution); i++) {
    digitalWrite(driverPUL2, LOW);
    delayMicroseconds(armStepDelay);
    digitalWrite(driverPUL2, HIGH);
    delayMicroseconds(armStepDelay);
  }
  currentArmStep += n;
  theta += thetaPerStep * n / abs(n);
}

void moveBase(int n = 0) {
  if (n > 0)
    digitalWrite(driverDIR1, LOW);
  else if (n < 0)
    digitalWrite(driverDIR1, HIGH);
  for (int i = 0; i < abs(n * scanResolution); i++) {
    digitalWrite(driverPUL1, LOW);
    delayMicroseconds(baseStepDelay);
    digitalWrite(driverPUL1, HIGH);
    delayMicroseconds(baseStepDelay);
  }
  currentBaseStep += n;
  phi += phiPerStep * n / abs(n);
}

// reset the base back to zero, accurate as we've been keeping track of positions
void resetToZero() {

  // return arm to horizontal 90 deg position
  moveArm(ARM_STEPS / 4 - currentArmStep);

  delay(250);

  // return base
  moveBase(-currentBaseStep);

  delay(250);

  currentBaseStep = 0; // reset base position tracker
  currentArmStep = ARM_STEPS / 4; // reset arm position tracker to horizontal position
  digitalWrite(relayPin, LOW);

  detachInterrupt(digitalPinToInterrupt(button3Pin));
  menuState = 1;
  started = false; // exit main loop and go back to main menu

}

void changeResolution() {
  if (scanResolution == 1) {
    scanResolution = 2;
    recalculateSteps();
  } else if (scanResolution == 2) {
    scanResolution = 4;
    recalculateSteps();
  } else if (scanResolution == 4) {
    scanResolution = 8;
    recalculateSteps();
  } else {
    scanResolution = 1;
    recalculateSteps();
  }
}

void changePrecision() {
  if (scanPrecision == 3) {
    scanPrecision = 0;
    lidar.configure(scanPrecision);
  } else if (scanPrecision == 0) {
    scanPrecision = 1;
    lidar.configure(scanPrecision);
  } else {
    scanPrecision = 3;
    lidar.configure(scanPrecision);
  }
}

void buttonPress(int btn) {
  switch (menuState) {
    case 1: // main menu screen
      switch (btn) {
        case 1:
          menuState = 2;
          digitalWrite(relayPin, HIGH);
          oldScanResolution = scanResolution;
          scanResolution = 1;
          recalculateSteps();
          break;
        case 2:
          //start the scan
          attachInterrupt(digitalPinToInterrupt(button3Pin), isr, CHANGE); // interrupt button
          menuState = 10; // go to scan screen
          started = true; // scan loop starter boolean
          break;
        case 3:
          menuState = 5;
          break;
        default:
          break;
      }
      break;
    case 2: // offset screen
      switch (btn) {
        case 1:
          menuState = 3; // tilt adjust screen
          break;
        case 2:
          menuState = 4; // base adjust screen
          break;
        case 3: // exit offset menu
          menuState = 1;
          scanResolution = oldScanResolution;
          recalculateSteps(); // put things back in their place
          currentBaseStep = 0;
          currentArmStep = 0;
          break;
        default:
          break;
      }
      break;
    case 3: // tilt adjust menu
      switch (btn) {
        case 1:
          moveArm(-1);
          tiltOffset++;
          break;
        case 2:
          moveArm(1);
          tiltOffset--;
          break;
        case 3:
          menuState = 2;
          break;
        default:
          break;
      }
      break;
    case 4: // base adjust menu
      switch (btn) {
        case 1:
          moveBase(-1);
          baseOffset++;
          break;
        case 2:
          moveBase(1);
          baseOffset--;
          break;
        case 3:
          menuState = 2;
          break;
        default:
          break;
      }
      break;
    case 5: // setup selection menu
      switch (btn) {
        case 1:
          menuState = 6;
          break;
        case 2:
          menuState = 7;
          break;
        case 3:
          menuState = 1;
          break;
        default:
          break;
      }
      break;
    case 6: // mode and resolution selection screen
      switch (btn) {
        case 1:
          // change precision
          changePrecision();
          break;
        case 2:
          // change resolution
          changeResolution();
          break;
        case 3:
          menuState = 5;
          break;
        default:
          break;
      }
      break;
    case 7: // range selection screen
      switch (btn) {
        case 1:
          menuState = 8;
          break;
        case 2:
          menuState = 9;
          break;
        case 3:
          menuState = 5;
          break;
        default:
          break;
      }
      break;
    case 8: // range tilt
      switch (btn) {
        case 1:
          changeRangeTiltStart();
          break;
        case 2:
          changeRangeTiltEnd();
          break;
        case 3:
          menuState = 7;
          break;
        default:
          break;
      }
      break;
    case 9: // range base
      switch (btn) {
        case 1:
          changeRangeBaseStart();
          break;
        case 2:
          changeRangeBaseEnd();
          break;
        case 3:
          menuState = 7;
          break;
        default:
          break;
      }
      break;
    case 10: // scan screeen
      switch (btn) {
        case 1:
          // pause
          //toggleStarted(); this dont work for some reason
          break;
        case 2:
          // reset
          resetToZero();
          menuState = 1;
          break;
        case 3:
          // abort
          // menuState = 1;
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
  // play button press sound
  playMelody(piezoPin, 4, 100);
}

void changeRangeTiltStart() {
  if (tiltEndRange - tiltStartRange - 45 > 0 && tiltStartRange != 135) {
    tiltStartRange += 45;
  } else {
    tiltStartRange = 0;
  }
}

void changeRangeTiltEnd() {
  if (tiltStartRange != 135) {
    if (tiltEndRange != 180) {
      tiltEndRange += 45;
    } else {
      tiltEndRange = tiltStartRange + 45;
    }
  }
}

void changeRangeBaseStart() {
  if (baseEndRange - baseStartRange - 90 > 0 && baseStartRange != 270) {
    baseStartRange += 90;
  } else {
    baseStartRange = 0;
  }
}

void changeRangeBaseEnd() {
  if (baseStartRange != 270) {
    if (baseEndRange != 360) {
      baseEndRange += 90;
    } else {
      baseEndRange = baseStartRange + 90;
    }
  }
}

void drawScreen() {
  Heltec.display->clear();
  switch (menuState) {
    case 1: // main menu
      Heltec.display->drawXbm(0, 0, 128, 64, menu1);
      break;
    case 2: // offset menu
      Heltec.display->drawXbm(0, 0, 128, 64, menu2);
      Heltec.display->setTextAlignment(TEXT_ALIGN_CENTER);
      Heltec.display->setFont(ArialMT_Plain_16);
      Heltec.display->drawString(35, 0, "TILT");
      Heltec.display->drawString(35, 20, String(tiltOffset));
      Heltec.display->drawString(93, 0, "BASE");
      Heltec.display->drawString(93, 20, String(baseOffset));
      break;
    case 3: // tilt offset
      Heltec.display->drawXbm(0, 0, 128, 64, menu3);
      Heltec.display->setTextAlignment(TEXT_ALIGN_CENTER);
      Heltec.display->setFont(ArialMT_Plain_16);
      Heltec.display->drawString(35, 0, "TILT");
      Heltec.display->drawString(35, 20, String(tiltOffset));
      Heltec.display->drawString(93, 0, "BASE");
      Heltec.display->drawString(93, 20, String(baseOffset));
      break;
    case 4: // base offset
      Heltec.display->drawXbm(0, 0, 128, 64, menu4);
      Heltec.display->setTextAlignment(TEXT_ALIGN_CENTER);
      Heltec.display->setFont(ArialMT_Plain_16);
      Heltec.display->drawString(35, 0, "TILT");
      Heltec.display->drawString(35, 20, String(tiltOffset));
      Heltec.display->drawString(93, 0, "BASE");
      Heltec.display->drawString(93, 20, String(baseOffset));
      break;
    case 5: // setup screen
      Heltec.display->drawXbm(0, 0, 128, 64, menu5);
      break;
    case 6: // setup screen
      Heltec.display->drawXbm(0, 0, 128, 64, menu6);
      Heltec.display->setFont(ArialMT_Plain_10);
      Heltec.display->setTextAlignment(TEXT_ALIGN_CENTER);
      Heltec.display->drawString(64, 0, "Resolution:");
      Heltec.display->drawString(64, 19, "Precision mode:");
      if (scanResolution == 1) {
        Heltec.display->drawString(17, 9, "[1600]");
        Heltec.display->drawString(50, 9, "800");
        Heltec.display->drawString(82, 9, "400");
        Heltec.display->drawString(112, 9, "200");
      } else if (scanResolution == 2) {
        Heltec.display->drawString(17, 9, "1600");
        Heltec.display->drawString(50, 9, "[800]");
        Heltec.display->drawString(82, 9, "400");
        Heltec.display->drawString(112, 9, "200");
      } else if (scanResolution == 4) {
        Heltec.display->drawString(17, 9, "1600");
        Heltec.display->drawString(50, 9, "800");
        Heltec.display->drawString(82, 9, "[400]");
        Heltec.display->drawString(112, 9, "200");
      } else if (scanResolution == 8) {
        Heltec.display->drawString(17, 9, "1600");
        Heltec.display->drawString(50, 9, "800");
        Heltec.display->drawString(82, 9, "400");
        Heltec.display->drawString(112, 9, "[200]");
      }
      if (scanPrecision == 3) {
        Heltec.display->drawString(24, 29, "[MAX]");
        Heltec.display->drawString(64, 29, "HIGH");
        Heltec.display->drawString(104, 29, "LOW");
      } else if (scanPrecision == 0) {
        Heltec.display->drawString(24, 29, "MAX");
        Heltec.display->drawString(64, 29, "[HIGH]");
        Heltec.display->drawString(104, 29, "LOW");
      } else if (scanPrecision == 1) {
        Heltec.display->drawString(24, 29, "MAX");
        Heltec.display->drawString(64, 29, "HIGH");
        Heltec.display->drawString(104, 29, "[LOW]");
      }
      break;
    case 7:
      Heltec.display->drawXbm(0, 0, 128, 64, menu7);
      Heltec.display->setFont(ArialMT_Plain_10);
      Heltec.display->setTextAlignment(TEXT_ALIGN_CENTER);
      Heltec.display->drawString(18, 25, String(tiltStartRange) + "°");
      Heltec.display->drawString(45, 25, String(tiltEndRange) + "°");
      Heltec.display->drawString(83, 25, String(baseStartRange) + "°");
      Heltec.display->drawString(114, 25, String(baseEndRange) + "°");
      break;
    case 8:
      Heltec.display->drawXbm(0, 0, 128, 64, menu8);
      Heltec.display->setFont(ArialMT_Plain_10);
      Heltec.display->setTextAlignment(TEXT_ALIGN_CENTER);
      Heltec.display->drawString(18, 25, String(tiltStartRange) + "°");
      Heltec.display->drawString(45, 25, String(tiltEndRange) + "°");
      Heltec.display->drawString(83, 25, String(baseStartRange) + "°");
      Heltec.display->drawString(114, 25, String(baseEndRange) + "°");
      break;
    case 9:
      Heltec.display->drawXbm(0, 0, 128, 64, menu9);
      Heltec.display->setFont(ArialMT_Plain_10);
      Heltec.display->setTextAlignment(TEXT_ALIGN_CENTER);
      Heltec.display->drawString(18, 25, String(tiltStartRange) + "°");
      Heltec.display->drawString(45, 25, String(tiltEndRange) + "°");
      Heltec.display->drawString(83, 25, String(baseStartRange) + "°");
      Heltec.display->drawString(114, 25, String(baseEndRange) + "°");
      break;
    case 10: // main scan screen
      Heltec.display->drawXbm(0, 0, 128, 64, menu10);
      Heltec.display->setTextAlignment(TEXT_ALIGN_CENTER);
      Heltec.display->setFont(ArialMT_Plain_10);
      Heltec.display->drawString(64, 0, "Scanning... Step: " + String(currentBaseStep -
                                 baseStartRange / 90 * (400 / scanResolution) + 1) + '/' +
                                 String(baseEndRange / 90 * (400 / scanResolution) - baseStartRange / 90 * (400 / scanResolution)));
      Heltec.display->drawProgressBar(0, 15, 120, 10, ceil((float(currentBaseStep - baseStartRange / 90 * (400 / scanResolution) + 1) /
                                      float(baseEndRange / 90 * (400 / scanResolution) - baseStartRange / 90 * (400 / scanResolution))) * 100));
      Heltec.display->drawString(64, 29, "Time remaining: " + String(calculateMinutesRemaining()) + " min");
      break;
    default:
      break;
  }
  Heltec.display->display();
}

// use  formula i made on desmos to continuously update remaining time, works pretty well in fact
int calculateMinutesRemaining() {
  return min(999, (int)(((millis() - timeAtScanStart) *
                         ((float(baseEndRange - baseStartRange) / 90 * (400 / scanResolution) /
                           float(currentBaseStep - baseStartRange / 90 * (400 / scanResolution)) - 1))) / 1000 / 60));
}

void pollButtons() {
  button1NewState = digitalRead(button1Pin);
  button2NewState = digitalRead(button2Pin);
  button3NewState = digitalRead(button3Pin);

  if (button1OldState == 1 && button1NewState == 0) {
    buttonPress(1);
  } else if (button2OldState == 1 && button2NewState == 0) {
    buttonPress(2);
  } else if (button3OldState == 1 && button3NewState == 0) {
    buttonPress(3);
  }

  button1OldState = button1NewState;
  button2OldState = button2NewState;
  button3OldState = button3NewState;
}

void loop() {

  drawScreen();
  pollButtons();
  delay(dt);

  if (started) {

    recalculateSteps();

    // update the progress bar
    drawScreen();

    playMelody(piezoPin, 3, 100);

    delay(250);

    // power on relay
    digitalWrite(relayPin, HIGH);

    delay(250);

    // move arm to starting position as set by range
    moveArm(-ARM_STEPS / 4 + tiltStartRange / 45 * (200 / scanResolution));
    theta = (tiltStartRange / 45 * (200 / scanResolution)) * thetaPerStep;

    delay(250);

    // move base to starting position as set by range
    moveBase(baseStartRange / 90 * (400 / scanResolution));
    phi = (baseStartRange / 90 * (400 / scanResolution)) * phiPerStep;

    //    short int tiltStartRange = 0;
    //    short int tiltEndRange = 180;
    //    short int baseStartRange = 0;
    //    short int baseEndRange = 360;

    delay(1000);

    timeAtScanStart = millis(); // when we started scan

    // main scanning loop
    while (currentBaseStep < (baseEndRange / 90) * (400 / scanResolution) && started) {

      currentArmStep = (tiltStartRange / 45 * (200 / scanResolution));

      drawScreen();

      // perform measurements, offset for radar distance being calculated from the back plate
      for (int i = 0; i < (tiltEndRange - tiltStartRange) / 45 * (200 / scanResolution); i++) { // i < POINT_STACK_STEPS - 1
        // take a bias measurement every 100 measurements
        if (i % 100 == 0) {
          d = lidar.distance() - 2;
        } else {
          d = lidar.distance(false) - 2;
        }

        // zero out nearby points and spherical to cartesian conversion
        if (d < 100 || d > 4500) {
          pointStackCartesian[i][0] = 0; // x
          pointStackCartesian[i][1] = 0; // y
          pointStackCartesian[i][2] = 0; // z
        } else {
          pointStackCartesian[i][0] = (d * sin(theta) * cos(phi)) + (lidarOffset * sin(phi)); // x
          pointStackCartesian[i][1] = (d * sin(theta) * sin(phi)) + (lidarOffset * -cos(phi)); // y
          pointStackCartesian[i][2] = d * cos(theta); // z
        }

        // move arm 1 step down (clockwise)
        moveArm(1);

        pollButtons();
      }

      // very last bottom point scan
      d = lidar.distance() - 2;

      if (d < 25) {
        pointStackCartesian[POINT_STACK_STEPS - 1][0] = 0; // x
        pointStackCartesian[POINT_STACK_STEPS - 1][1] = 0; // y
        pointStackCartesian[POINT_STACK_STEPS - 1][2] = 0; // z
      } else {
        pointStackCartesian[POINT_STACK_STEPS - 1][0] = (d * sin(theta) * cos(phi)) + (lidarOffset * sin(phi)); // x
        pointStackCartesian[POINT_STACK_STEPS - 1][1] = (d * sin(theta) * sin(phi)) + (lidarOffset * -cos(phi)); // y
        pointStackCartesian[POINT_STACK_STEPS - 1][2] = d * cos(theta); // z
      }

      // move arm back to starting position as set by range
      if (started) moveArm(-(tiltEndRange - tiltStartRange) / 45 * (200 / scanResolution)); // if (started) moveArm(-ARM_STEPS / 2);

      // reset arm theta according to where starting pos is
      theta = (tiltStartRange / 45 * (200 / scanResolution)) * thetaPerStep;

      // another delay to prevent slap
      delay(100 * sqrt(scanResolution));

      // move base 1 step counter-clockwise
      if (started) moveBase(1);

      // let the head settle from vibration of base moving
      delay(100 * sqrt(scanResolution));

      // send coordinates over bluetooth serial port
      for (int i = 0; i < POINT_STACK_STEPS; i++) {
        // check if we came across a "zeroed out" point, and if so, don't send it
        if (!((pointStackCartesian[i][0] < 1 && pointStackCartesian[i][0] > -1) &&
              (pointStackCartesian[i][1] < 1 && pointStackCartesian[i][1] > -1) &&
              (pointStackCartesian[i][2] < 1 && pointStackCartesian[i][2] > -1))) {
          SerialBT.print(pointStackCartesian[i][0], 2); // sending 2 decimal point precision
          SerialBT.print(',');
          SerialBT.print(pointStackCartesian[i][1], 2);
          SerialBT.print(',');
          SerialBT.println(pointStackCartesian[i][2], 2);
        }
      }
    }

    // play success melody and reset to zero
    if (started) playMelody(piezoPin, 1, 100);
    if (started) resetToZero();
    digitalWrite(relayPin, LOW);  // power off relay
    detachInterrupt(digitalPinToInterrupt(button3Pin));
    menuState = 1;
    started = false; // exit main loop and go back to main menu
  }
}

// uses notes[] array to manually play chosen melody through a specified pin with built-in tone() function
void playMelody(int pin, int song, int songDelay) {

  switch (song) {
    case 1:
      tone(pin, notes[0] * 2);
      delay(songDelay);
      tone(pin, notes[4] * 2);
      delay(songDelay);
      tone(pin, notes[7] * 2);
      delay(songDelay);
      tone(pin, notes[0] * 4);
      delay(songDelay * 2);
      tone(pin, notes[7] * 2);
      delay(songDelay);
      tone(pin, notes[0] * 4);
      delay(songDelay * 3);
      noTone(pin);
      break;
    case 2:
      tone(pin, notes[4]);
      delay(songDelay);
      tone(pin, notes[3]);
      delay(songDelay);
      tone(pin, notes[2]);
      delay(songDelay);
      tone(pin, notes[1]);
      delay(songDelay * 2.5);
      tone(pin, notes[10]);
      delay(songDelay * 2.5);
      noTone(pin);
      break;
    case 3: // start scan sound
      delay(songDelay);
      tone(pin, notes[0] * 8);
      delay(songDelay / 2);
      tone(pin, notes[7] * 8);
      delay(songDelay / 2);
      tone(pin, notes[0] * 8);
      delay(songDelay / 2);
      tone(pin, notes[7] * 8);
      delay(songDelay / 2);
      noTone(pin);
      break;
    case 4: // button press sound
      tone(pin, notes[0] * 8);
      delay(songDelay / 4);
      noTone(pin);
      break;
    case 5: // startup sound
      tone(pin, notes[0] * 8);
      delay(songDelay / 4);
      noTone(pin);
      delay(songDelay / 2);
      tone(pin, notes[0] * 8);
      delay(songDelay / 4);
      noTone(pin);
      break;
    default:
      break;
  }
}

// tone function redefinition to enable tone functionality for esp32
void tone(byte pin, int freq) {
  ledcSetup(0, 2000, 8); // setup beeper
  ledcAttachPin(pin, 0); // attach beeper
  ledcWriteTone(0, freq); // play tone
}
void noTone(int pin) {
  tone(pin, 0);
}
