#include <Servo.h>
#include <EEPROM.h>

// Rotary encoder pins
#define ROTARYA A2
#define ROTARYB A3
#define ROTBUTTON A1

#define SERVOPIN A0
Servo vaultServo;


#define BUZZER 13
#define REDLED A4
#define GREENLED A5
#define DOORBUTTON 11
#define RESETBUTTON 12

#define SHIFTCLK 2

const int SHIFT1[2] = {
  3,  // Data
  4   // Latch
};

const int SHIFT2[2] = {
  5,  // Data
  6   // Latch
};

#define BCD1 7
#define BCD2 8
#define BCD3 9
#define BCD4 10

const int RomPass1 = 0;
const int RomPass2 = 2;
const int RomPass3 = 4;
const int RomEdited = 6;
bool value;

byte bcdNum[11][4] = {
  { 0, 0, 0, 0 },  // 0
  { 1, 0, 0, 0 },  // 1
  { 0, 1, 0, 0 },  // 2
  { 1, 1, 0, 0 },  // 3
  { 0, 0, 1, 0 },  // 4
  { 1, 0, 1, 0 },  // 5
  { 0, 1, 1, 0 },  // 6
  { 1, 1, 1, 0 },  // 7
  { 0, 0, 0, 1 },  // 8
  { 1, 0, 0, 1 },  // 9
  { 1, 1, 1, 1 }   // Off
};

int lastStateClk;

const char common = 'c';  // 7-segment with common cathode


// cases for detecting the binary of the numbers given from 0 to 9 */
byte binary(int Number) {
  switch (Number) {
    case 0:
      return B11111100;  // if 0 is given then return binary for zero
      break;
    case 1:
      return B01100000;  // if 1 is given then return binary for one
      break;
    case 2:
      return B11011010;  // if 2 is given then return binary for two
      break;
    case 3:
      return B11110010;  // if 3 is given then return binary for three
      break;
    case 4:
      return B01100110;  // if 4 is given then return binary for four
      break;
    case 5:
      return B10110110;  // if 5 is given then return binary for five
      break;
    case 6:
      return B00111110;  // if 6 is given then return binary for six
      break;
    case 7:
      return B11100000;  // if 7 is given then return binary for seven
      break;
    case 8:
      return B11111110;  // if 8 is given then return binary for eight
      break;
    case 9:
      return B11100110;  // if 9 is given then return binary for nine
    case 10:
      return B00000000;  // if 10 is given then set a blank screen
  }
  return B00000000;
}

// Function to write an int into the EEPROM. EEPROM uses bytes too store data, and since int is 2 bytes we place it in accordingly using bitwise operations
void writeIntIntoEEPROM(int address, int number) {
  byte byte1 = number >> 8;
  byte byte2 = number & 0xFF;
  EEPROM.write(address, byte1);
  EEPROM.write(address + 1, byte2);
}

// Reading from EEPROM and returning a read int. The int is taken from 2 bytes, so it needs to be reconstucted using bitwise operations
int readIntFromEEPROM(int address) {
  return (EEPROM.read(address) << 8) + EEPROM.read(address + 1);
}

// Function used to display a byte number to the 7-segment
void display(byte eightBits, const int shift[]) {
  if (common == 'c') {                  // If the configuration is common cathode
    eightBits = eightBits ^ B11111111;  // Change the bits from 0 to 1
  }
  digitalWrite(shift[1], LOW);                        // Preparing shift register for data1
  shiftOut(shift[0], SHIFTCLK, LSBFIRST, eightBits);  // Sending the data1 to  7-segmnet starting from the least significant bit
  digitalWrite(shift[1], HIGH);                       // Displaying the data1 on the 7 segment
}

// Writing the BCD number through individual pins using the bcdNum array initialized at the top
void writeNum(byte num) {
  digitalWrite(BCD1, bcdNum[num][0]);
  digitalWrite(BCD2, bcdNum[num][1]);
  digitalWrite(BCD3, bcdNum[num][2]);
  digitalWrite(BCD4, bcdNum[num][3]);
}

// Change the numbers in the array using the rotary encoder
void changeNum(int *currentStateClk, int *currentPass, int *pos) {
  // If the data from ROTARYB is different from the current state that means we are
  // rotating CW so we increment
  if (digitalRead(ROTARYB) != *currentStateClk) {
    // Check if the password is getting higher than 18. We use 18 becuase even though the
    // rotary encoder is debounced, it still increments and decrements in values of 2
    if (currentPass[*pos] > 18) {
      currentPass[*pos] = 0;
    } else {
      currentPass[*pos]++;
    }
  // Else the rotary is rotating CCW so we decrease the number
  } else {
    if (currentPass[*pos] < 0) {
      currentPass[*pos] = 18;
    } else {
      currentPass[*pos]--;
    }
  }
}

// Check if the rotary button is pressed to move the pos of the password
void nextButtonCheck(int *pos) {
  int buttonState = digitalRead(ROTBUTTON);
  // Since it is a INPUT_PULLUP it means the button has an input of LOW when being pressed
  if (buttonState == LOW) {
    // Wait for user to release button
    while (buttonState == LOW) {
      buttonState = digitalRead(ROTBUTTON);
      delay(10);
    }
    *pos = *pos + 1;
  }
}

// Check whether the password is correct and return a bool
bool passCompleted(int *originalPass, int *currentPass) {
  // Check the password
  for (int i = 0; i < 3; i++) {
    // If it is wrong, reset the currentPass and display red led and buzzer
    if (originalPass[i] != currentPass[i] / 2) {
      for (int j = 0; j < 3; j++) {
        currentPass[j] = 0;
      }
      digitalWrite(BUZZER, HIGH);
      digitalWrite(REDLED, HIGH);
      delay(50);
      digitalWrite(BUZZER, LOW);
      delay(50);
      digitalWrite(BUZZER, HIGH);
      delay(150);
      digitalWrite(BUZZER, LOW);
      digitalWrite(REDLED, LOW);
      return false;
    }
    // If the end is reached and everything matches, return true after turning on buzzer
    if (i == 2) {
      for (int j = 0; j < 2; j++) {
        digitalWrite(BUZZER, HIGH);
        delay(50);
        digitalWrite(BUZZER, LOW);
        delay(50);
      }
      return true;
    }
  }
}

// Button that resets user input
void resetButton(int *currentPass, int *pos) {
  int resetState = digitalRead(RESETBUTTON);
  if (resetState == LOW) {
    while (resetState == LOW) {
      resetState = digitalRead(RESETBUTTON);
      delay(10);
    }
    // Set the currentPass to 0
    for (int i = 0; i < 3; i++) {
      currentPass[i] = 0;
      *pos = 0;
    }
  }
}

// Check if the door button is pressed
void doorButton(int *currentPass, int *pos, bool *unlocked) {
  int doorState = digitalRead(DOORBUTTON);
  if (doorState == LOW) {
    // Wait fopr user to release button
    while (doorState == LOW) {
      doorState = digitalRead(DOORBUTTON);
      delay(10);
    }
    for (int i = 0; i < 3; i++) {
      currentPass[i] = 0;
    }
    // Lock after resetting
    *pos = 0;
    *unlocked = false;
  }
}

// Change the password after a set delay
void changePass(int *currentPass, int changeState, bool *pressed, int *pressedTime, int *releaseTime, bool *changingPass, const int period) {
  if (changeState == LOW && !*pressed) {
    // Record time that button was pressed and mark that the button was pressed
    *pressedTime = millis();
    *pressed = true;
  } else if (changeState == HIGH && *pressed) {
    // release button and set release time
    *pressed = false;
    *releaseTime = millis();
    // calculate the difference of the times and check if they are more than the set delay
    if ((*releaseTime - *pressedTime) > period) {
      *changingPass = true;
    }
  }
  // Reset currentPass after changing the saved password
  for (int i = 0; i < 3; i++) {
    currentPass[i] = 0;
  }
}

void setup() {

  // Set all the pins
  pinMode(ROTARYA, INPUT);
  pinMode(ROTARYB, INPUT);
  pinMode(ROTBUTTON, INPUT_PULLUP);

  vaultServo.attach(SERVOPIN);

  pinMode(SHIFT1[0], OUTPUT);
  pinMode(SHIFT1[1], OUTPUT);
  pinMode(SHIFTCLK, OUTPUT);
  pinMode(SHIFT2[0], OUTPUT);
  pinMode(SHIFT2[1], OUTPUT);

  pinMode(BCD1, OUTPUT);
  pinMode(BCD2, OUTPUT);
  pinMode(BCD3, OUTPUT);
  pinMode(BCD4, OUTPUT);

  pinMode(GREENLED, OUTPUT);
  pinMode(REDLED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(DOORBUTTON, INPUT_PULLUP);
  pinMode(RESETBUTTON, INPUT_PULLUP);


  // Set bcd display to be empty
  digitalWrite(BCD1, 1);
  digitalWrite(BCD2, 1);
  digitalWrite(BCD3, 1);
  digitalWrite(BCD4, 1);

  lastStateClk = digitalRead(ROTARYA);
  value = (bool)readIntFromEEPROM(RomEdited);
  // If RomEdited has no value, assign the password 123
  if (!value) {
    writeIntIntoEEPROM(RomPass1, 1);
    writeIntIntoEEPROM(RomPass2, 2);
    writeIntIntoEEPROM(RomPass3, 3);
    writeIntIntoEEPROM(RomEdited, 1);
  }
}

void loop() {
  static bool unlocked = false;
  static int pressedTime;
  static bool pressed = false;
  static int releasedTime;
  static const int period = 3000;
  static int currentStateClk;
  static int blinkTime = millis();
  static const int blinkPeriod = 250;

  static bool changingPass = false;

  static int originalPass[3] = { 1, 2, 3 };
  static int currentPass[3] = { 0, 0, 0 };
  static int pos = 0;

  // Runs if vault is lcoked
  if (!unlocked) {
    // Turn green led off
    digitalWrite(GREENLED, LOW);

    // Read from the EEPROM and set is as original pass
    originalPass[0] = readIntFromEEPROM(RomPass1);
    originalPass[1] = readIntFromEEPROM(RomPass2);
    originalPass[2] = readIntFromEEPROM(RomPass3);
    // Read the current state of the clock
    currentStateClk = digitalRead(ROTARYA);
    vaultServo.write(180);

    // If last and current state of CLK are different, then pulse occurred
    // React to only 1 state change to avoid double count
    if (currentStateClk != lastStateClk) {
      changeNum(&currentStateClk, currentPass, &pos);
    }
    // Remember last CLK state
    lastStateClk = currentStateClk;
    nextButtonCheck(&pos);
    // At end of array submit the password
    if (pos > 2) {
      pos = 0;
      unlocked = passCompleted(originalPass, currentPass);
    }
    resetButton(currentPass, &pos);

  }

  // If unlocked and not changing the passwrod
  else if (unlocked && !changingPass) {
    int changeState = digitalRead(ROTBUTTON);

    // Unlock the vault and turn on the green led
    vaultServo.write(90);
    digitalWrite(GREENLED, HIGH);

    // Check for door button and the password change button
    doorButton(currentPass, &pos, &unlocked);
    changePass(currentPass, changeState, &pressed, &pressedTime, &releasedTime, &changingPass, period);
  }

  // If unlocked and changing the password
  else if (unlocked && changingPass) {

    currentStateClk = digitalRead(ROTARYA);

    // If last and current state of CLK are different, then pulse occurred
    // React to only 1 state change to avoid double count
    if (currentStateClk != lastStateClk) {
      changeNum(&currentStateClk, currentPass, &pos);
    }
    lastStateClk = currentStateClk;
    // Check for next and reset buttons being pressed
    resetButton(currentPass, &pos);
    nextButtonCheck(&pos);
    // At end of password replace the originalPass with current pass. We use currentPass[i] / 2 because
    // the password increments by 2, and we divide by 2 to compensate
    if (pos > 2) {
      pos = 0;
      for (int i = 0; i < 3; i++) {
        originalPass[i] = currentPass[i] / 2;
      }
      // Stop chaning the pass and lock the bault once password is changed
      changingPass = false;
      unlocked = false;
      // Reset currentPass
      for (int j = 0; j < 3; j++) {
        currentPass[j] = 0;
      }
      // Add the new pass to EEPROM
      writeIntIntoEEPROM(RomPass1, originalPass[0]);
      writeIntIntoEEPROM(RomPass2, originalPass[1]);
      writeIntIntoEEPROM(RomPass3, originalPass[2]);
      writeIntIntoEEPROM(RomEdited, 1);
    }
    // This blinks the green led to signify the changing of the password
    if ((millis() % blinkPeriod) > (blinkPeriod / 2)) {
      digitalWrite(GREENLED, LOW);
    } else if ((millis() % blinkPeriod) < (blinkPeriod / 2)) {
      digitalWrite(GREENLED, HIGH);
    }
    resetButton(currentPass, &pos);
  }

  // Display 2 numbers on the shift, and one on the bcd. We devide by 2 to compensate for the 2 inputs that one rotation
  // that the rotary encoder gives us
  display(binary((currentPass[0]) / 2), SHIFT1);
  display(binary(currentPass[1] / 2), SHIFT2);
  writeNum(currentPass[2] / 2);
}