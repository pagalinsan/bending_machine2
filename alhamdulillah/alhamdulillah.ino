#include <Encoder.h>
#include <LedControl.h>  // Include the LedControl library
int readPosition();

// If the either of the two padles are in bending mode and there
// encoder does not registers any motion till this time, then throw error.
unsigned int no_movement_time_period = 10000; // Milliseconds
long int bending_start_time = 0;

// Time limit between the start of bending and the end.
#define TOTAL_BENDING_TIME_LIMIT 40000 // Milliseconds
long int bending_start_timestamp = 0;

// Defining acceptable ranges for the default angle 
// (ideally it should be 0 degrees but practically it'll never be)
#define ACCEPTABLE_RANGE_LOWER_BOUND 4
#define ACCEPTABLE_RANGE_UPPER_BOUND 356


// Define a timer variable to track the time with no motion
unsigned long noMotionTimer = 0;
const unsigned long noMotionThreshold = 200; // Set the threshold to 200 milliseconds

bool calibrationModeEntered = false;


int currentP = 0;


// Push button delay
int buttonPressDelay = 200; // milliseconds
long int buttonPressTime = 0;
      // for 106 err
bool curPosMatchesDispCntr1 = false;
bool curPosMatchesDispCntr2 = false;

// Initialize Rotary Encode Class
#define SELECT_PIN 4
#define CLOCK_PIN 3
#define DATA_PIN 2

// Define the pins for MAX7219 DIsplay countroller.
const int DIN_PIN = 10; // MOSI (Master Out Slave In)
const int CLK_PIN = 12; // SCK (Serial Clock)
const int CS_PIN = 11;  // SS (Slave Select)

LedControl lc = LedControl(DIN_PIN, CLK_PIN, CS_PIN, 1); // 1 MAX7219 IC controlling both displays

// Rotary Encoder Sate tracking.
long currentCount = 0;
long previousCount = 0;
int curPos = 0;
float angle = 0;
float AngShow = 0;


// Buttons name to pin number.
int inc_disp_1_btn = A0;  // Button for increment display 1
int dec_disp_1_btn = A1;  // Button for decrement display 1
int inc_disp_2_btn = A2;  // Button for increment display 2
int dec_disp_2_btn = A3;  // Button for decrement display 2

//Direction Buttons
int relay_fwd_btn = A4;   //Button for Turn on revers Relay
int relay_rev_btn = A5;   //Button for Turn on forword Relay


// Mode Buttons
int pedal_90 = 8;   // Button to switch display 1 mode
int pedal_135 = 9;   // Button to switch display 2 mode

// Relay Outputs
int relay_fwd = 6;  
int relay_rev = 5;  
  
// Default Counter Value
int disp_cntr_1 = 90;   // Counter for display 1
int disp_cntr_2 = 135;   // Counter for display 2


int modeButtonState2 = HIGH;
int modeButtonStatePrev = HIGH;
int modeButtonState1 = HIGH;

int varCus = 0;

  //Display Mode 
  enum display_state {
      BENDING,
      SHOWING_COUNTER,
      Calibration_mode

  };


enum display_state prev_disp_state = SHOWING_COUNTER;
enum display_state display_1_state = SHOWING_COUNTER;
enum display_state display_2_state = SHOWING_COUNTER;

unsigned long incButtonsStartTime = 0;

bool bending_just_started = false;

//.....................................................................................
//..........................Code For Err Handling .....................................
void err(int err_code) {
  // err_code:
  // 1: Both relays working together.
  // 2: No motion in bending mode after 1.5 seconds.
  // 3: Rotary encoder reading greater or smaller than 180 and -4.
  // 4:
  // 5: If bending has started and it takes more than 4 seconds for it to finish the whol process.
  // 6: If the intital position is not at 0.

  lc.clearDisplay(0); // Clear the display
  
  while (1) {
    digitalWrite(relay_fwd, LOW);
    digitalWrite(relay_rev, LOW);

    switch (err_code) {
      case 1:
        lc.setChar(0, 0, 'e', false); // Display "e" for error code 1
        lc.setChar(0, 1, 'e', false); // Display "r" for error code 1
        lc.setChar(0, 2, 'e', false); // Display "1" for error code 1
        lc.setChar(0, 3, '1', false); // Display "e" for error code 1
        lc.setChar(0, 4, '0', false); // Display "r" for error code 1
        lc.setChar(0, 5, '1', false); // Display "1" for error code 1
        break;
      case 2:
        lc.setChar(0, 0, 'e', false); // Display "e" for error code 2
        lc.setChar(0, 1, 'e', false); // Display "r" for error code 2
        lc.setChar(0, 2, 'e', false); // Display "2" for error code 2
        lc.setChar(0, 3, '2', false); // Display "e" for error code 1
        lc.setChar(0, 4, '0', false); // Display "r" for error code 1
        lc.setChar(0, 5, '1', false); // Display "1" for error code 1
        break;
      case 3:
        lc.setChar(0, 0, 'e', false); // Display "e" for error code 3
        lc.setChar(0, 1, 'e', false); // Display "r" for error code 3
        lc.setChar(0, 2, 'e', false); // Display "3" for error code 3
        lc.setChar(0, 3, '3', false); // Display "e" for error code 1
        lc.setChar(0, 4, '0', false); // Display "r" for error code 1
        lc.setChar(0, 5, '1', false); // Display "1" for error code 1

      if (digitalRead(inc_disp_1_btn) == LOW && digitalRead(inc_disp_2_btn) == LOW) {
        if (incButtonsStartTime == 0) {
            incButtonsStartTime = millis();
        }

            if (millis() - incButtonsStartTime >= 1000) {
              display_1_state = Calibration_mode;
              display_2_state = Calibration_mode;
              calibrationModeEntered = true;
              enterCalibrationMode();
            }
      } else {
              // Handle the Calibration_mode state
               incButtonsStartTime = 0;   
              }
        break;
      
      case 4:
        lc.setChar(0, 0, 'e', false); // Display "e" for error code 3
        lc.setChar(0, 1, 'e', false); // Display "r" for error code 3
        lc.setChar(0, 2, 'e', false); // Display "3" for error code 3
        lc.setChar(0, 3, '4', false); // Display "e" for error code 1
        lc.setChar(0, 4, '0', false); // Display "r" for error code 1
        lc.setChar(0, 5, '1', false); // Display "1" for error code 1
        break;
      
      case 5:
        lc.setChar(0, 0, 'e', false); // Display "e" for error code 3
        lc.setChar(0, 1, 'e', false); // Display "r" for error code 3
        lc.setChar(0, 2, 'e', false); // Display "3" for error code 3
        lc.setChar(0, 3, '5', false); // Display "e" for error code 1
        lc.setChar(0, 4, '0', false); // Display "r" for error code 1
        lc.setChar(0, 5, '1', false); // Display "1" for error code 1
        break;
      default:
        lc.setChar(0, 3, 'E', false); // Display "E" for unknown error code
        break;
    }

    delay(500); // Display the error code for 500 milliseconds

    lc.clearDisplay(0);
    delay(500);
  }
} 
  //.......................................END.........................................

  //.................................Voide Setup Start.................................
void setup() {
  pinMode(DATA_PIN, INPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(SELECT_PIN, OUTPUT);
  digitalWrite(CLOCK_PIN, HIGH);
  digitalWrite(SELECT_PIN, HIGH);

  pinMode(inc_disp_1_btn, INPUT_PULLUP);
  pinMode(dec_disp_1_btn, INPUT_PULLUP);
  pinMode(inc_disp_2_btn, INPUT_PULLUP);
  pinMode(dec_disp_2_btn, INPUT_PULLUP);
  pinMode(pedal_90, INPUT_PULLUP);
  pinMode(pedal_135, INPUT_PULLUP);

  pinMode(relay_fwd_btn, INPUT_PULLUP);
  pinMode(relay_rev_btn, INPUT_PULLUP);


  pinMode(relay_fwd, OUTPUT);
  pinMode(relay_rev, OUTPUT);

  lc.shutdown(0, false); // Wake up the MAX7219
  lc.setIntensity(0, 8); // Set brightness (0-15, 0 being off, 15 being brightest)
  lc.clearDisplay(0);    // Clear the display
  // Initialize both displays with counting from 1 to 9 for each digit.
  for (int i = 9; i >= 0; i--) {
    // Set units digit on Display 1 and Display 2
    lc.setDigit(0, 0, i % 10, false); // Display units digit on Display 1
    lc.setDigit(0, 3, i % 10, false); // Display units digit on Display 2

    // Set tens digit on Display 1 and Display 2
    lc.setDigit(0, 1, (i / 10) % 10, false); // Display tens digit on Display 1
    lc.setDigit(0, 4, (i / 10) % 10, false); // Display tens digit on Display 2

    // Set hundreds digit on Display 1 and Display 2
    lc.setDigit(0, 2, i / 100, false); // Display hundreds digit on Display 1
    lc.setDigit(0, 5, i / 100, false); // Display hundreds digit on Display 2

    delay(250); // Delay to show each number for 500 milliseconds
  }
  
  //....................................End...........................................................
  //...............Code  For Print  601 when encoder is not in defoult position.......................

angle = readPosition(); // Update the current encoder value
    if (angle >= 0) {
      currentCount = ((float)angle / 1024.0) * 360.0;
    }

// When the encoder reads any value which is not between (358-2 degrees)
  if (currentCount > ACCEPTABLE_RANGE_LOWER_BOUND && currentCount < ACCEPTABLE_RANGE_UPPER_BOUND) {
    lc.setDigit(0, 0, 1, false); // Display 6 on the leftmost digit
    lc.setDigit(0, 1, 0, false); // Display 0 on the middle digit
    lc.setDigit(0, 2, 6, false); // Display 1 on the rightmost digit

    while (currentCount > ACCEPTABLE_RANGE_LOWER_BOUND && currentCount < ACCEPTABLE_RANGE_UPPER_BOUND) {
                    if (currentCount > 183 && currentCount < ACCEPTABLE_RANGE_UPPER_BOUND) {
                    err(3);
                    }
                   angle = readPosition(); // Update the current encoder value
                  if (angle >= 0) {
                  currentCount = ((float)angle / 1024.0) * 360.0;
                  }


            if (digitalRead(inc_disp_1_btn) == LOW && digitalRead(inc_disp_2_btn) == LOW) {
                if (incButtonsStartTime == 0) {
                  incButtonsStartTime = millis();
                }
              if (millis() - incButtonsStartTime >= 1000) {
                display_1_state = Calibration_mode;
                display_2_state = Calibration_mode;
                calibrationModeEntered = true;
                enterCalibrationMode();
            
              }
            } 
                
          else {
          // Handle the Calibration_mode state
          incButtonsStartTime = 0;
        
          }
      if (digitalRead(pedal_90) == LOW) {
              digitalWrite(relay_rev, HIGH);
              angle = readPosition(); // Update the current encoder value
        if (angle >= 0) {
              currentCount = ((float)angle / 1024.0) * 360.0;
        }
          updateDisplay(0, curPos, false);
      }
    }
  }
  digitalWrite(relay_rev, LOW);
}
  
//...................................END........................................................
//............... Code for Determine the digits for the display based on the value..............
void updateDisplay(byte address, int value, bool reverse) {
  // Determine the digits for the display based on the value.
  int digit1 = value % 10;
  int digit2 = (value / 10) % 10;
  int digit3 = value / 100;

 
  lc.setDigit(0, address, digit1, false); // Display units digit
  lc.setDigit(0, address + 1, digit2, false); // Display tens digit
  lc.setDigit(0, address + 2, digit3, false); // Display hundreds digit
}
//..............................................END........................................................
//....................................code for encoder function............................................
int readPosition() {
  unsigned int position = 0;

  // Shift in our data  
  digitalWrite(SELECT_PIN, LOW);
  delayMicroseconds(1);
  byte d1 = shiftIn(DATA_PIN, CLOCK_PIN);
  byte d2 = shiftIn(DATA_PIN, CLOCK_PIN);
  digitalWrite(SELECT_PIN, HIGH);

  // Get our position variable
  position = d1;
  position = position << 8;
  position |= d2;

  position = position >> 6;

  // Check the offset compensation flag: 1 == started up
  if (!(d2 & B00100000))
    position = -1;

  // Check the cordic overflow flag: 1 = error
  if (d2 & B00010000)
    position = -2;

  // Check the linearity alarm: 1 = error
  if (d2 & B00001000)
    position = -3;

  // Check the magnet range: 11 = error
  if ((d2 & B00000110) == B00000110)
    position = -4;

  return position;
}


int calculate_offset(int x) {
  if (x < ACCEPTABLE_RANGE_UPPER_BOUND && x > ACCEPTABLE_RANGE_LOWER_BOUND) err(5);
  if (x => ACCEPTABLE_RANGE_UPPER_BOUND) return 360-x;
  if (x <= ACCEPTABLE_RANGE_LOWER_BOUND) return -x;
}

    //........................................END...................................................
    //..................................voide loop start............................................
void loop() {

    //...................................................................................................
    //...........................Code for Calibration_mode...............................................    
    
    // Define a flag to track if Calibration_mode is entered
    bool calibrationModeEntered = false;

    // Check if both inc1 and inc2 buttons are pressed for 3 seconds
    // Check if both inc1 and inc2 buttons are pressed for 3 seconds

      if (digitalRead(inc_disp_1_btn) == LOW && digitalRead(inc_disp_2_btn) == LOW) {
          if (incButtonsStartTime == 0) {
            incButtonsStartTime = millis();
          }

        if (millis() - incButtonsStartTime >= 1000) {
            display_1_state = Calibration_mode;
            display_2_state = Calibration_mode;
            calibrationModeEntered = true;
            enterCalibrationMode();
        }
      }   
    else{
    // Handle the Calibration_mode state
    incButtonsStartTime = 0;   
}


//.....................................END......................................................
//.............Code for  turns the reverse relay and forward relay with button..................

    // Read the state of relay buttons
    int revButtonState = digitalRead(relay_rev_btn);
    int fwdButtonState = digitalRead(relay_fwd_btn);

        // If reverse button is pressed, turn on the reverse relay
        if (revButtonState == LOW) {
        digitalWrite(relay_rev, HIGH);
        angle = readPosition(); // Update the current encoder value
          if (angle >= 0) {
            currentCount = ((float)angle / 1024.0) * 360.0;
          }
        updateDisplay(0, curPos, false);  // Update display 1
        updateDisplay(3, curPos, false);  // Update display 2
        } 
          else if ((display_1_state != BENDING) && (display_2_state != BENDING)) {
          // If the button is released, turn off the reverse relay
          digitalWrite(relay_rev, LOW);
        }

    // If forward button is pressed, turn on the forward relay
    if (fwdButtonState == LOW) {
        digitalWrite(relay_fwd, HIGH);
        // Update both displays with the encoder value
        angle = readPosition(); // Update the current encoder value
        if (angle >= 0) {
          currentCount = ((float)angle / 1024.0) * 360.0;
        }
        updateDisplay(0, curPos, false);  // Update display 1
        updateDisplay(3, curPos, false);  // Update display 2
    }
      else if ((display_1_state != BENDING) && (display_2_state != BENDING)) {
        // If the button is released, turn off the forward relay
        digitalWrite(relay_fwd, LOW);
      }
    //..........................................END..................................................

    //................Code For print err5 when bendimg time is greater than 1 sec....................

    if ( (display_1_state == BENDING || display_1_state == BENDING) && millis() - bending_start_timestamp > TOTAL_BENDING_TIME_LIMIT ) err(5);
    //..........................................END..................................................

    //.....................Code for print err1 when both relays working together.....................
    if (digitalRead(relay_fwd) == HIGH && digitalRead(relay_rev) == HIGH) err(1);
    //..........................................END..................................................


    if ( display_1_state != BENDING && display_2_state != BENDING) {
        int inc1 = digitalRead(inc_disp_1_btn);
        int dec1 = digitalRead(dec_disp_1_btn);
        int inc2 = digitalRead(inc_disp_2_btn);
        int dec2 = digitalRead(dec_disp_2_btn);

            
  //.................................Code For Increment Display 1...................................
        if (inc1 == LOW && disp_cntr_1 < 180 && (millis() - buttonPressTime) > buttonPressDelay) {
          disp_cntr_1++;
          buttonPressTime = millis();
        }
  //..........................................END....................................................
  //.................................Code For Decrement Display 1....................................
        if (dec1 == LOW && disp_cntr_1 > 0 && (millis() - buttonPressTime) > buttonPressDelay) {
          disp_cntr_1--;
          buttonPressTime = millis();
        }
  //..........................................END....................................................
  //.............................Code For Increment Display 2........................................
        if (inc2 == LOW && disp_cntr_2 < 180 && (millis() - buttonPressTime) > buttonPressDelay) {
          disp_cntr_2++;
          buttonPressTime = millis();
        }
  //..........................................END....................................................

  //.............................. Code For Decrement Display 2......................................
        if (dec2 == LOW && disp_cntr_2 > 0 && (millis() - buttonPressTime) > buttonPressDelay) {
          disp_cntr_2--;
          buttonPressTime = millis(); 
        }
    }
  //..........................................END....................................................
    // Read both mode buttons
    modeButtonState1= digitalRead(pedal_135);
    modeButtonState2 = digitalRead(pedal_90);

  //.....If beding has started but the encoder is not moving for 3 or whaever seconds then call error.......
    if (bending_just_started && (millis() - bending_start_time > no_movement_time_period) ) err(2);
  //.....................................END................................................................


    //..............................Code For bending mode 90 Default........................................
    if (modeButtonState1 == LOW && display_2_state != BENDING && display_1_state != BENDING) {
      angle = readPosition(); // Update the current encoder value
      if (angle >= 0) {
        currentCount = ((float)angle / 1024.0) * 360.0;
        currentP = calculate_offset(currentCount); // Store the initial position before bending starts
      }
    
        display_1_state = BENDING;
        bending_start_time = millis();
        bending_start_timestamp = millis(); // This is a global-ish counter which measures the whole time (bending_retracting) for a bending session
        bending_just_started = true;
        delay(200);
        digitalWrite(relay_fwd, HIGH); // Start relay FWD (90 Default)
        prev_disp_state = BENDING;
    }
    //...........................................End........................................................
    
    //..............................Code For bending mode 135 Default........................................
    
    if (modeButtonState2 == LOW && display_1_state != BENDING && display_2_state != BENDING) {
        angle = readPosition(); // Update the current encoder value
        if (angle >= 0) {
            currentCount = ((float)angle / 1024.0) * 360.0;
            currentP = calculate_offset(currentCount);
        
        }
        display_2_state = BENDING;
        bending_start_time = millis();
        bending_start_timestamp = millis();
        bending_just_started = true;
        delay(200);
        digitalWrite(relay_fwd, HIGH); // Start relay FWD (135 default)
        prev_disp_state = BENDING;
    }
    //...........................................End........................................................

    // Read the current encoder value.
    angle = readPosition();
   if (angle >= 0)
   {
    currentCount = ((float)angle / 1024.0) * 360.0;
   }
    // Check if the encoder value has changed
    if (currentCount != previousCount) {

        if (bending_just_started = false);
        
        curPos += (currentCount - previousCount); // Update the current position
        previousCount = currentCount; // Update the previous count
    }

    if (curPos > 183 && curPos < ACCEPTABLE_RANGE_UPPER_BOUND ) {
      err(3);
    }
    
    AngShow = curPos - currentP;
    // Update Display 1 with Encoder or Counter Value
    if (display_1_state == BENDING) updateDisplay(0, AngShow, false);
    else updateDisplay(0, disp_cntr_1, false);
    

    // Update Display 2 with Encoder or Counter Value
    if (display_2_state == BENDING) updateDisplay(3, AngShow, false);
    else updateDisplay(3,disp_cntr_2, false);


    // Check if the either of the counters have reached the desired postion.
    // If so, then stop bending and reverse.
    if ((curPos - currentP) == disp_cntr_1 && display_1_state == BENDING) {
        curPosMatchesDispCntr1 = true;
        digitalWrite(relay_fwd, LOW);
        digitalWrite(relay_rev, HIGH);
    }
    if ((curPos - currentP)== disp_cntr_2 && display_2_state == BENDING) {
        curPosMatchesDispCntr2 = true;
        digitalWrite(relay_fwd,LOW);
        digitalWrite(relay_rev, HIGH);
    }

  if ((curPos <= 0 || curPos == 1 || curPos == 2 || curPos == 3 || curPos == 4 ||curPos == 356 || curPos == 357 || curPos == 358 || curPos == 359) && bending_just_started == false) {
    if (display_1_state == BENDING || display_2_state == BENDING) {
      if (curPosMatchesDispCntr1 || curPosMatchesDispCntr2) {
          unsigned long noChangeTimer = millis();
          long int initialEncoderValue = readPosition();

        while (millis() - noChangeTimer <= 500) {
          
          // Check if the encoder value has changed
          if (readPosition() != initialEncoderValue) {
              noChangeTimer = millis(); // Reset the timer if there is a change
              initialEncoderValue = readPosition(); // Update the initial encoder value
          }
            // delay(100); // Adjust the delay based on the desired interval
            angle = readPosition();
            if (angle >= 0) currentCount = ((float)angle / 1024.0) * 360.0;
            updateDisplay( (display_1_state == BENDING)? 0:3 , currentCount - currentP , false);
        }

        delay(300);

        digitalWrite(relay_rev, LOW);

        curPosMatchesDispCntr1 = false;
        curPosMatchesDispCntr2 = false;
        display_1_state = SHOWING_COUNTER;
        display_2_state = SHOWING_COUNTER;
        curPos = 0;
        previousCount = 0;
        currentCount = 0;
      }
    }
  }
}

byte shiftIn(byte data_pin, byte clock_pin) {
  byte data = 0;

  for (int i=7; i>=0; i--) {
    digitalWrite(clock_pin, LOW);
    delayMicroseconds(1);
    digitalWrite(clock_pin, HIGH);
    delayMicroseconds(1);

    byte bit = digitalRead(data_pin);
    data |= (bit << i);
  }
  return data;
}
  void enterCalibrationMode() {
    display_1_state = Calibration_mode;
    calibrationModeEntered = true;

        while (display_1_state == Calibration_mode || display_2_state == Calibration_mode) {
        int encoderValue = readPosition();
        if (encoderValue >= 0) {
            curPos = ((float)encoderValue / 1024.0) * 360.0;
        }
        updateDisplay(0, curPos, false);
        updateDisplay(3, curPos, false);
    }
}