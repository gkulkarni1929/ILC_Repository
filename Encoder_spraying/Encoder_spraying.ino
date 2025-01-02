#include <genieArduinoDEV.h>
#include <HardwareSerial.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;
Genie genie;
HardwareSerial HMI(2);

#define RESETLINE 4
#define ENCODER_A 32       // Pin for Encoder A White ENCA
#define ENCODER_B 33       // Pin for Encoder B Green ENCB
#define IR_IN 35           // Pin for IR_IN sensor PE1
#define IR_OUT 34          // Pin for IR_OUT sensor PE2
#define enc_wheel_diam 64  //mm
#define pi 3.14159
#define ppr 1200
#define sqm_to_sqft 10.7639
#define molder_speed 25                // m/min
#define req_coverage 79.2602377807133  //sqft/l
#define fixed_length 163.0             //mm

float current_mA = 0;
float length = 0.0;
float total_length = 0.0;
bool measuring_length = false;  // Flag to track when measurement is happening
float sqft = 0.0;
float litres_req = 0.0;

volatile int encoder_value = 0;
int last_encoder_value = 0;
int ir_sensor_value_in = 1;
int ir_sensor_value_out = 1;
bool board_detected = false;  // This is the flag

const int spray[5] = { 13, 14, 27, 26, 25 };  // 5 relays for solenoid valves

//HMI Varibales
int profile_state = 0;
int profile_selected = 0;
int system_mode_state = 0;
int system_mode_selected = 0;
int spray_manual_state = 0;
int spray_manual_selected = 0;
const int ir_led_in = 10;
const int ir_led_out = 12;
const int encoder_led = 11;
const int spray_selected_led[5] = { 0, 1, 2, 3, 4 };
const int spray_active_led[5] = { 5, 6, 7, 8, 9 };
const int spray_manual[5] = { 10, 11, 12, 13, 14 };
const int images[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };  // 0 - 4 (spray image), 5 (laser beam image), 6 (board image)

int system_mode = 2;  //0 = Standby, 1 = Manual,  2 = Automatic

unsigned long start_time = 0;  // To store the start time
unsigned long end_time = 0;    // To store the end time
unsigned long time_taken = 0;  // To store the calculated time taken

void setup() {

  Serial.begin(115200);

  ina219.begin();
  ina219.setCalibration_16V_400mA();

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(IR_IN, INPUT);
  pinMode(IR_OUT, INPUT);

  for (int i = 0; i < 5; i++) {
    pinMode(spray[i], OUTPUT);
  }
  for (int i = 0; i < 5; i++) {
    digitalWrite(spray[i], LOW);
  }

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoder_isr, CHANGE);

  HMI.begin(115200, SERIAL_8N1, 16, 17);  // Begin Serial1 at 9600 baud rate, RX on GPIO 16, TX on GPIO 17
  resetHMI();
  genie.Begin(HMI);
  genie.AttachEventHandler(myGenieEventHandler);
  genie.WriteContrast(15);
}

void loop() {

  genie.DoEvents();

  current_mA = ina219.getCurrent_mA();

  if (system_mode == 2) {

    length = (encoder_value * pi * enc_wheel_diam) / ppr;                // length in mm
    sqft = fixed_length / 1000.0 * total_length / 1000.0 * sqm_to_sqft;  // area in sqft , converting sqm to sqft
    litres_req = (sqft / req_coverage) * 1000.0;


    // Read the IR sensor
    ir_sensor_value_in = digitalRead(IR_IN);
    ir_sensor_value_out = digitalRead(IR_OUT);

    genie.WriteObject(GENIE_OBJ_USER_LED, encoder_led, 0);
    genie.WriteObject(GENIE_OBJ_USER_LED, ir_led_in, 0);
    genie.WriteObject(GENIE_OBJ_USER_LED, ir_led_out, 0);
    genie.WriteObject(GENIE_OBJ_USERIMAGES, images[5], 0);
    genie.WriteObject(GENIE_OBJ_USERIMAGES, images[7], 0);

    // Check if the encoder has moved
    bool encoder_moving = (encoder_value != last_encoder_value);
    if (encoder_moving) {
      last_encoder_value = encoder_value;
      genie.WriteObject(GENIE_OBJ_USER_LED, encoder_led, 1);
    }

    // Start length measurement if IR sensor detects a board and the encoder is moving
    if (ir_sensor_value_in == 0 && encoder_moving) {
      if (!measuring_length) {
        measuring_length = true;  // Start measuring
        encoder_value = 0;        // Reset encoder count
        start_time = millis();    // Record the start time
      }
    }

    // Stop length measurement if the IR sensor no longer detects the board
    if (ir_sensor_value_in == 1 && measuring_length) {
      measuring_length = false;            // Stop measuring
      total_length = length;               // Store the total measured length
      end_time = millis();                 // Record the end time
      time_taken = end_time - start_time;  // Calculate the time taken in milliseconds
    }

    if (ir_sensor_value_in == 0) {
      genie.WriteObject(GENIE_OBJ_USER_LED, ir_led_in, 1);
      genie.WriteObject(GENIE_OBJ_USERIMAGES, images[5], 1);
    }

    if (ir_sensor_value_out == 0) {
      genie.WriteObject(GENIE_OBJ_USER_LED, ir_led_out, 1);
      genie.WriteObject(GENIE_OBJ_USERIMAGES, images[7], 1);
    }

    // Implementing the logic based on the table
    if ((ir_sensor_value_in == 0 && encoder_moving && ir_sensor_value_out == 1) || (ir_sensor_value_in == 0 && encoder_moving && ir_sensor_value_out == 0) || (ir_sensor_value_in == 1 && !encoder_moving && ir_sensor_value_out == 0)) {

      board_detected = true;
      genie.WriteObject(GENIE_OBJ_USERIMAGES, images[6], 1);

      switch (profile_selected) {
        case 1:
          for (int i = 0; i < 5; i++) {
            genie.WriteObject(GENIE_OBJ_USER_LED, spray_active_led[i], 1);
            genie.WriteObject(GENIE_OBJ_USERIMAGES, images[i], 1);
            digitalWrite(spray[i], HIGH);
          }
          break;

        case 2:
          for (int i = 0; i < 4; i++) {
            genie.WriteObject(GENIE_OBJ_USER_LED, spray_active_led[i], 1);
            genie.WriteObject(GENIE_OBJ_USERIMAGES, images[i], 1);
            digitalWrite(spray[i], HIGH);
          }
          break;

        case 3:
          for (int i = 0; i < 3; i++) {
            genie.WriteObject(GENIE_OBJ_USER_LED, spray_active_led[i], 1);
            genie.WriteObject(GENIE_OBJ_USERIMAGES, images[i], 1);
            digitalWrite(spray[i], HIGH);
          }
          break;

        case 4:
          for (int i = 0; i < 2; i++) {
            genie.WriteObject(GENIE_OBJ_USER_LED, spray_active_led[i], 1);
            genie.WriteObject(GENIE_OBJ_USERIMAGES, images[i], 1);
            digitalWrite(spray[i], HIGH);
          }
          break;
      }
    } else {

      board_detected = false;

      for (int i = 0; i < 5; i++) {
        digitalWrite(spray[i], LOW);
      }

      for (int i = 0; i < 5; i++) {
        genie.WriteObject(GENIE_OBJ_USER_LED, spray_active_led[i], 0);
      }

      for (int i = 0; i < 8; i++) {
        genie.WriteObject(GENIE_OBJ_USERIMAGES, images[i], 0);
      }
    }

    // Print the current status
    Serial.print("Profile: ");
    Serial.print(profile_selected);
    Serial.print(" | IR sensor in: ");
    Serial.print(ir_sensor_value_in);
    Serial.print(" | IR sensor out: ");
    Serial.print(ir_sensor_value_out);
    // Serial.print(" | Encoder value: ");
    // Serial.print(encoder_value);
    Serial.print(" | Distance travelled: ");
    Serial.print(length);
    Serial.print(" | Measured length: ");
    Serial.print(total_length);
    Serial.print(" | SQFT: ");
    Serial.print(sqft);
    Serial.print(" | Millilitres required: ");
    Serial.print(litres_req);
    Serial.print(" | Time taken: ");
    Serial.print(time_taken);
    Serial.print(" | Encoder: ");
    // Serial.print(encoder_moving ? "M" : "NM");
    Serial.print(" | Board status: ");
    Serial.print(board_detected ? "yes" : "no");
    Serial.print(" | Flag: ");
    Serial.println(board_detected ? "yes" : "no");
  }
}

void resetHMI() {
  pinMode(RESETLINE, OUTPUT);
  digitalWrite(RESETLINE, 0);
  delay(100);
  digitalWrite(RESETLINE, 1);
  delay(3500);
}

void reset_hmi_form0() {

  genie.WriteObject(GENIE_OBJ_USER_LED, encoder_led, 0);
  genie.WriteObject(GENIE_OBJ_USER_LED, ir_led_in, 0);
  genie.WriteObject(GENIE_OBJ_USER_LED, ir_led_out, 0);
  for (int i = 0; i < 8; i++) {
    genie.WriteObject(GENIE_OBJ_USERIMAGES, images[i], 0);
  }
  for (int i = 0; i < 5; i++) {
    genie.WriteObject(GENIE_OBJ_USER_LED, spray_active_led[i], 0);
    genie.WriteObject(GENIE_OBJ_4DBUTTON, spray_manual[i], 0);
    digitalWrite(spray[i], LOW);
  }
}

void encoder_isr() {
  encoder_value += (digitalRead(ENCODER_A) == digitalRead(ENCODER_B)) ? 1 : -1;
}

void myGenieEventHandler(void) {
  genieFrame Event;
  genie.DequeueEvent(&Event);

  if (Event.reportObject.cmd == GENIE_REPORT_EVENT) {
    if (Event.reportObject.object == GENIE_OBJ_4DBUTTON) {
      // Array of button indices and corresponding wood failure mode values
      const int buttonIndices[6] = { 1, 2, 3, 4 };

      // Iterate through each button index
      for (int i = 0; i < 4; i++) {
        if (Event.reportObject.index == buttonIndices[i]) {

          profile_state = genie.GetEventData(&Event);
          profile_selected = buttonIndices[i];

          if (profile_state == 1) {  // Button is ON

            Serial.println(profile_selected);

            for (int i = 0; i < 5; i++) {
              genie.WriteObject(GENIE_OBJ_USER_LED, spray_selected_led[i], 0);
            }

            switch (profile_selected) {

              case 1:
                for (int i = 0; i < 5; i++) {
                  genie.WriteObject(GENIE_OBJ_USER_LED, spray_selected_led[i], 0);
                }

                for (int i = 0; i < 5; i++) {
                  genie.WriteObject(GENIE_OBJ_USER_LED, spray_selected_led[i], 1);
                }

                break;

              case 2:
                for (int i = 0; i < 5; i++) {
                  genie.WriteObject(GENIE_OBJ_USER_LED, spray_selected_led[i], 0);
                }
                for (int i = 0; i < 4; i++) {
                  genie.WriteObject(GENIE_OBJ_USER_LED, spray_selected_led[i], 1);
                }
                break;

              case 3:
                for (int i = 0; i < 5; i++) {
                  genie.WriteObject(GENIE_OBJ_USER_LED, spray_selected_led[i], 0);
                }

                for (int i = 0; i < 3; i++) {
                  genie.WriteObject(GENIE_OBJ_USER_LED, spray_selected_led[i], 1);
                }
                break;

              case 4:
                for (int i = 0; i < 5; i++) {
                  genie.WriteObject(GENIE_OBJ_USER_LED, spray_selected_led[i], 0);
                }

                for (int i = 0; i < 2; i++) {
                  genie.WriteObject(GENIE_OBJ_USER_LED, spray_selected_led[i], 1);
                }
                break;
            }

            // Turn off other buttons
            for (int j = 0; j < 4; j++) {
              if (j != i) {
                genie.WriteObject(GENIE_OBJ_4DBUTTON, buttonIndices[j], 0);
              }
            }
          }
          break;
        }
      }
    }

    if (Event.reportObject.object == GENIE_OBJ_4DBUTTON) {
      if (Event.reportObject.index == 5) {
        if (profile_state != 0) {
          genie.SetForm(0);
        }
      }
    }

    if (Event.reportObject.object == GENIE_OBJ_4DBUTTON) {

      const int hoaindices[3] = { 7, 8, 9 };

      // Iterate through each button index
      for (int i = 0; i < 3; i++) {
        if (Event.reportObject.index == hoaindices[i]) {

          system_mode_state = genie.GetEventData(&Event);
          system_mode_selected = hoaindices[i];

          if (system_mode_state == 1) {  // Button is ON

            Serial.println(system_mode_selected);

            switch (system_mode_selected) {

              case 7:
                system_mode = 1;
                Serial.println(system_mode);
                genie.WriteStr(0, "    System in manual mode");
                break;

              case 8:
                system_mode = 2;
                Serial.println(system_mode);
                genie.WriteStr(0, "  System in automatic mode");
                reset_hmi_form0();
                break;

              case 9:
                system_mode = 0;
                Serial.println(system_mode);
                genie.WriteStr(0, "  System in standby mode");
                reset_hmi_form0();
                break;
            }

            // Turn off other buttons
            for (int j = 0; j < 3; j++) {
              if (j != i) {
                genie.WriteObject(GENIE_OBJ_4DBUTTON, hoaindices[j], 0);
              }
            }
          } else if (system_mode_state == 0) {
            system_mode = 0;
            genie.WriteObject(GENIE_OBJ_4DBUTTON, 9, 1);
            genie.WriteStr(0, "   System in standby mode");
            reset_hmi_form0();
          }
          break;
        }
      }
    }


    if (Event.reportObject.object == GENIE_OBJ_4DBUTTON) {

      // Iterate through each button index
      for (int i = 0; i < 5; i++) {
        if (Event.reportObject.index == spray_manual[i]) {

          spray_manual_state = genie.GetEventData(&Event);
          spray_manual_selected = spray_manual[i];

          if (spray_manual_state == 1 && system_mode == 1) {  //Button is ON

            Serial.println(spray_manual_selected);

            genie.WriteObject(GENIE_OBJ_USER_LED, spray_active_led[i], 1);
            genie.WriteObject(GENIE_OBJ_USERIMAGES, images[i], 1);
            digitalWrite(spray[i], HIGH);
          } else if (spray_manual_state == 0 && system_mode == 1) {  //Button is ON

            Serial.println(spray_manual_selected);

            genie.WriteObject(GENIE_OBJ_USER_LED, spray_active_led[i], 0);
            genie.WriteObject(GENIE_OBJ_USERIMAGES, images[i], 0);
            digitalWrite(spray[i], LOW);
          }

          else {
            reset_hmi_form0();
          }
        }
      }
    }
  }
}