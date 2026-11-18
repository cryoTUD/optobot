#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Initialize the LCD with address 0x27
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Analog pin for AD8495 output
const int tempPin = A0;

// Encoder pins
const int encoderClkPin = 8; // CLK
const int encoderDtPin = 10; // DT
const int encoderSwPin = 11; // SW

// Heater control pin
const int heaterPin = 3;

// PID parameters
float kp = 1.0, ki = 0.1, kd = 0.01;
float set_temperature = -185; // Desired temperature
float PID_error = 0, previous_error = 0;
float PID_p = 0, PID_i = 0, PID_d = 0, PID_value = 0;
int VoltageVal = 0;
float heaterPercentage = 0;
unsigned long Time = 0, timePrev = 0;
float elapsedTime = 0;

// Temperature variable to display
float temperature_read = 0;

// Menu navigation
int menuIndex = 0; // 0: Setpoint, 1: Kp, 2: Ki, 3: Kd, 4: Heater toggle

// Variables for encoder
int lastClkState = HIGH;
int encoderValue = 0;

// Heater control state
bool heaterOn = false;

// Timing variables
unsigned long time1 = 0, lcdUpdateTime = 0;

// Function prototypes
float readThermocouple();

// Interpolation Function
float interpolate(float x, float x_arr[], float y_arr[], int size) {
    if (x <= x_arr[0]) return y_arr[0];
    if (x >= x_arr[size - 1]) return y_arr[size - 1];

    for (int i = 0; i < size - 1; i++) {
        if (x >= x_arr[i] && x <= x_arr[i + 1]) {
            // Linear interpolation formula
            return y_arr[i] + (x - x_arr[i]) * (y_arr[i + 1] - y_arr[i]) / (x_arr[i + 1] - x_arr[i]);
        }
    }
    return 0; // Should never reach here
}

// Ideal Voltage values for temperature
float x_values[] = {-1.3, -1.2, -1.1, -1, -0.9, -0.8, -0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1,
                     0, 0.1, 0.125, 0.2, 0.3, 0.4, 0.5};
// Real voltage values for temperature
float y_values[] = {-0.786, -0.774, -0.751, -0.719, -0.677, -0.627, -0.569, -0.504, -0.432, -0.355,
                    -0.272, -0.184, -0.093, 0.003, 0.1, 0.125, 0.2, 0.301, 0.402, 0.504};

void setup() {
  analogReference(DEFAULT); // Use the default 5V reference

  // Start serial communication
  Serial.begin(9600);

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("PID Control");

  // Setup encoder pins
  pinMode(encoderClkPin, INPUT);
  pinMode(encoderDtPin, INPUT);
  pinMode(encoderSwPin, INPUT_PULLUP); // Enable pull-up for switch

  // Setup heater pin
  pinMode(heaterPin, OUTPUT);
  digitalWrite(heaterPin, LOW); // Ensure heater is off initially

  // Initialize encoder state
  lastClkState = digitalRead(encoderClkPin);
}

// Buffer for averaging temperature readings
const int numReadings = 20;
float tempReadings[numReadings];
int tempIndex = 0;
float tempSum = 0;
float avgTemperature = 0;
unsigned long lastAvgUpdate = 0;
bool initialized = false;

void loop() {
  static int lastButtonState = HIGH;
  static unsigned long lastTempReadTime = 0;
  static float tempSum = 0;
  static int tempCount = 0;

  int currentButtonState = digitalRead(encoderSwPin);

  // Handle menu navigation with encoder switch
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    if (menuIndex == 4) {
      menuIndex = 0; // Exit heater mode
    } else {
      menuIndex = (menuIndex + 1) % 5; // Cycle menus
    }
  }
  lastButtonState = currentButtonState;

  // Read encoder rotation
  int currentClkState = digitalRead(encoderClkPin);
  if (currentClkState != lastClkState) {
    encoderValue = (digitalRead(encoderDtPin) != currentClkState) ? 1 : -1;

    // Adjust parameters based on menu selection
    if (menuIndex == 0) {
      set_temperature = constrain(set_temperature + encoderValue * 0.5, -210, -100);
    } else if (menuIndex == 1) {
      kp = max(0.0, kp + encoderValue * 0.1);
    } else if (menuIndex == 2) {
      ki = max(0.0, ki + encoderValue * 0.01);
    } else if (menuIndex == 3) {
      kd = max(0.0, kd + encoderValue * 0.01);
    } else if (menuIndex == 4 && (encoderValue == 1 || encoderValue == -1)) {
      heaterOn = !heaterOn;
      digitalWrite(heaterPin, heaterOn ? HIGH : LOW);
      VoltageVal = heaterOn ? 255 : 0;
    }
  }
  lastClkState = currentClkState;

  // Read temperature and calculate average every 50ms
  if (millis() - lastTempReadTime > 50) {
    lastTempReadTime = millis();
    temperature_read = readThermocouple();
    tempSum += temperature_read;
    tempCount++;

    if (tempCount >= 5) {  // Average over 250ms (5x 50ms readings)
      avgTemperature = tempSum / tempCount;
      tempSum = 0;
      tempCount = 0;
    }
  }

  // PID Control Logic
  if (heaterOn) {
    PID_error = set_temperature - avgTemperature + 3;
    PID_p = kp * PID_error;
    PID_i += ki * PID_error * 0.01;
    PID_i = constrain(PID_i, -3, 3); // Limit integral windup

    timePrev = Time;
    Time = millis();
    elapsedTime = (Time - timePrev) / 1000.0;
    PID_d = kd * (PID_error - previous_error) / elapsedTime;

    PID_value = constrain(PID_p + PID_i + PID_d, 0, 255);
    VoltageVal = (PID_value > 25) ? 255 : PID_value * 8;
    analogWrite(heaterPin, VoltageVal);
  }

  // Update LCD every 500ms
  if (millis() - lcdUpdateTime > 500) {
    updateDisplay();
    lcdUpdateTime = millis();
  }

  // Print debugging information every 1 second
  if (millis() - time1 > 1000) {
    Serial.print("Avg Temp: ");
    Serial.print(avgTemperature, 1);
    Serial.print(" C, Setpoint: ");
    Serial.print(set_temperature);
    Serial.print(", PID Output: ");
    Serial.print(PID_value);
    heaterPercentage = VoltageVal * 100 / 255;
    Serial.print(", Heater: ");
    Serial.println(heaterOn ? "ON" : "OFF");
    time1 = millis();
  }

  delay(5);
}


// Function to read temperature from AD8495
float readThermocouple() {
  int sensorValue = analogRead(tempPin);
  float voltage = sensorValue * (5.19 / 1023.0);
  float RV = voltage - 1.25; //Real Value to interpolate corresponding ideal value
  float calib_Volt = interpolate(RV, y_values, x_values, 20);
  Serial.print(", sensor: ");
  Serial.print(sensorValue);
  Serial.print(", RV: ");
  Serial.print(RV);
  Serial.print(", Calib_V: ");
  Serial.print(calib_Volt);
  return calib_Volt/0.005; // Convert voltage to temperature
//  return RV/0.005; // Convert voltage to temperature
}

// Function to update the LCD display efficiently
void updateDisplay() {
  lcd.clear();
  lcd.setCursor(0, 0);

  if (menuIndex == 0) {
    lcd.print("Set Temp: ");
    lcd.print(set_temperature, 1);

    lcd.setCursor(0, 1);
    lcd.print("T=");
    lcd.print(avgTemperature, 1);
    lcd.print(" C ");
    lcd.print(heaterPercentage, 0);
    lcd.print("%  "); // Clear extra chars

  } else if (menuIndex == 1) {
    lcd.print("Kp: ");
    lcd.print(kp, 2);
    lcd.print("      "); // Clear extra digits if value decreases

    lcd.setCursor(0, 1);
    lcd.print("         "); // Clear second row

  } else if (menuIndex == 2) {
    lcd.print("Ki: ");
    lcd.print(ki, 2);
    lcd.print("      ");

    lcd.setCursor(0, 1);
    lcd.print("         "); // Clear second row

  } else if (menuIndex == 3) {
    lcd.print("Kd: ");
    lcd.print(kd, 3);
    lcd.print("      ");

    lcd.setCursor(0, 1);
    lcd.print("         "); // Clear second row

  } else if (menuIndex == 4) {
    lcd.print("Heater:");
    lcd.print(heaterOn ? "ON " : "OFF ");
    lcd.print(heaterPercentage, 1);
    lcd.print("%");

    lcd.setCursor(0, 1);
    lcd.print("Temp: ");
    lcd.print(avgTemperature, 1);
    lcd.print(" C"); // Clear extra chars
  }
}
