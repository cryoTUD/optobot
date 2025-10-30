#include <driver/adc.h>
#include <esp_adc_cal.h>

#define AREF 3.3                          // Reference voltage
#define ADC_RESOLUTION 12                 // 12-bit resolution for ESP32
#define SIZE_OF_DATA 12000                  // Reduced size for demonstration purposes

// ADC Pin Configuration for Thermocouple
#define TC_PIN ADC2_CHANNEL_6            // GPIO 14 is ADC2 Channel 6 on ESP32
#define TRIGGER_PIN 2                     // GPIO 2 is D2 on ESP32

// Arrays to store temperature readings and timestamps
float temperatureArray[SIZE_OF_DATA];
unsigned long timeArray[SIZE_OF_DATA];
bool measure = false;

// Variables to store raw ADC value and temperature during waiting time
int raw_value_waiting;
float voltage_waiting;
float temperature_waiting;

// ADC calibration characteristics structure
esp_adc_cal_characteristics_t *adc_chars = new esp_adc_cal_characteristics_t;

// Function to convert raw ADC value to voltage in volts
float get_voltage(int raw_adc) {
    return esp_adc_cal_raw_to_voltage(raw_adc, adc_chars) / 1000.0;  // Convert to volts
}

// Function to convert voltage to temperature using AD8495 amplifier formula
float get_temperature(float voltage) {
    return (voltage - 1.25) / 0.005;  // Conversion formula per AD8495
}

// Read sensor data, storing temperature and time data in the arrays
void read_sensor(float tempArray[], unsigned long timeArray[]) {
    int raw_value;
    for (int i = 0; i < SIZE_OF_DATA; i++) {
        // Capture the timestamp at the beginning of each measurement
        timeArray[i] = micros();
        
        // Read raw ADC value from the thermocouple pin
        adc2_get_raw(TC_PIN, ADC_WIDTH_BIT_12, &raw_value);
        
        // Convert raw value to temperature and store it
        float voltage = get_voltage(raw_value);
        //tempArray[i] = get_temperature(voltage);
        tempArray[i] = voltage;
        
        
       delay(0.01);  // Add a slight delay if needed for sampling rate control
    }
}

// Send the temperature and time arrays data over serial
void send_data(float tempArray[], unsigned long timeArray[], int sample_time) {
    Serial.println("Start of Temperature and Time Data:");
    Serial.print("Sample Time (microseconds): ");
    Serial.println(sample_time);
    for (int i = 0; i < SIZE_OF_DATA; i++) {
        // Serial.print("Time (us): ");
        Serial.print(timeArray[i]);
        Serial.print(", ");
        Serial.println(tempArray[i]);
    }
    Serial.println("End of Data");
}

void setup() {
    Serial.begin(115200);

    // Configure the ADC pin for the ESP32
    adc2_config_channel_atten(TC_PIN, ADC_ATTEN_DB_11);  // Set attenuation for 0-3.3V range
    esp_adc_cal_characterize(ADC_UNIT_2, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, adc_chars);

    // Configure trigger pin as input
    pinMode(TRIGGER_PIN, INPUT);
}

void loop() {
    // Check if the trigger pin is HIGH (5V signal)
    if (digitalRead(TRIGGER_PIN) == HIGH) {
        measure = true;
    }

    if (measure) {
        unsigned long begin_time = micros();

        // Fill temperatureArray and timeArray with sensor readings
        read_sensor(temperatureArray, timeArray);

        unsigned long end_time = micros();
        int sample_time = (end_time - begin_time) / SIZE_OF_DATA;  // Average time per sample

        // Send data over serial
        send_data(temperatureArray, timeArray, sample_time);

        // Stop measurement after one complete set of readings
        measure = false;
        delay(100);  // Optional delay
    }
    else {
        // Read raw ADC value from the thermocouple pin
        adc2_get_raw(TC_PIN, ADC_WIDTH_BIT_12, &raw_value_waiting);
        // Convert raw value to temperature and store it
        voltage_waiting = get_voltage(raw_value_waiting);
        temperature_waiting = get_temperature(voltage_waiting);
        Serial.println("Current Temperature:" + String(temperature_waiting));
        delay(5);  // Add a slight delay for monitoring purposes
    }
    
}
