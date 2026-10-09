#include <Arduino.h>
#include "FS.h"
#include "SPIFFS.h"
#include <Preferences.h>

// =========================================================================
// 1. HARDWARE & PIN DEFINITIONS
// =========================================================================
#define LED_PIN       2
#define ENA_PIN       14  
#define IN1_PIN       27  
#define IN2_PIN       26  
#define IN3_PIN       25  
#define IN4_PIN       33  
#define ENB_PIN       32  
#define ENCODER_PIN   18  

const float WHEEL_CIRCUMFERENCE = 3.14159 * 0.066; // Wheel diameter 66mm
const int   TICKS_PER_REV       = 20;              // 20 slots on encoder disc

// =========================================================================
// 2. GLOBALS & TELEMETRY LOGGING
// =========================================================================
struct LogPoint { float error; float dError; int pwm; };
const int MAX_SAMPLES = 200; // 10 seconds at 50ms intervals
LogPoint logBuffer[MAX_SAMPLES];
int sampleIndex = 0;

bool isRunning = false;
Preferences preferences;

float TARGET_SPEED   = 0.50;   
int current_pwm      = 0;    
float prev_error     = 0.0;    
float filtered_speed = 0.0;    

volatile long encoder_ticks = 0;
volatile unsigned long last_interrupt_time = 0;
unsigned long previous_time = 0;
const unsigned long INTERVAL_MS = 50; 

// 1ms (1,000 us) interrupt debounce threshold
void IRAM_ATTR encoderISR() {
    unsigned long interrupt_time = micros();
    if (interrupt_time - last_interrupt_time > 1000) { 
        encoder_ticks++;
        last_interrupt_time = interrupt_time;
    }
}

// =========================================================================
// 3. ANN CONTROLLER
// =========================================================================
int calculate_ann_output(float error_val, float derror_val) {
    float W1[10][2] = {
        {-5.319664, -4.246876},
        {-3.617298, -2.531313},
        { 5.915203,  2.965242},
        {-4.547266, -2.101213},
        {-4.214084, -1.956538},
        {-4.560779, -3.085883},
        { 4.394485,  3.063191},
        {-4.243847, -2.896430},
        { 5.181577,  2.420892},
        {-7.998518, -5.771434}
    };

    float b1[10] = {-8.115760, -1.182491, -6.303299, 1.941213, -1.280264, -3.704172, -3.641466, 0.902836, 5.661152, 11.635712};
    float W2[10] = {-7.352247, -9.225656, 7.758930, -7.016056, -8.174524, -7.969265, 9.983360, -8.213231, 9.354292, -6.054862};
    float output_z_bias = 12.587216;

    float hidden_layer[10];
    for (int i = 0; i < 10; i++) {
        float z1 = (W1[i][0] * error_val) + (W1[i][1] * derror_val) + b1[i];
        hidden_layer[i] = 1.0 / (1.0 + exp(-z1));
    }

    float ann_out = output_z_bias;
    for (int i = 0; i < 10; i++) {
        ann_out += W2[i] * hidden_layer[i];
    }

    return (int)ann_out;
}

void drive_motors(int pwm_value) {
    digitalWrite(IN1_PIN, HIGH); digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, HIGH); digitalWrite(IN4_PIN, LOW);
    analogWrite(ENA_PIN, pwm_value);
    analogWrite(ENB_PIN, pwm_value);
}

void stop_motors() { 
    digitalWrite(IN1_PIN, LOW); digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, LOW); digitalWrite(IN4_PIN, LOW);
    analogWrite(ENA_PIN, 0);
    analogWrite(ENB_PIN, 0);
}

// =========================================================================
// 4. MEMORY & SPIFFS TELEMETRY
// =========================================================================
void saveDataToFlash() {
    File file = SPIFFS.open("/run_log.txt", FILE_WRITE);
    if (file) {
        file.println("Error,dError,PWM");
        for (int i = 0; i < sampleIndex; i++) {
            file.print(logBuffer[i].error, 2); file.print(",");
            file.print(logBuffer[i].dError, 2); file.print(",");
            file.println(logBuffer[i].pwm);
        }
        file.close();
    }
    digitalWrite(LED_PIN, HIGH);
}

void printFlashData() {
    File file = SPIFFS.open("/run_log.txt", FILE_READ);
    if (!file) return;
    Serial.println("\n--- COPY DATA BELOW THIS LINE ---");
    while (file.available()) Serial.write(file.read());
    Serial.println("--- COPY DATA ABOVE THIS LINE ---\n");
    file.close();
}

void startNewRun() {
    sampleIndex = 0; 
    prev_error = 0.0; 
    filtered_speed = 0.0; 
    current_pwm = 0;
    
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, HIGH); delay(200);
        digitalWrite(LED_PIN, LOW); delay(800);
    }
    digitalWrite(LED_PIN, HIGH);
    noInterrupts(); encoder_ticks = 0; interrupts();
    previous_time = millis();
    isRunning = true;
}

// =========================================================================
// 5. SETUP & MAIN LOOP
// =========================================================================
void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    pinMode(IN1_PIN, OUTPUT); pinMode(IN2_PIN, OUTPUT);
    pinMode(IN3_PIN, OUTPUT); pinMode(IN4_PIN, OUTPUT);
    pinMode(ENA_PIN, OUTPUT); pinMode(ENB_PIN, OUTPUT);
    pinMode(ENCODER_PIN, INPUT_PULLUP);
    
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, CHANGE);
    stop_motors();
    
    SPIFFS.begin(true);
    preferences.begin("car_config", false);
    TARGET_SPEED = preferences.getFloat("target", 0.50);

    startNewRun(); 
}

void loop() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n'); 
        input.trim();
        if (input.equalsIgnoreCase("READ")) {
            printFlashData();
        } else if (input.startsWith("SPEED ")) {
            TARGET_SPEED = input.substring(6).toFloat();
            preferences.putFloat("target", TARGET_SPEED);
            Serial.print("Target speed updated: "); 
            Serial.println(TARGET_SPEED);
        }
    }

    if (isRunning) {
        unsigned long current_time = millis();
        unsigned long dt = current_time - previous_time;

        if (dt >= INTERVAL_MS) {
            noInterrupts(); long ticks = encoder_ticks; encoder_ticks = 0; interrupts();

            // Calculate speed from pulses (using CHANGE trigger: 40 transitions per rev)
            float raw_speed = (((float)ticks / (TICKS_PER_REV * 2)) * WHEEL_CIRCUMFERENCE) / (dt / 1000.0);
            filtered_speed = (0.75 * filtered_speed) + (0.25 * raw_speed); 
            
            float e = TARGET_SPEED - filtered_speed;
            float delta_e = e - prev_error;
            prev_error = e;

            int ann_out = calculate_ann_output(e, delta_e);

            if (current_pwm == 0) {
                current_pwm = 140; 
            } else {
                current_pwm += (ann_out / 2); 
            }

            current_pwm = constrain(current_pwm, 0, 255);
            drive_motors(current_pwm);

            if (sampleIndex < MAX_SAMPLES) {
                logBuffer[sampleIndex].error = e; 
                logBuffer[sampleIndex].dError = delta_e; 
                logBuffer[sampleIndex].pwm = current_pwm;
                sampleIndex++;
            }

            previous_time = current_time;

            if (sampleIndex >= MAX_SAMPLES) {
                stop_motors(); 
                isRunning = false; 
                saveDataToFlash();
            }
        }
    }
}