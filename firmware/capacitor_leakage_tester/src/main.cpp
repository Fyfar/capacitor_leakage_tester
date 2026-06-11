/**
                         ┌───────────────────────────────┐
                         │           U3                  │
                         │       RP2040-ZERO             │
                         │  1  GP0                       │
                         │  2  GP1                       │
        ENC_BTN      ───▶│  3  GP2                       │
        EN           ───▶│  4  GP3                       │
                         │  5  GP4                       │
        R_10V        ───▶│  6  GP5                       │
        R_16V        ───▶│  7  GP6                       │
        R_25V        ───▶│  8  GP7                       │
        R_50V        ───▶│  9  GP8                       │
        DISCHARGE_REL.──▶│ 10  GP9                       │
                         │ 11  GP10   ◀── SHUNT_RELAY    │
                         │ 12  GP11   ◀── ENC_A          │
                         │ 13  GP12   ◀── ENC_B          │
                         │ 14  GP13   ◀── ADC_RDY        │
                         │ 15  GP14   ◀── SDA            │
                         │ 16  GP15   ◀── SCL            │
                         │ 17  GP26   ◀── ADC_HV         │
                         │ 18  GP27   ◀── ADC_BATT       │
                         │ 19  GP28                      │
                         │ 20  GP29                      │
                         │ 21  3V3  ◀── +3.3V            │
                         │ 22  GND  ◀── GND              │
                         │ 23  5V   ◀── +5V              │
                         └───────────────────────────────┘

Notes:
- PWR_FLAG connected to +3.3V.
- ENC_A, ENC_B, ENC_BTN = rotary encoder.
- SDA/SCL = I²C bus.
- ADC_HV, ADC_BATT = ADC measurement lines.
- Relays controlled via SHUNT_RELAY and DISCHARGE_RELAY.

 */

#define EB_NO_FOR
#define EB_NO_COUNTER

#define ADC_SMOOTHING 0.5f  // smoothing factor for ADC readings

#define LOG(x) \
    if (Serial) Serial.println(x)

#include <Adafruit_ADS1X15.h>
#include <Arduino.h>
#include <EncButton.h>

#define Wire Wire1
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

EncButtonT<11, 12, 2> enc;
Adafruit_ADS1115 adc;
hd44780_I2Cexp lcd;

constexpr uint8_t LCD_COLUMNS = 16;
constexpr uint8_t LCD_ROWS = 2;

constexpr uint16_t BAT_MEASURE_PERIOD = 5000;     // ms
constexpr uint16_t OUTPUT_MEASURE_PERIOD = 250;   // period between ADC readings in ms
constexpr uint16_t LEAKAGE_MEASURE_PERIOD = 250;  // period between ADC readings in ms
constexpr uint16_t UPDATE_LCD_PERIOD = 250;       // period between LCD updates in ms
constexpr uint16_t ADC_CONVERSION_TIME = 65;      // ADC measurement each ~62.5ms

constexpr float MAX_VOLTAGE_TO_REDUCE_GAIN = 3200.0f;  // mV
constexpr float LOW_VOLTAGE_TO_INCREASE_GAIN = 43.0f;  // mV, ~200uA on low shunt

constexpr uint8_t DC_DC_ENABLE = 3;
constexpr uint8_t R_10V = 5;
constexpr uint8_t R_16V = 6;
constexpr uint8_t R_25V = 7;
constexpr uint8_t R_50V = 8;
constexpr uint8_t DISCHARGE_RELAY_PIN = 9;
constexpr uint8_t SHUNT_RELAY_PIN = 10;
constexpr uint8_t ADC_BATTERY_PIN = 27;
constexpr uint8_t ADC_OUTPUT_PIN = 26;

constexpr uint8_t dividerPins[4] = {R_10V, R_16V, R_25V, R_50V};

constexpr adsGain_t adcGain[5] = {GAIN_ONE, GAIN_TWO, GAIN_FOUR, GAIN_EIGHT, GAIN_SIXTEEN};

constexpr float BAT_DIVIDER = 2.7065f;     // ~5.1k and ~3k resistors
constexpr float OUTPUT_DIVIDER = 17.008f;  // ~75k and ~4.7k resistors 75.03 4.687
constexpr float OPAMP_AMPLIFIER_GAIN = 18.05f;
constexpr float DIOD_LEAKAGE_mV = 0.4f * OPAMP_AMPLIFIER_GAIN;  // mV
constexpr float LOW_SHUNT = 9.980f;                             // Ohm
constexpr float HIGH_SHUNT = 1000.3f;                           // Ohm

constexpr uint16_t UPPER_ADC_GAIN_LIMIT = 30000;
constexpr uint16_t LOWER_ADC_GAIN_LIMIT = UPPER_ADC_GAIN_LIMIT / 2;

uint8_t currentOutputDividerIndex = 0;

float batteryVoltage = 0.0f;
float outputVoltage = 0.0f;
float leakageCurrent = 0.0f;

unsigned long adcOutputTimer = 0;
unsigned long adcLeakageTimer = 0;
unsigned long lcdUpdateTimer = 0;

uint8_t gain = 0;

void encoderTurnISR();
void encoderButtonISR();

void encCallback();

void toggleOutput();
void setOutVoltage(uint8_t dividerPin);
void changeVoltage(int8_t direction);
void dischargeCapacitor(EBAction action);

float measureBattery();
void measureOutput();
void measureLeakage(bool force = false);
void highLeakageShuntEnable();
void lowLeakageShuntEnable();
bool isADCGainTuned(int16_t adcValue);
void updateLcd(bool force = false);

void encoderTurnISR() {
    enc.tickISR();
}

void encoderButtonISR() {
    enc.pressISR();
}

void toggleOutput() {
    digitalRead(DC_DC_ENABLE) == HIGH ? digitalWrite(DC_DC_ENABLE, LOW) : digitalWrite(DC_DC_ENABLE, HIGH);
}

void setOutVoltage(uint8_t dividerPinIndex) {
    digitalWrite(DC_DC_ENABLE, LOW);

    for (byte pin : dividerPins) {
        digitalWrite(pin, LOW);
    }
    digitalWrite(dividerPins[dividerPinIndex], HIGH);
}

void changeVoltage(int8_t direction) {
    currentOutputDividerIndex += direction;
    currentOutputDividerIndex = constrain(currentOutputDividerIndex, 0, 3);

    setOutVoltage(currentOutputDividerIndex);
    updateLcd(true);
}

void dischargeCapacitor(EBAction action) {
    if (action == EBAction::Hold) {
        digitalWrite(DISCHARGE_RELAY_PIN, HIGH);  // Activate discharge relay
        digitalWrite(DC_DC_ENABLE, LOW);          // Disable DC-DC converter while discharging
    } else if (action == EBAction::ReleaseHold) {
        digitalWrite(DISCHARGE_RELAY_PIN, LOW);  // Deactivate discharge relay
    }
}

void encCallback() {
    switch (enc.getAction()) {
        case EBAction::Click:
            LOG("Toggle output");
            toggleOutput();
            break;
        case EBAction::Hold:
            LOG("HOLD ");
            dischargeCapacitor(EBAction::Hold);
            break;
        case EBAction::ReleaseHold:
        case EBAction::ReleaseStep:
            LOG("RELEASE HOLD/STEP");
            dischargeCapacitor(EBAction::ReleaseHold);
            break;
        case EBAction::Turn:
            LOG("Turn");
            changeVoltage(enc.dir());
            break;
        default:
            break;
    }
}

float measureBattery() {
    int raw = analogRead(ADC_BATTERY_PIN);
    return (raw / 4095.0) * 3.3 * BAT_DIVIDER;
}

void measureOutput() {
    unsigned long ms = millis();

    if (ms - adcOutputTimer >= OUTPUT_MEASURE_PERIOD) {
        adcOutputTimer = ms;

        int raw = analogRead(ADC_OUTPUT_PIN);
        outputVoltage = (raw / 4095.0) * 3.3 * OUTPUT_DIVIDER;
    }
}

void highLeakageShuntEnable() {
    digitalWrite(SHUNT_RELAY_PIN, LOW);  // Turn off shunt relay
}

void lowLeakageShuntEnable() {
    digitalWrite(SHUNT_RELAY_PIN, HIGH);  // Activate shunt relay
}

bool isADCGainTuned(int16_t adcValue) {
    int16_t absValue = abs(adcValue);

    if (adcValue >= UPPER_ADC_GAIN_LIMIT && gain > 0) {
        gain--;
        adc.setGain(adcGain[gain]);
        LOG("Reducing ADC gain...");
        LOG(gain);
        return false;
    } else if (adcValue <= LOWER_ADC_GAIN_LIMIT && gain < 4) {
        gain++;
        adc.setGain(adcGain[gain]);
        LOG("Increasing ADC gain...");
        LOG(gain);
        return false;
    }

    return true;
}

void measureLeakage(bool force) {
    static unsigned long nextMeasurement = 0;
    unsigned long ms = millis();

    if (ms < nextMeasurement) return;

    int16_t adcLeakage = adc.readADC_Differential_0_1();
    if (!isADCGainTuned(adcLeakage)) {
        nextMeasurement = ms + ADC_CONVERSION_TIME;
        return;
    }

    bool isHighLeakageShunt = digitalRead(SHUNT_RELAY_PIN) == LOW;
    float voltage = adc.computeVolts(adcLeakage) * 1000.0f;  // in mV

    bool shuntSwithNeeded = false;

    if (voltage >= MAX_VOLTAGE_TO_REDUCE_GAIN && !isHighLeakageShunt) {
        highLeakageShuntEnable();
        shuntSwithNeeded = true;
        gain = 0;
        adc.setGain(adcGain[gain]);
    } else if (voltage <= LOW_VOLTAGE_TO_INCREASE_GAIN && isHighLeakageShunt) {
        lowLeakageShuntEnable();
        shuntSwithNeeded = true;
        gain = 4;
        adc.setGain(adcGain[gain]);
    }

    if (shuntSwithNeeded) {
        nextMeasurement = ms + ADC_CONVERSION_TIME + 15;  // 15 ms relay settle
        return;
    }

    if (voltage < DIOD_LEAKAGE_mV) {
        leakageCurrent = 0.0f;  // compensate diode leakage
    } else {
        float shunt = isHighLeakageShunt ? LOW_SHUNT : HIGH_SHUNT;
        leakageCurrent = (voltage - DIOD_LEAKAGE_mV) / OPAMP_AMPLIFIER_GAIN / shunt;
    }

    nextMeasurement = ms + LEAKAGE_MEASURE_PERIOD;
}

void updateLcd(bool force) {
    unsigned long ms = millis();

    if (force || (ms - lcdUpdateTimer) >= UPDATE_LCD_PERIOD) {
        lcdUpdateTimer = ms;

        lcd.setCursor(0, 0);
        lcd.print("Set:");
        switch (dividerPins[currentOutputDividerIndex]) {
            case R_10V:
                lcd.print("10V");
                break;
            case R_16V:
                lcd.print("16V");
                break;
            case R_25V:
                lcd.print("25V");
                break;
            case R_50V:
                lcd.print("50V");
                break;
            default:
                lcd.print("----");
                break;
        }
        lcd.print("  ");
        lcd.print(outputVoltage, 2);

        lcd.setCursor(0, 1);

        bool uARange = digitalRead(SHUNT_RELAY_PIN) == HIGH;

        lcd.print(leakageCurrent * (uARange ? 1000.0f : 1.0f), 2);
        lcd.print(uARange ? " uA" : " mA");
        lcd.print("  GOOD  ");
    }
}

void setup() {
    Serial.begin(115200);

    unsigned long startSerial = millis();
    while (!Serial && (millis() - startSerial < 2000)) {
        delay(10);
    }

    Wire1.setSDA(14);
    Wire1.setSCL(15);

    pinMode(DC_DC_ENABLE, OUTPUT);         // DC-DC enable
    pinMode(R_10V, OUTPUT);                // R_10V
    pinMode(R_16V, OUTPUT);                // R_16V
    pinMode(R_25V, OUTPUT);                // R_25V
    pinMode(R_50V, OUTPUT);                // R_50V
    pinMode(DISCHARGE_RELAY_PIN, OUTPUT);  // DISCHARGE_RELAY
    pinMode(SHUNT_RELAY_PIN, OUTPUT);      // SHUNT_RELAY

    pinMode(ADC_OUTPUT_PIN, INPUT);   // ADC output
    pinMode(ADC_BATTERY_PIN, INPUT);  // ADC battery
    analogReadResolution(12);         // 12-bit ADC resolution

    setOutVoltage(currentOutputDividerIndex);
    digitalWrite(DC_DC_ENABLE, LOW);  // Disable DC-DC converter
    highLeakageShuntEnable();

    attachInterrupt(digitalPinToInterrupt(11), encoderTurnISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(12), encoderTurnISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(2), encoderButtonISR, FALLING);
    enc.setEncISR(true);
    enc.attach(encCallback);
    enc.setEncType(EB_STEP4_LOW);
    enc.setBtnLevel(LOW);

    lcd.begin(LCD_COLUMNS, LCD_ROWS);
    lcd.noLineWrap();
    lcd.clear();

    adc.begin(ADS1X15_ADDRESS, &Wire1);
    adc.setDataRate(RATE_ADS1115_16SPS);
    adc.setGain(adcGain[gain]);

    lcd.setCursor(0, 0);
    float vbat = measureBattery();
    lcd.print("Battery: ");
    lcd.print(vbat, 2);
    lcd.setCursor(0, 1);
    lcd.print("Leakage tester");
    delay(3000);

    lcd.clear();

    LOG("Setup complete");
}

void loop() {
    enc.tick();

    measureOutput();
    measureLeakage();

    updateLcd();
}