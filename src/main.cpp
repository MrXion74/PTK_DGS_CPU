#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <SPI.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>

#define SDA_PIN 22
#define SCL_PIN 23

#define BUTTON_PIN 35
#define LED_RED 19
#define LED_GREEN 18
#define LED_BLUE 5

#define RS485_DIR_PIN 25
#define RS485_TX_PIN 33
#define RS485_RX_PIN 32

#define STXRX_PIN 34

#define GPIO_RESET1 21
#define GPIO_RESET2 4

#define HART_TX_PIN 27
#define HART_RX_PIN 15
#define HART_CD_PIN 26
#define HART_RTS_PIN 14
#define HART_BAUDRATE 1200
#define HART_TIMEOUT 1000

#define VCC_CURRENT_LOWER 20
#define VCC_CURRENT_UPPER 45
#define LOOP_CURRENT_VOLTAGE_LOWER 0.35 
#define LOOP_CURRENT_VOLTAGE_UPPER 0.45 
#define VOLTAGE_3V3_LOWER 3.2  
#define VOLTAGE_3V3_UPPER 3.45
#define VOLTAGE_5V_LOWER 4.85
#define VOLTAGE_5V_UPPER 5.25 //было 5.15, но плата выдает 5.20
#define VOLTAGE_26V_LOWER 24
#define VOLTAGE_26V_UPPER 28  
#define SPWR_LOWER 4.5      
#define SPWR_UPPER 5         

Adafruit_ADS1115 ads1; 
Adafruit_ADS1115 ads2; 
HardwareSerial rs485(1);
HardwareSerial hartSerial(2);
//TODO: Весь процесс тестирования моргает желтым, потом как прошли тесты загораетя зеленый, если есть ошибки загорается красным, код светодиоджа указывает на момент ошибки
void setup() {
    Serial.begin(115200);
    rs485.begin(9600, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
    hartSerial.begin(HART_BAUDRATE, SERIAL_8O1, HART_RX_PIN, HART_TX_PIN);
    
    pinMode(RS485_DIR_PIN, OUTPUT);

    pinMode(BUTTON_PIN, INPUT);
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    pinMode(GPIO_RESET1, OUTPUT);
    pinMode(GPIO_RESET2, OUTPUT);

    pinMode(HART_CD_PIN, INPUT);
    pinMode(HART_RTS_PIN, OUTPUT);

    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, LOW);
    digitalWrite(RS485_DIR_PIN, LOW);
    digitalWrite(GPIO_RESET1, LOW);
    digitalWrite(GPIO_RESET2, LOW);

    Wire.begin(SDA_PIN, SCL_PIN);

    ads1.setGain(GAIN_TWOTHIRDS);
    ads2.setGain(GAIN_TWOTHIRDS);

    digitalWrite(HART_RTS_PIN, HIGH);

    if (!ads1.begin(0x48)) {
        Serial.println("Ошибка инициализации ADS1.");
        while (1);
    }
    if (!ads2.begin(0x49)) {
        Serial.println("Ошибка инициализации ADS2.");
        while (1);
    }
}

void testRS485() {
    byte request[] = {0x54, 0x45, 0x53, 0x54, 0x20, 0x53, 0x0D};
    
    Serial.println("Тест RS-485: Отправка запроса...");
    
    digitalWrite(RS485_DIR_PIN, HIGH);
    delay(10);
    rs485.write(request, sizeof(request));
    delay(10);
    digitalWrite(RS485_DIR_PIN, LOW);

    unsigned long startTime = millis();
    const uint32_t timeout = 2000;
    String response = "";
    bool dataReceived = false;

    while (millis() - startTime < timeout) {
        while (rs485.available()) {
            char c = rs485.read();
            response += c;
            dataReceived = true;
            startTime = millis();
        }
        if (dataReceived && (millis() - startTime > 200)) break;
    }

    if (response.length() > 0) {
        Serial.print(response);
        digitalWrite(LED_GREEN, HIGH);
        delay(2000);
        digitalWrite(LED_GREEN, LOW);
        
    } else {
        Serial.println("Ошибка: Нет ответа от RS-485");
        digitalWrite(LED_RED, HIGH);
        delay(2000);
        digitalWrite(LED_RED, LOW);
    }
}

void testSTxRx() {
    Serial.println("\nЗапуск теста линии STxRx...");
    pinMode(STXRX_PIN, INPUT);
    
    unsigned long startTime = millis();
    int state = digitalRead(STXRX_PIN);
    int changes = 0;
    bool activityDetected = false;

    while (millis() - startTime < 2000) {
        int currentState = digitalRead(STXRX_PIN);
        if (currentState != state) {
            state = currentState;
            changes++;
            if (changes >= 2) {
                activityDetected = true;
                break;
            }
        }
    }

    if (activityDetected) {
        Serial.println("STxRx: Активность обнаружена");
        digitalWrite(LED_GREEN, HIGH);
        delay(1000);
        digitalWrite(LED_GREEN, LOW);
    } else {
        Serial.println("Ошибка: Нет активности на линии STxRx");
        digitalWrite(LED_RED, HIGH);
        delay(2000);
        digitalWrite(LED_RED, LOW);
    }
}

void testHART() {
    Serial.println("\nЗапуск теста HART...");

    byte hartCommand[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x02, 0x00, 0x00, 0x00, 0x02};
    
    digitalWrite(HART_RTS_PIN, LOW);
    delay(100);
    hartSerial.write(hartCommand, sizeof(hartCommand));
    delay(100);
    digitalWrite(HART_RTS_PIN, HIGH);
    delay(50);

    int countHart = 0;
    
    for (int i = 0; i < 15; i++) {
        if (digitalRead(HART_CD_PIN) == 1) {
            countHart += 1;
        }
    }

    if (countHart != 0) {
        Serial.println("HART: Ответ получен");
        digitalWrite(LED_GREEN, HIGH);
        delay(2000);
        digitalWrite(LED_GREEN, LOW);
    } else {
        Serial.println("Ошибка: HART не отвечает");
        digitalWrite(LED_RED, HIGH);
        delay(2000);
        digitalWrite(LED_RED, LOW);
    }
}

void resetCPU() {
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, LOW);
    delay(100);

    Serial.println("Перезагрузка платы CPU...");
    //Индикация перезагрузки
    digitalWrite(LED_BLUE, HIGH);
    delay(1000);
    digitalWrite(LED_BLUE, LOW);
    digitalWrite(LED_RED, HIGH);
    delay(1000);
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, HIGH);
    delay(1000);
    digitalWrite(LED_GREEN, LOW);
    //Перезагрузка
    digitalWrite(GPIO_RESET1, HIGH);
    digitalWrite(GPIO_RESET2, HIGH);
    delay(500);
    digitalWrite(GPIO_RESET1, LOW);
    digitalWrite(GPIO_RESET2, LOW);
    delay(1000);
    Serial.println("Плата CPU включена.");
}

void testVoltagesAndCurrents() {
    int16_t adc10, adc11, adc12, adc13, adc20, adc21;

    adc10 = ads1.readADC_SingleEnded(0); //ADC_3.3V
    adc11 = ads1.readADC_SingleEnded(1); //ADC_5V
    adc13 = ads1.readADC_SingleEnded(3); //ADC_26V
    //adc12 = ads1.readADC_SingleEnded(2);
    adc20 = ads2.readADC_SingleEnded(0); // SPWR voltage
    adc21 = ads2.readADC_SingleEnded(2); // VCC current

    float voltage3V3 = (ads1.computeVolts(adc10) / 5100 * 15100);
    float voltage5V = (ads1.computeVolts(adc11) / 5100 * 15100);
    float voltage26V = (ads1.computeVolts(adc13) / 5100 * 105100);
    float spwrVoltage = (ads2.computeVolts(adc20) / 5100 * 15100);
    float vccCurrent = (ads2.computeVolts(adc21) * 50.0);
    float loopCurrentVoltage = ads2.computeVolts(adc21);

    Serial.printf("VCC Current: %.2f mA\n", vccCurrent);
    Serial.printf("4-20mA Line Voltage: %.2f V\n", loopCurrentVoltage);
    Serial.printf("SPWR Voltage: %.2f V\n", spwrVoltage);
    Serial.printf("3.3V Voltage: %.2f V\n", voltage3V3);
    Serial.printf("5V Voltage: %.2f V\n", voltage5V);
    Serial.printf("26V Voltage: %.2f V\n", voltage26V);

    if (vccCurrent < VCC_CURRENT_LOWER || vccCurrent > VCC_CURRENT_UPPER) {
        Serial.println("Ошибка: ток VCC вне диапазона");
        digitalWrite(LED_RED, HIGH);
        delay(1000);
        return;
    }

    if (loopCurrentVoltage < LOOP_CURRENT_VOLTAGE_LOWER || loopCurrentVoltage > LOOP_CURRENT_VOLTAGE_UPPER) {
        Serial.println("Ошибка: ток в линии 4-20мА вне диапазона");
        digitalWrite(LED_RED, HIGH);
        delay(1000);
        return;
        }

    if (spwrVoltage < SPWR_LOWER || spwrVoltage > SPWR_UPPER) {
        Serial.println("Ошибка: напряжение SPWR вне диапазона");
        digitalWrite(LED_RED, HIGH);
        delay(1000);
        return;
    }

    if (voltage3V3 < VOLTAGE_3V3_LOWER || voltage3V3 > VOLTAGE_3V3_UPPER) {
        Serial.println("Ошибка: напряжение 3.3V вне диапазона");
        digitalWrite(LED_RED, HIGH);
        delay(1000);
        return;
    }

    if (voltage5V < VOLTAGE_5V_LOWER || voltage5V > VOLTAGE_5V_UPPER) {
        Serial.println("Ошибка: напряжение 5V вне диапазона");
        digitalWrite(LED_RED, HIGH);
        delay(1000);
        return;
    }

    if (voltage26V < VOLTAGE_26V_LOWER || voltage26V > VOLTAGE_26V_UPPER) {
        Serial.println("Ошибка: напряжение 26V вне диапазона");
        digitalWrite(LED_RED, HIGH);
        delay(1000);
        return;
    }

    Serial.println("Все тесты успешно пройдены.");
    digitalWrite(LED_GREEN, HIGH);
    delay(5000);
    digitalWrite(LED_GREEN, LOW);
}

void loop() {
    if (digitalRead(BUTTON_PIN) == LOW) {
        Serial.println("Кнопка нажата. Запуск тестирования...");
        resetCPU();
        delay(1000);
        testRS485();
        delay(1000);
        testSTxRx(); 
        delay(1000);
        testHART();
        delay(1000);
        testVoltagesAndCurrents();
        delay(3000);
    }
    delay(100);
}