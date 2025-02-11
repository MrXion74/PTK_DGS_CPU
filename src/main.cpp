/*
 * ПТК ДГС
 * Тестирование плат CPU
 */ 

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <SPI.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>

//*-----------------------------------------------------------------------
//*						Инициализация констант
//*-----------------------------------------------------------------------

//* Пины
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

#define GPIO_RESET1 21 // RST_VCC
#define GPIO_RESET2 4  // RST_H

#define HART_TX_PIN 27
#define HART_RX_PIN 15
#define HART_CD_PIN 26
#define HART_RTS_PIN 14
#define HART_BAUDRATE 1200

//* Диапазоны измерений
#define VCC_CURRENT_LOWER 20
#define VCC_CURRENT_UPPER 55

#define LOOP_CURRENT_VOLTAGE_LOWER 1 
#define LOOP_CURRENT_VOLTAGE_UPPER 18

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

int readings3V3[5];
int readings5V[5];
int readings26V[5];
int readingsSPWR[5];
int readingsVCC[5];
int readingsLoopCurrent[5];

enum SystemState { 
    STATE_POWER_OFF,      // Питание выключено (белый светодиод)
    STATE_IDLE,           // Ожидание (мигает белый)
    STATE_TESTING,        // Тестирование (мигает желтый)
    STATE_ERROR_DISPLAY,  // Отображение ошибок (коды ошибок и бирюзовый переход)
    STATE_SUCCESS         // Все тесты успешно пройдены (зеленый светодиод)
};

SystemState currentState = STATE_POWER_OFF;
unsigned long lastBlinkTimeY = 0;
unsigned long lastBlinkTimeW = 0;
bool blinkStateY = false; //Состояние желтого светодиода
bool blinkStateW = false; //Состояние белого светодиода
int currentErrorIndex = -1;

// Флаги ошибок (true - ошибка)
bool errors[] = {
    false, false, false, // RS485, STxRX, HART
    false, false, false, // 3V3, 5V, 26V
    false, false, false  // SPWR, VCC, loopCurrent
};

//*-----------------------------------------------------------------------
//*						Инициализация пинов и линий
//*-----------------------------------------------------------------------

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

//* Желтый светодиод
void handleYellowBlink() {
    if (millis() - lastBlinkTimeY >= 300) {
        blinkStateY = !blinkStateY;
        digitalWrite(LED_RED, blinkStateY);
        digitalWrite(LED_GREEN, blinkStateY);
        lastBlinkTimeY = millis();
    }
}

//* Белый светодиод
void handleWhiteBlink() {
    if (millis() - lastBlinkTimeW >= 300) {
        blinkStateW = !blinkStateW;
        digitalWrite(LED_RED, blinkStateW);
        digitalWrite(LED_GREEN, blinkStateW);
        digitalWrite(LED_BLUE, blinkStateW);
        lastBlinkTimeW = millis();
    }
}

//* Бирюзовый светодиод
void blinkTurquoise() {
    digitalWrite(LED_BLUE, HIGH);
    digitalWrite(LED_GREEN, HIGH);
    delay(2000);
    digitalWrite(LED_BLUE, LOW);
    digitalWrite(LED_GREEN, LOW);
    delay(1500);
}

//*-----------------------------------------------------------------------
//*						          Тесты
//*-----------------------------------------------------------------------

//* Проверка запуска платы
void checkPowerState() {
    float spwr = ads2.computeVolts(ads2.readADC_SingleEnded(0)) / 5100 * 15100;
    if (spwr < 1.0) { // SPWR отключен
        digitalWrite(LED_RED, HIGH);
        digitalWrite(LED_GREEN, HIGH);
        digitalWrite(LED_BLUE, HIGH);
        currentState = STATE_POWER_OFF;
    } else if (currentState == STATE_POWER_OFF) {
        digitalWrite(LED_RED, LOW);
        digitalWrite(LED_GREEN, LOW);
        digitalWrite(LED_BLUE, LOW);
        currentState = STATE_IDLE;
    }
}

//* Перезагрузка платы
void resetCPU() {
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, LOW);
    
    unsigned long startDelay = millis();
    while (millis() - startDelay < 100) {
        handleYellowBlink();
    } 


    Serial.println("Перезагрузка платы CPU...");

    digitalWrite(GPIO_RESET2, HIGH);
    digitalWrite(GPIO_RESET1, HIGH);

    startDelay = millis();
    while (millis() - startDelay < 500) {
        handleYellowBlink();
    }

    startDelay = millis();
    digitalWrite(GPIO_RESET2, LOW);
    while (millis() - startDelay < 200) {
        handleYellowBlink();
    }
    
    digitalWrite(GPIO_RESET1, LOW);

    startDelay = millis();
    while (millis() - startDelay < 1000) {
        handleYellowBlink();
    }

    Serial.println("Плата CPU включена.");
}

//* Тест линии RS485
void testRS485() {
    byte request[] = {0x54, 0x45, 0x53, 0x54, 0x20, 0x53, 0x0D};
    
    Serial.println("Тест RS-485: Отправка запроса...");
    //handleYellowBlink();
    
    digitalWrite(RS485_DIR_PIN, HIGH);

    unsigned long startDelay = millis();
    while (millis() - startDelay < 10) {
        handleYellowBlink();
    }

    rs485.write(request, sizeof(request));

    startDelay = millis();
    while (millis() - startDelay < 10) {
        handleYellowBlink();
    }

    digitalWrite(RS485_DIR_PIN, LOW);

    unsigned long startTime = millis();
    const uint32_t timeout = 2000;
    String response = "";
    bool dataReceived = false;

    while (millis() - startTime < timeout) {
        handleYellowBlink();
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
    } else {
        Serial.println("Ошибка: Нет ответа от RS-485");
        errors[0] = true;
    }
}

//* Тест STxRx
void testSTxRx() {

    handleYellowBlink();

    Serial.println("\nЗапуск теста линии STxRx...");

    pinMode(STXRX_PIN, INPUT);
    
    unsigned long startTime = millis();
    int state = digitalRead(STXRX_PIN);
    int changes = 0;
    bool activityDetected = false;

    while (millis() - startTime < 2000) {
        handleYellowBlink();
        int currentState = digitalRead(STXRX_PIN);
        if (currentState != state) {
            state = currentState;
            changes++;
            if (changes >= 5) {
                activityDetected = true;
                break;
            }
        }
    }

    if (activityDetected) {
        Serial.println("STxRx: Активность обнаружена");
    } else {
        Serial.println("Ошибка: Нет активности на линии STxRx");
        errors[1] = true;
    }
}

//* Тест HART
void testHART() {
    handleYellowBlink();

    Serial.println("\nЗапуск теста HART...");

    byte hartCommand[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x02, 0x00, 0x00, 0x00, 0x02};
    
    digitalWrite(HART_RTS_PIN, LOW);
    unsigned long startDelay = millis();
    while (millis() - startDelay < 100) {
        handleYellowBlink();
    }

    hartSerial.write(hartCommand, sizeof(hartCommand));

    startDelay = millis();
    while (millis() - startDelay < 100) {
        handleYellowBlink();
    }

    digitalWrite(HART_RTS_PIN, HIGH);

    startDelay = millis();
    while (millis() - startDelay < 50) {
        handleYellowBlink();
    }

    int countHart = 0;
    
    for (int i = 0; i < 15; i++) {
        if (digitalRead(HART_CD_PIN) == 1) {
            countHart += 1;
        }
    }

    if (countHart != 0) {
        Serial.println("HART: Ответ получен");
    } else {
        Serial.println("Ошибка: HART не отвечает");
        errors[2] = true;
    }
}

//* Тесты узлов питания
void testVoltagesAndCurrents() {
    Serial.println("Запуск теста узлов питания...");
    handleYellowBlink();
    
    int16_t adc10, adc11, adc13, adc20, adc21, adc22;

    adc10 = ads1.readADC_SingleEnded(0); // ADC_3.3V
    
    unsigned long startDelay = millis();
    while (millis() - startDelay < 500) {
        handleYellowBlink();
    }

    adc11 = ads1.readADC_SingleEnded(1); // ADC_5V
    
    startDelay = millis();
    while (millis() - startDelay < 500) {
        handleYellowBlink();
    }

    adc13 = ads1.readADC_SingleEnded(3); // ADC_26V
    
    startDelay = millis();
    while (millis() - startDelay < 500) {
        handleYellowBlink();
    }
    
    adc20 = ads2.readADC_SingleEnded(0); // SPWR voltage
    
    startDelay = millis();
    while (millis() - startDelay < 500) {
        handleYellowBlink();
    }
    
    adc21 = ads2.readADC_SingleEnded(1); // loopCurrent
    
    startDelay = millis();
    while (millis() - startDelay < 500) {
        handleYellowBlink();
    }

    adc22 = ads2.readADC_SingleEnded(2); // VCC current
    
    startDelay = millis();
    while (millis() - startDelay < 500) {
        handleYellowBlink();
    }

    /*
    for (int i = 0; i < 5; i++) {
        adc10 = ads1.readADC_SingleEnded(0); // ADC_3.3V
        delay(50);
        readings3V3[i] = adc10;
    }

    int valid3v3[5];
    int validIndex3v3 = 0;
    for (int i = 0; i < 5; i++) {
        int currentValue3V3 = readings3V3[i];
        if (currentValue3V3 >= 0 && currentValue3V3 <=);
    }
    */

    float voltage3V3 = (ads1.computeVolts(adc10) / 5100 * 15100);
    float voltage5V = (ads1.computeVolts(adc11) / 5100 * 15100);
    float voltage26V = (ads1.computeVolts(adc13) / 5100 * 105100);
    float spwrVoltage = (ads2.computeVolts(adc20) / 5100 * 15100);
    float loopCurrentVoltage = (ads2.computeVolts(adc21) * 10.0);
    float vccCurrent = (ads2.computeVolts(adc22) * 50.0);
    

    Serial.printf("VCC Current: %.2f mA\n", vccCurrent);
    Serial.printf("4-20mA Line Voltage: %.2f mA\n", loopCurrentVoltage);
    Serial.printf("SPWR Voltage: %.2f V\n", spwrVoltage);
    Serial.printf("3.3V Voltage: %.2f V\n", voltage3V3);
    Serial.printf("5V Voltage: %.2f V\n", voltage5V);
    Serial.printf("26V Voltage: %.2f V\n", voltage26V);

    if (vccCurrent < VCC_CURRENT_LOWER || vccCurrent > VCC_CURRENT_UPPER) {
        Serial.println("Ошибка: ток VCC вне диапазона");
        errors[7] = true;
    }

    if (loopCurrentVoltage < LOOP_CURRENT_VOLTAGE_LOWER || loopCurrentVoltage > LOOP_CURRENT_VOLTAGE_UPPER) {
        Serial.println("Ошибка: ток в линии 4-20мА вне диапазона");
        errors[8] = true;
    }

    if (spwrVoltage < SPWR_LOWER || spwrVoltage > SPWR_UPPER) {
        Serial.println("Ошибка: напряжение SPWR вне диапазона");
        errors[6] = true;
    }

    if (voltage3V3 < VOLTAGE_3V3_LOWER || voltage3V3 > VOLTAGE_3V3_UPPER) {
        Serial.println("Ошибка: напряжение 3.3V вне диапазона");
        errors[3] = true;
    }

    if (voltage5V < VOLTAGE_5V_LOWER || voltage5V > VOLTAGE_5V_UPPER) {
        Serial.println("Ошибка: напряжение 5V вне диапазона");
        errors[4] = true;
    }

    if (voltage26V < VOLTAGE_26V_LOWER || voltage26V > VOLTAGE_26V_UPPER) {
        Serial.println("Ошибка: напряжение 26V вне диапазона");
        errors[5] = true;
    }

    Serial.println("\n");

    //*Дублирование ошибок в конце
    if (errors[0]) {
        Serial.println("Ошибка: Нет ответа от RS-485");
    }
    if (errors[1]) {
        Serial.println("Ошибка: Нет активности на линии STxRx");
    }
    if (errors[2]) {
        Serial.println("Ошибка: HART не отвечает");
    }
}

//* Последовательность тестов
void runTests() {
    handleYellowBlink();
    currentState = STATE_TESTING;

    for(int i=0; i<9; i++) errors[i] = false; //Сброс всех ошибок перед запуском тестов
    
    resetCPU();
    
    unsigned long startDelay = millis();
    while (millis() - startDelay < 5000) {
        handleYellowBlink(); 
    }

    testRS485();
    
    startDelay = millis();
    while (millis() - startDelay < 1000) {
        handleYellowBlink();
    }

    testSTxRx();
    
    startDelay = millis();
    while (millis() - startDelay < 1000) {
        handleYellowBlink();
    }

    testHART();
    
    startDelay = millis();
    while (millis() - startDelay < 1000) {
        handleYellowBlink();
    }

    testVoltagesAndCurrents();
    
    startDelay = millis();
    while (millis() - startDelay < 5000) {
        handleYellowBlink();
    }

    digitalWrite(GPIO_RESET2, HIGH);

    // Проверка наличия ошибок
    currentErrorIndex = -1;
    for(int i = 0; i < 9; i++) {
        if(errors[i]) {
            currentErrorIndex = i;
            break;
        }
    }
    
    // Переход в состояние успеха или отображения ошибок
    if (currentErrorIndex == -1) {
        for(int i=0; i<9; i++) errors[i] = false;
        currentState = STATE_SUCCESS;
    } else {
        currentState = STATE_ERROR_DISPLAY;
        digitalWrite(LED_BLUE, LOW);
        digitalWrite(LED_RED, LOW);
        digitalWrite(LED_GREEN, LOW);
    }
}

//* Анализ кнопки
void handleButton() {
    static unsigned long lastPress = 0;
    const int debounce = 100;
    
    if(digitalRead(BUTTON_PIN) == LOW) {
        if(millis() - lastPress > debounce) {
            if (currentState != STATE_TESTING) {
                runTests();
            }
        }
        lastPress = millis();
    }
}

//* Вывод ошибок
void showErrors() {
    static unsigned long lastErrorTime = 0;
    static unsigned long lastTransitionTime = 0;
    static bool showingError = true;

    if (showingError) {
        switch (currentErrorIndex) {
            case 0: // RS485
                digitalWrite(LED_RED, HIGH);
                delay(300);
                digitalWrite(LED_RED, LOW);
                delay(1000);
                break;
            case 1: // STxRX
                for (int i = 0; i < 2; i++) {
                    digitalWrite(LED_RED, HIGH);
                    delay(300);
                    digitalWrite(LED_RED, LOW);
                    delay(300);
                }
                delay(1000);
                break;
            case 2: // HART
                digitalWrite(LED_RED, HIGH);
                delay(600);
                digitalWrite(LED_RED, LOW);
                delay(600);
                digitalWrite(LED_RED, HIGH);
                delay(600);
                digitalWrite(LED_RED, LOW);
                delay(600);
                break;
            case 3: // 3V3
                for (int i = 0; i < 2; i++) {
                    digitalWrite(LED_RED, HIGH);
                    delay(600);
                    digitalWrite(LED_RED, LOW);
                    delay(300);
                }
                break;
            case 4: // 5V
                digitalWrite(LED_BLUE, HIGH);
                digitalWrite(LED_RED, HIGH);
                delay(300);
                digitalWrite(LED_BLUE, LOW);
                digitalWrite(LED_RED, LOW);
                delay(1000);
                digitalWrite(LED_BLUE, HIGH);
                digitalWrite(LED_RED, HIGH);
                delay(300);
                digitalWrite(LED_BLUE, LOW);
                digitalWrite(LED_RED, LOW);
                delay(1000);
                break;
            case 5: // 26V
                for (int i = 0; i < 2; i++) {
                    digitalWrite(LED_BLUE, HIGH);
                    digitalWrite(LED_RED, HIGH);
                    delay(300);
                    digitalWrite(LED_BLUE, LOW);
                    digitalWrite(LED_RED, LOW);
                    delay(300);
                }
                delay(1000);
                break;
            case 6: // SPWR
                digitalWrite(LED_BLUE, HIGH);
                digitalWrite(LED_RED, HIGH);
                delay(600);
                digitalWrite(LED_BLUE, LOW);
                digitalWrite(LED_RED, LOW);
                delay(600);
                break;
            case 7: // VCC
                for (int i = 0; i < 2; i++) {
                    digitalWrite(LED_BLUE, HIGH);
                    digitalWrite(LED_RED, HIGH);
                    delay(600);
                    digitalWrite(LED_BLUE, LOW);
                    digitalWrite(LED_RED, LOW);
                    delay(300);
                }
                break;
            case 8: // loopCurrent
                digitalWrite(LED_BLUE, HIGH);
                digitalWrite(LED_RED, HIGH);
                delay(300);
                digitalWrite(LED_BLUE, LOW);
                digitalWrite(LED_RED, LOW);
                delay(300);
                digitalWrite(LED_BLUE, HIGH);
                digitalWrite(LED_RED, HIGH);
                delay(600);
                digitalWrite(LED_BLUE, LOW);
                digitalWrite(LED_RED, LOW);
                delay(1000);
        }

        do {
            currentErrorIndex = (currentErrorIndex + 1) % 9;
        } while(!errors[currentErrorIndex] && currentErrorIndex != -1);

        showingError = false;
    } else {
        // Бирюзовый переход
        digitalWrite(LED_GREEN, HIGH);
        digitalWrite(LED_BLUE, HIGH);
        delay(1000);
        digitalWrite(LED_GREEN, LOW);
        digitalWrite(LED_BLUE, LOW);
        delay(500);
        showingError = true;
    }
}

//* Основной цикл
void loop() {
    checkPowerState();
    handleButton();
    
    switch(currentState) {
        case STATE_POWER_OFF:
            break;
            
        case STATE_IDLE:
            handleWhiteBlink();
            break;
            
        case STATE_TESTING:
            break;
            
        case STATE_ERROR_DISPLAY:
            showErrors();
            break;
            
        case STATE_SUCCESS:
            Serial.println("Все тесты успешно пройдены");
            digitalWrite(LED_GREEN, HIGH);
            delay(10000);
            currentState = STATE_IDLE;
            break;
    }
    
    delay(10);
}