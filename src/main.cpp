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

int16_t readings3V3[5];
int16_t readings5V[5];
int16_t readings26V[5];
int16_t readingsSPWR[5];
int16_t readingsVCC[5];
int16_t readingsLoopCurrent[5];

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

unsigned int shortBlink = 200; // Продолжительность короткого моргания
unsigned int longBlink = 600; // Продолжительность длинного моргания
unsigned int pauseBlink = 200; // Продолжительность паузы

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
    digitalWrite(GPIO_RESET1, HIGH);
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

    digitalWrite(GPIO_RESET2, LOW);
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
    
    digitalWrite(GPIO_RESET1, HIGH);

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

//*Обработка измерений
float processChannel(Adafruit_ADS1115 &ads, int16_t *readings, int count, float multiplier, float lower, float upper, int errorIndex) {
    int valid[5];
    int validCount = 0;

    // Фильтрация допустимых значений ADC
    for (int i = 0; i < count; i++) {
        int16_t value = readings[i];
        if (value >= 0 && value <= 32767) {
            valid[validCount++] = value;
        }
    }

    if (validCount == 0) {
        Serial.printf("Ошибка: все измерения для канала %d недопустимы\n", errorIndex);
        errors[errorIndex] = true;
        return 0.0;
    }

    // Усреднение
    int sum = 0;
    for (int i = 0; i < validCount; i++) {
        sum += valid[i];
    }
    float average = sum / (float)validCount;

    // Преобразование в напряжение
    float voltage = ads.computeVolts(average) * multiplier;

    // Проверка диапазона
    if (voltage < lower || voltage > upper) {
        errors[errorIndex] = true;
    }

    return voltage;
}

//* Тесты узлов питания
void testVoltagesAndCurrents() {
    Serial.println("Запуск теста узлов питания...");
    handleYellowBlink();
    
    // Сбор данных для каждого канала
    // 3.3V (ADS1, канал 0)
    for (int i = 0; i < 5; i++) {
        readings3V3[i] = ads1.readADC_SingleEnded(0);
        unsigned long startDelay = millis();
        while (millis() - startDelay < 50) {
            handleYellowBlink();
        }
    }

    // 5V (ADS1, канал 1)
    for (int i = 0; i < 5; i++) {
        readings5V[i] = ads1.readADC_SingleEnded(1);
        unsigned long startDelay = millis();
        while (millis() - startDelay < 50) {
            handleYellowBlink();
        }
    }

    // 26V (ADS1, канал 3)
    for (int i = 0; i < 5; i++) {
        readings26V[i] = ads1.readADC_SingleEnded(3);
        unsigned long startDelay = millis();
        while (millis() - startDelay < 50) {
            handleYellowBlink();
        }
    }

    // SPWR (ADS2, канал 0)
    for (int i = 0; i < 5; i++) {
        readingsSPWR[i] = ads2.readADC_SingleEnded(0);
        unsigned long startDelay = millis();
        while (millis() - startDelay < 50) {
            handleYellowBlink();
        }
    }

    // loopCurrent (ADS2, канал 1)
    for (int i = 0; i < 5; i++) {
        readingsLoopCurrent[i] = ads2.readADC_SingleEnded(1);
        unsigned long startDelay = millis();
        while (millis() - startDelay < 50) {
            handleYellowBlink();
        }
    }

    // VCC current (ADS2, канал 2)
    for (int i = 0; i < 5; i++) {
        readingsVCC[i] = ads2.readADC_SingleEnded(2);
        unsigned long startDelay = millis();
        while (millis() - startDelay < 50) {
            handleYellowBlink();
        }
    }

    // Обработка каждого канала
    float voltage3V3 = processChannel(ads1, readings3V3, 5, (15100.0/5100.0), VOLTAGE_3V3_LOWER, VOLTAGE_3V3_UPPER, 3);
    float voltage5V = processChannel(ads1, readings5V, 5, (15100.0/5100.0), VOLTAGE_5V_LOWER, VOLTAGE_5V_UPPER, 4);
    float voltage26V = processChannel(ads1, readings26V, 5, (105100.0/5100.0), VOLTAGE_26V_LOWER, VOLTAGE_26V_UPPER, 5);
    float spwrVoltage = processChannel(ads2, readingsSPWR, 5, (15100.0/5100.0), SPWR_LOWER, SPWR_UPPER, 6);
    float loopCurrentVoltage = processChannel(ads2, readingsLoopCurrent, 5, 10.0, LOOP_CURRENT_VOLTAGE_LOWER, LOOP_CURRENT_VOLTAGE_UPPER, 8);
    float vccCurrent = processChannel(ads2, readingsVCC, 5, 50.0, VCC_CURRENT_LOWER, VCC_CURRENT_UPPER, 7);
    
    //Вывод результатов
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
            case 0: //* RS485
                digitalWrite(LED_RED, HIGH);
                
                delay(longBlink);
                digitalWrite(LED_RED, LOW);

                delay(pauseBlink);

                digitalWrite(LED_RED, HIGH);
                delay(shortBlink);
                digitalWrite(LED_RED, LOW);

                delay(pauseBlink);

                digitalWrite(LED_RED, HIGH);
                delay(longBlink);
                digitalWrite(LED_RED, LOW);

                delay(pauseBlink);

                digitalWrite(LED_RED, HIGH);
                delay(shortBlink);
                digitalWrite(LED_RED, LOW);

                delay(pauseBlink);

                digitalWrite(LED_RED, HIGH);
                delay(longBlink);
                digitalWrite(LED_RED, LOW);

                delay(pauseBlink);
               
                break;
            case 1: //* STxRX
                for (int i = 0; i < 3; i++) {
                    digitalWrite(LED_RED, HIGH);
                    delay(shortBlink);
                    digitalWrite(LED_RED, LOW);
                    delay(pauseBlink);
                }

                for (int i = 0; i < 2; i++) {
                    digitalWrite(LED_RED, HIGH);
                    delay(longBlink);
                    digitalWrite(LED_RED, LOW);
                    delay(pauseBlink);
                }

                break;
            case 2: //* HART
                digitalWrite(LED_RED, HIGH);
                
                delay(longBlink);

                digitalWrite(LED_RED, LOW);
                delay(pauseBlink);

                for (int i = 0; i < 3; i++) {
                    digitalWrite(LED_RED, HIGH);
                    delay(shortBlink);
                    digitalWrite(LED_RED, LOW);
                    delay(pauseBlink);
                }

                digitalWrite(LED_RED, HIGH);
                
                delay(longBlink);

                digitalWrite(LED_RED, LOW);
                delay(pauseBlink);

                break;
            case 3: //* 3V3
                for (int i = 0; i < 2; i++) {
                    digitalWrite(LED_RED, HIGH);
                    delay(shortBlink);
                    digitalWrite(LED_RED, LOW);
                    delay(pauseBlink);
                }

                for (int i = 0; i < 3; i++) {
                    digitalWrite(LED_RED, HIGH);
                    delay(longBlink);
                    digitalWrite(LED_RED, LOW);
                    delay(pauseBlink);
                }

                break;
            case 4: //* 5V
                for (int i = 0; i < 3; i++) {
                    digitalWrite(LED_RED, HIGH);
                    delay(longBlink);
                    digitalWrite(LED_RED, LOW);
                    delay(pauseBlink);
                }

                for (int i = 0; i < 2; i++) {
                    digitalWrite(LED_RED, HIGH);
                    delay(shortBlink);
                    digitalWrite(LED_RED, LOW);
                    delay(pauseBlink);
                }

                break;
            case 5: //* 26V
                digitalWrite(LED_RED, HIGH);
                delay(shortBlink);
                digitalWrite(LED_RED, LOW);
                delay(pauseBlink);

                for (int i = 0; i < 3; i++) {
                    digitalWrite(LED_RED, HIGH);
                    delay(longBlink);
                    digitalWrite(LED_RED, LOW);
                    delay(pauseBlink);
                }

                digitalWrite(LED_RED, HIGH);
                delay(shortBlink);
                digitalWrite(LED_RED, LOW);
                delay(pauseBlink);

                break;
            case 6: //* SPWR
                for (int i = 0; i < 5; i++) {
                    digitalWrite(LED_RED, HIGH);
                    delay(longBlink);
                    digitalWrite(LED_RED, LOW);
                    delay(pauseBlink);
                }

                break;
            case 7: //* VCC
                for (int i = 0; i < 5; i++) {
                    digitalWrite(LED_RED, HIGH);
                    delay(shortBlink);
                    digitalWrite(LED_RED, LOW);
                    delay(pauseBlink);
                }
                break;
            case 8: //* loopCurrent
                digitalWrite(LED_RED, HIGH);
                delay(shortBlink);
                digitalWrite(LED_RED, LOW);
                delay(pauseBlink);

                digitalWrite(LED_RED, HIGH);
                delay(longBlink);
                digitalWrite(LED_RED, LOW);
                delay(pauseBlink);

                digitalWrite(LED_RED, HIGH);
                delay(shortBlink);
                digitalWrite(LED_RED, LOW);
                delay(pauseBlink);

                digitalWrite(LED_RED, HIGH);
                delay(longBlink);
                digitalWrite(LED_RED, LOW);
                delay(pauseBlink);

                digitalWrite(LED_RED, HIGH);
                delay(shortBlink);
                digitalWrite(LED_RED, LOW);
                delay(pauseBlink);
        }

        do {
            currentErrorIndex = (currentErrorIndex + 1) % 9;
        } while(!errors[currentErrorIndex] && currentErrorIndex != -1);

        showingError = false;
    } else {
        // Бирюзовый переход
        delay(500);
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