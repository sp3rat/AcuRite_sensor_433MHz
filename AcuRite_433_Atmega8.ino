// Sensor AcuRite mode 00606TX; Atmega8A 8MHz internal, DS18B20; FS1000A 433MHz;

#include <OneWire.h>
#include <DallasTemperature.h>

#define SYNC_LENGTH  9000  //sygnał synchronizacji??
#define SEP_LENGTH   484   //odstęp
#define BIT1_LENGTH  4000  //długość 1 = 4000us
#define BIT0_LENGTH  2000  //długość 0 = 2000us

#define MESSAGE_SIZE 32
#define LSFR_OFFSET 4

const int TX_PIN = 12;      // Pin nadajnika 433 MHz; PINS: Arduino=12; Atmega8=18
const int DS18B20_PIN = 2;  // Pin czujnika DS18B20; PINS: Arduino=2; Atmega8=4

uint8_t LSFR_sequence[MESSAGE_SIZE] = {0};
byte payload[4];

// Inicjalizacja DS18B20
OneWire oneWire(DS18B20_PIN);
DallasTemperature sensors(&oneWire);

void setup() {
    pinMode(TX_PIN, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);

    // Inicjalizacja DS18B20
    sensors.begin();
    
    Serial.println("Starting transmission...");
}

void loop() {
    Serial.println("Reading temperature...");
    if (generatePayload()) {  // Jeśli udało się pobrać temperaturę
        Serial.println("Transmitting...");
        doTransmission();
    }
    Serial.println("Done. Waiting...");
    delay(4000);  // 4 sekund przerwy
}

bool generatePayload() {
    byte rollingCode = 0xAA;
    
    // Odczyt temperatury
    sensors.requestTemperatures();
    float tempC = sensors.getTempCByIndex(0);  // Pobranie temperatury w °C

    // Obsługa błędu odczytu
    if (tempC == DEVICE_DISCONNECTED_C) {
        Serial.println("Error: Could not read temperature data!");
        return false;
    }

    // Konwersja temperatury na wartość całkowitą (temp * 10)
    int temperature = (int)(tempC * 10);

    Serial.print("Temp: ");
    Serial.println(tempC, 1);

    unsigned int highByteTemp = (temperature >> 8) & 0x0F;
    unsigned int lowByteTemp = temperature & 0xFF;

    highByteTemp = 0x80 + highByteTemp;  // Battery OK flag

    payload[0] = rollingCode;
    payload[1] = highByteTemp;
    payload[2] = lowByteTemp;
    payload[3] = computeChecksum(3, payload);

    Serial.print("Payload: ");
    for (int i = 0; i < 4; i++) {
        Serial.print(payload[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    
    return true;  // Sukces
}

void toggleLED() {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
}

void doTransmission() {
    toggleLED();   // burst - ilość powtórzeń
    for (int burst = 1; burst <= 11; burst++) {
        sendPayload();
        sendSync();
    }
}

void sendPayload() {
    for (int i = 0; i < 4; i++) {
        sendByte(payload[i]);
    }
}

void sendByte(byte b) {
    for (int i = 7; i >= 0; i--) {
        sendBit((b >> i) & 1);
    }
}

void sendBit(int val) {
    digitalWrite(TX_PIN, HIGH);
    delayMicroseconds(SEP_LENGTH);

    digitalWrite(TX_PIN, LOW);
    delayMicroseconds(val == 0 ? BIT0_LENGTH : BIT1_LENGTH);
}

void sendSync() {
    digitalWrite(TX_PIN, HIGH);
    delayMicroseconds(SEP_LENGTH);
    digitalWrite(TX_PIN, LOW);
    delayMicroseconds(SYNC_LENGTH);
}

void calculateLSFR() {
    uint8_t reg = 0x7C;
    for (int i = 0; i < MESSAGE_SIZE; i++) {
        uint8_t temp_reg = reg & 0x01;
        reg >>= 1;
        reg |= (temp_reg << 7);
        if (temp_reg) {
            reg ^= 0x18;
        }
        LSFR_sequence[i] = reg;
    }
}

uint8_t combineLSFR(uint8_t len, uint8_t *data) {
    uint8_t hash_reg = 0;
    for (int byte_idx = 0; byte_idx < len; byte_idx++) {
        for (int bit_idx = 7; bit_idx >= 0; bit_idx--) {
            if ((data[byte_idx] >> bit_idx) & 1) {
                hash_reg ^= LSFR_sequence[byte_idx * 8 + (7 - bit_idx) + LSFR_OFFSET];
            }
        }
    }
    return hash_reg;
}

uint8_t computeChecksum(int length, uint8_t *buff) {
    calculateLSFR();
    return combineLSFR(length, buff);
}
