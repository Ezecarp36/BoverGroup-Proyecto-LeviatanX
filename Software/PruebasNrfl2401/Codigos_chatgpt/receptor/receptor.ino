#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN 27
#define CSN_PIN 15

RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";
int receivedText;

void setup() {
  Serial.begin(115200);
  radio.begin();
  radio.openReadingPipe(1, address);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();
}

void loop() {
  bool ok = radio.available();
  Serial.println(ok);
  if (ok) {
    radio.read(&receivedText, sizeof(receivedText));
    Serial.println(receivedText);
  }
}
