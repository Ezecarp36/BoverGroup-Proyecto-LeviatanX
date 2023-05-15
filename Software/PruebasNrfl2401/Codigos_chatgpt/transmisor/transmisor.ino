#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN 9
#define CSN_PIN 10

RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";
int texto = 12;

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_LOW);
}

void loop() {
  bool ok = radio.write(&texto, sizeof(texto));
  if (ok){ 
    Serial.print("SE ENVIO: "); 
    Serial.println(texto);
    }
  else{ Serial.println("NO SE ENVIO"); }
  delay(1000);
}
