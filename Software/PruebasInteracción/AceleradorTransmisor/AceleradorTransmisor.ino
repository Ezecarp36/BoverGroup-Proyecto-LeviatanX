#include <RF24.h>

#define CE_PIN  9
#define CSN_PIN 10
#define PIN_ACELERADOR A2
#define ADDRESS 1234

RF24 radio(CE_PIN, CSN_PIN);

unsigned long currentTime = 0;
#define TICK_DEBUG 100

void setup() {
  pinMode(PIN_ACELERADOR, INPUT);
  Serial.begin(9600);
  Serial.println("StarSend");
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(ADDRESS);
}

void loop() {
  int acelerador = analogRead(PIN_ACELERADOR);
  int aceleradorMapeado = map(acelerador, 0, 1023, 2000, 1000); //Invierto la lectura del acelerador ya que el potenciometro esta al revez
  radio.write(&aceleradorMapeado, sizeof(aceleradorMapeado));
  if (millis() > currentTime + TICK_DEBUG)
    {
        Serial.print("Acelerador: ");
        Serial.println(aceleradorMapeado);
        currentTime = millis();
    }
  
}
