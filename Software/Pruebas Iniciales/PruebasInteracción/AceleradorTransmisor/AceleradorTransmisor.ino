#include <RF24.h>

#define CE_PIN  9
#define CSN_PIN 10
#define PIN_ACELERADOR A2
#define ADDRESS 1234
#define TICK_DEBUG 100

unsigned long currentTime = 0;

RF24 radio(CE_PIN, CSN_PIN);

void setup() {
  pinMode(PIN_ACELERADOR, INPUT);
  Serial.begin(9600);
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(ADDRESS);
  Serial.println("StartSend");
  delay(3000);
}

void loop() {
  int lecturaAcelerador = analogRead(PIN_ACELERADOR);
  int lecturaAceleradorMapeado = map(lecturaAcelerador, 0, 1023, 2000, 1000); //Invierto la lectura del acelerador ya que el potenciometro esta al revez
  radio.write(&lecturaAceleradorMapeado, sizeof(lecturaAceleradorMapeado));
  if (millis() > currentTime + TICK_DEBUG)
    {
        Serial.print("Acelerador: ");
        Serial.println(lecturaAceleradorMapeado);
        currentTime = millis();
    }
  
}
