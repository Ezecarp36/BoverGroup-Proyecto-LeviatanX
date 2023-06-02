#include <RF24.h>

#define CE_PIN  9
#define CSN_PIN 10
#define PIN_ACELERADOR A2
#define ADDRESS 1234
#define TICK_DEBUG 100

unsigned long currentTime = 0;
int muestras = 10;
int sum = 0;

RF24 radio(CE_PIN, CSN_PIN);

int acelerador;
int aceleradorMapeado;
int aceleradorActual;
int aceleradorAnterior;
#define ACELERADOR_UMBRAL 5

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
  
  acelerador = analogRead(PIN_ACELERADOR);

  aceleradorMapeado = map(acelerador, 0, 1023, 2000, 1000); //Invierto la lectura del acelerador ya que el potenciometro esta al revez
  
  aceleradorActual = aceleradorMapeado;
    if (aceleradorActual > aceleradorAnterior + ACELERADOR_UMBRAL || aceleradorActual < aceleradorAnterior - ACELERADOR_UMBRAL) 
      aceleradorActual = aceleradorActual;

    else 
      aceleradorActual = aceleradorAnterior;
    
    aceleradorAnterior = aceleradorActual;

  
  radio.write(&aceleradorActual, sizeof(aceleradorActual));
  if (millis() > currentTime + TICK_DEBUG)
    {
        Serial.print("Acelerador: ");
        Serial.println(aceleradorActual);
        currentTime = millis();
    }
}
