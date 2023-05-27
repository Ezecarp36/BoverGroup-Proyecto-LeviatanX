#include <RF24.h>
#include <ESP32Servo.h>

#define CE_PIN 27
#define CSN_PIN 15
#define ADDRESS 1234

#define PIN_MOTOR 27
Servo esc;
int velocidad = 1000;

unsigned long currentTime = 0;
#define TICK_DEBUG 100

RF24 radio(CE_PIN, CSN_PIN);

void setup() {
  Serial.begin(9600);
  esc.attach(PIN_MOTOR);
  Serial.println("StartReceive");
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(1, ADDRESS);
  radio.startListening();
}

void loop() {
  if (radio.available()) 
  {
    radio.read(&velocidad, sizeof(velocidad));
    if (velocidad > 1700) velocidad = 1700;
    esc.writeMicroseconds(velocidad);
    if (millis() > currentTime + TICK_DEBUG)
    {
      Serial.print("Acelerador recibido: ");
      Serial.println(velocidad);
      currentTime = millis();
    }
  }
}