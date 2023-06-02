#include <RF24.h>
#include <ESP32Servo.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define CE_PIN 27
#define CSN_PIN 17
#define ADDRESS 1234
#define PIN_MOTOR_1 32
#define PIN_MOTOR_2 33
#define PIN_MOTOR_3 25
#define TICK_DEBUG 100
#define VELOCIDAD_ESCALADA 5
#define VELOCIDAD_MAXIMA 2000


int velocidad = 1000 ;
int velocidad_anterior;
unsigned long currentTime = 0;

BluetoothSerial SerialBT;
RF24 radio(CE_PIN, CSN_PIN);
Servo motor1;

void setup() {
  Serial.begin(9600);
  SerialBT.begin();
  motor1.attach(PIN_MOTOR_1);
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(1, ADDRESS);
  radio.startListening();
  SerialBT.begin("ESP32test"); //Bluetooth device name
  SerialBT.println("Start Bluetooth");
  delay(3000);
}

void loop() {
  
  if (radio.available()) 
  {   
    radio.read(&velocidad, sizeof(velocidad));

    int velocidad_actual = velocidad; 
    if (velocidad_actual > velocidad_anterior + VELOCIDAD_ESCALADA ){
      velocidad_anterior = velocidad_actual;
    }
    else if (velocidad_actual > velocidad_anterior - VELOCIDAD_ESCALADA){
      velocidad_anterior = velocidad_actual;
    }
    velocidad_anterior = velocidad_actual;
    if (velocidad_actual > VELOCIDAD_MAXIMA) velocidad_actual = VELOCIDAD_MAXIMA;
    
    motor1.writeMicroseconds(velocidad_actual);

    if (millis() > currentTime + TICK_DEBUG)
    {
      SerialBT.print("Acelerador recibido: ");
      SerialBT.println(velocidad);
      SerialBT.print("Acelerador escalado: ");
      SerialBT.println(velocidad_actual);
      currentTime = millis();
    }
    
  }
}