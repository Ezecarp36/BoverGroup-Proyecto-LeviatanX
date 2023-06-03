#include <RF24.h>
#include <ESP32Servo.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
//Constantes
#define CE_PIN 27
#define CSN_PIN 17
#define ADDRESS 1234
#define PIN_MOTOR_1 32
#define PIN_MOTOR_2 33
#define PIN_MOTOR_3 25
#define PIN_MOTOR_4 26
#define TICK_DEBUG 100
#define VELOCIDAD_ESCALADA 3
#define VELOCIDAD_MAXIMA 1800

//variables
int velocidad = 1000 ;
int velocidad_anterior;
unsigned long currentTime = 0;

//inicializamos objetos
BluetoothSerial SerialBT;
RF24 radio(CE_PIN, CSN_PIN);
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

void setup() {
  //seriales 
  Serial.begin(9600);
  SerialBT.begin();
  //configuracion comunicacion a rf del receptor
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(1, ADDRESS);
  radio.startListening();
  //configuramos motores
  motor1.attach(PIN_MOTOR_1);  // Inicializa el motor brushless en el pin correspondiente
  motor2.attach(PIN_MOTOR_2);
  motor3.attach(PIN_MOTOR_3);
  motor4.attach(PIN_MOTOR_4);
  //nombre bluetooth y mensaje
  SerialBT.begin("LeviatanX"); //Bluetooth device name
  SerialBT.println("Start Bluetooth");
  delay(3000);
}

void loop() {
  
  if (radio.available()) //si hay comunicacion recibe mensaje
  {   
    radio.read(&velocidad, sizeof(velocidad));//recibe mensaje
    
    //configuramos la potencia/velocidad al motor
    motor1.writeMicroseconds(velocidad);
    motor2.writeMicroseconds(velocidad);
    motor3.writeMicroseconds(velocidad);
    motor4.writeMicroseconds(velocidad);

    //imprimimos la velocidad
    if (millis() > currentTime + TICK_DEBUG)
    {
      SerialBT.print("Acelerador recibido: ");
      SerialBT.println(velocidad);
      currentTime = millis();
    }
    
  }
}