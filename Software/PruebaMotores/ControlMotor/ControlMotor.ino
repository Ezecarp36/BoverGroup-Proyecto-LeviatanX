#include <ESP32Servo.h>
#include "BluetoothSerial.h"

//BLUETOOTH
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
//CONSTANTES
#define PIN_MOTOR 25
#define NIVEL_MOTOR_PARADO 0
#define NIVEL_MOTOR_BAJO 1000
#define NIVEL_MOTOR_MEDIOBAJO 1300
#define NIVEL_MOTOR_MEDIOALTO 1600
#define NIVEL_MOTOR_ALTO 1800
#define DELAY_GIRO_MOTORES 5000
//VARIABLES
bool sentido = 0;

enum velocidad{
  MOTOR_PARADO,
  MOTOR_VELOCIDAD_BAJA,
  MOTOR_VELOCIDAD_MEDIABAJA,
  MOTOR_VELOCIDAD_MEDIAALTA,
  MOTOR_VELOCIDAD_ALTA
};
int velocidad = MOTOR_PARADO;
//INSTANCIAMOS OBJETOS
BluetoothSerial SerialBT;
Servo esc;  // Objeto del Servo para controlar el ESC

void setup() {
  Serial.begin(115200);     // Inicializa la comunicación serial a 9600 bps
  esc.attach(PIN_MOTOR);  // Inicializa el servo en el pin correspondiente
  SerialBT.begin("ESP32test"); //Bluetooth device name
  delay(3000);
  SerialBT.println("The device started, now you can pair it with bluetooth!");
}

void loop() {
if(SerialBT.available()){
 switch (velocidad){
    case MOTOR_PARADO:{
      esc.writeMicroseconds(NIVEL_MOTOR_PARADO);
      SerialBT.print("VELOCIDAD: ");
      SerialBT.println(velocidad);
      delay(DELAY_GIRO_MOTORES);
      if (sentido == 1)
      {
        esc.writeMicroseconds(NIVEL_MOTOR_PARADO);
        SerialBT.println("VELOCIDAD FIN ");
        break;
      }
      else velocidad = MOTOR_VELOCIDAD_BAJA;
      break;
      }
    case MOTOR_VELOCIDAD_BAJA:{
      esc.writeMicroseconds(NIVEL_MOTOR_BAJO);
      SerialBT.print("VELOCIDAD: ");
      SerialBT.println(velocidad);
      delay(DELAY_GIRO_MOTORES);
      if (sentido == 1) velocidad = MOTOR_PARADO;
      else velocidad = MOTOR_VELOCIDAD_MEDIABAJA;
      break;
      }
    case MOTOR_VELOCIDAD_MEDIABAJA:{
      esc.writeMicroseconds(NIVEL_MOTOR_MEDIOBAJO);
      SerialBT.print("VELOCIDAD: ");
      SerialBT.println(velocidad);
      delay(DELAY_GIRO_MOTORES);
      if(sentido == 1) velocidad = MOTOR_VELOCIDAD_BAJA;
      else velocidad = MOTOR_VELOCIDAD_MEDIAALTA;
      break;
      }
    case MOTOR_VELOCIDAD_MEDIAALTA:{
      esc.writeMicroseconds(NIVEL_MOTOR_MEDIOALTO);
      SerialBT.print("VELOCIDAD: ");
      SerialBT.println(velocidad);
      delay(DELAY_GIRO_MOTORES);
      if(sentido == 1) velocidad = MOTOR_VELOCIDAD_MEDIABAJA;
      else velocidad = MOTOR_VELOCIDAD_ALTA;
      break;
      }
    case MOTOR_VELOCIDAD_ALTA:{
      esc.writeMicroseconds(NIVEL_MOTOR_ALTO);
      SerialBT.print("VELOCIDAD: ");
      SerialBT.println(velocidad);
      delay(DELAY_GIRO_MOTORES);
      sentido = 1;
      if (sentido == 1) velocidad = MOTOR_VELOCIDAD_MEDIAALTA;
      else velocidad = MOTOR_VELOCIDAD_BAJA;
      break;
      }
  }
}
delay(20);
}
