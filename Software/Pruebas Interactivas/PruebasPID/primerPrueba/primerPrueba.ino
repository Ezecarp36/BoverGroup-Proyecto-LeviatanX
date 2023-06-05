#include "PID.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <RF24.h>
#include <ESP32Servo.h>
#include "BluetoothSerialBT.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// constantes
#define PIN_MOTOR_1 32
#define PIN_MOTOR_2 33
#define PIN_MOTOR_3 25
#define PIN_MOTOR_4 26
#define PIN_INTERRUPT 16
#define PIN_CE 27
#define PIN_CSN 17
#define TICK_DEBUG 100
#define VELOCIDAD_ESCALADA 3
#define VELOCIDAD_MAXIMA 1800
#define TICK_PID_PITCH 10
#define NIVEL_MOTOR_PARADO 0
#define NIVEL_MOTOR_BAJO 1200
#define NIVEL_MOTOR_MEDIOBAJO 1400
#define NIVEL_MOTOR_MEDIOALTO 1600
#define NIVEL_MOTOR_ALTO 1800
#define CALIBRACION_XGYROOFFSET 220
#define CALIBRACION_YGYROOFFSET 76
#define CALIBRACION_ZGYROOFFSET -85
#define CALIBRACION_ZACCELOFFSET 1688

// variables
double input = 0.0;
double setpoint_pitch = 0.0;
double kp_pitch = 0.2, ti_pitch = 0.2, td_pitch = 0.2;
int pitch, yaw, roll;
int velocidad_pitch = 0;
bool estado_boton = false;
int velocidad_pitch_aumenta, velocidad_pitch_disminuye;
int resultadoPidPitch;

enum state
{
    ESPERA,
    INICIO_DE_VUELO,
    PRUEBA_PID
};
int state = ESPERA;

// inicializo objetos
RF24 radio(PIN_CE, PIN_CSN);
BluetoothSerial SerialBT;
MPU6050 mpu;
Pid *calculo_pid_pitch = new Pid(kp_pitch, ti_pitch, td_pitch, setpoint_pitch, TICK_PID_PITCH);
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

// Variables de estado y control MPU
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]
VectorInt16 aa;         // [x, y, z]
VectorInt16 aaReal;     // [x, y, z]
VectorInt16 aaWorld;    // [x, y, z]
VectorFloat gravity;    // [x, y, z]
float ypr[3];           // [yaw, pitch, roll]

// FUNCION QUE CAMBIA A ALTO CUANDO INT RECIBE DATOS DE LA MPU E INTERRUMPE uC
volatile bool mpuInterrupt = false;
void dmpDataReady()
{
    mpuInterrupt = true;
}

void setup()
{
    // inicializar I2C con MPU6050 en biblioteca I2Cdev se utiliza la biblioteca Wire o Fastwire para establecer la comunicación y configurar la frecuencia de reloj del bus I2C.
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif
    // configuracion comunicacion a rf del receptor
    radio.begin();
    radio.setDataRate(RF24_250KBPS);
    radio.openReadingPipe(1, ADDRESS);
    radio.startListening();
    Serial.begin(9600);

    // nombre bluetooth y mensaje
    SerialBT.begin("LeviatanX"); // Bluetooth device name
    SerialBT.println("Start Bluetooth");
    delay(3000);
    // Iniciar MPU6050
    SerialBT.println(F("Initializing I2C devices..."));
    mpu.initialize();
    // configuramos motores
    motor1.attach(PIN_MOTOR_1); // Inicializa el motor brushless en el pin correspondiente
    motor2.attach(PIN_MOTOR_2);
    motor3.attach(PIN_MOTOR_3);
    motor4.attach(PIN_MOTOR_4);
    pinMode(PIN_INTERRUPT, INPUT);
    // Comprobar  conexion
    SerialBT.println(F("Testing device connections..."));
    SerialBT.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    // Iniciar DMP
    SerialBT.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    // Valores de calibracion
    mpu.setXGyroOffset(CALIBRACION_XGYROOFFSET);
    mpu.setYGyroOffset(CALIBRACION_YGYROOFFSET);
    mpu.setZGyroOffset(CALIBRACION_ZGYROOFFSET);
    mpu.setZAccelOffset(CALIBRACION_ZACCELOFFSET);
    // Activar DMP
    if (devStatus == 0)
    {
        SerialBT.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        // Activar interrupcion
        attachInterrupt(digitalPinToInterrupt(PIN_INTERRUPT), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        SerialBT.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        SerialBT.print(F("DMP Initialization failed (code "));
        SerialBT.print(devStatus);
        SerialBT.println(F(")"));
    }
}

void loop()
{
    // Si fallo al iniciar, parar programa
    if (!dmpReady)
        return;

    // Ejecutar mientras no hay interrupcion
    while (!mpuInterrupt && fifoCount < packetSize)
    {                          // AQUI EL RESTO DEL CODIGO DE TU PROGRRAMA
        if (radio.available()) // si hay comunicacion recibe mensaje
        {
            radio.read(&estado_boton, sizeof(estado_boton)); // recibe mensaje

            resultadoPidPitch = calculo_pid_pitch->ComputePid(pitch);
            switch (state)
            {
            case ESPERA:
            {
                velocidad_pitch = NIVEL_MOTOR_PARADO;
                motor1.writeMicroseconds(velocidad_pitch);
                motor2.writeMicroseconds(velocidad_pitch);
                motor3.writeMicroseconds(velocidad_pitch);
                motor4.writeMicroseconds(velocidad_pitch);
                if (digitalRead(estado_boton))
                {
                    state = INICIO_DE_VUELO;
                }
                break;
            }
            case INICIO_DE_VUELO:
            {
                for (int i = 1000; i < 1800; i = i + 25)
                {
                    motor1.writeMicroseconds(i);
                    motor2.writeMicroseconds(i);
                    motor3.writeMicroseconds(i);
                    motor4.writeMicroseconds(i);
                    delay(500);
                    if (i >= 1400)
                    {
                        state = PRUEBA_PID;
                    }
                }
                break;
            }
            case PRUEBA_PID:
            {
                if (pitch > setpoint_pitch)
                {
                    velocidad_pitch_aumenta = velocidad_pitch + resultadoPidPitch;
                    velocidad_pitch_disminuye = velocidad_pitch - resultadoPidPitch;
                    motor1.writeMicroseconds(velocidad_pitch_aumenta);
                    motor2.writeMicroseconds(velocidad_pitch_disminuye);
                    motor3.writeMicroseconds(velocidad_pitch_aumenta);
                    motor4.writeMicroseconds(velocidad_pitch_disminuye);
                }
                else if (pitch < setpoint_pitch)
                {
                    velocidad_pitch_aumenta = velocidad_pitch + resultadoPidPitch;
                    velocidad_pitch_disminuye = velocidad_pitch - resultadoPidPitch;
                    motor1.writeMicroseconds(velocidad_pitch_disminuye);
                    motor2.writeMicroseconds(velocidad_pitch_aumenta);
                    motor3.writeMicroseconds(velocidad_pitch_disminuye);
                    motor4.writeMicroseconds(velocidad_pitch_aumenta);
                }
                else
                {
                    velocidad_pitch = 1400;
                    motor1.writeMicroseconds(velocidad_pitch);
                    motor2.writeMicroseconds(velocidad_pitch);
                    motor3.writeMicroseconds(velocidad_pitch);
                    motor4.writeMicroseconds(velocidad_pitch);
                }
                break;
            }
            }
        }
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // Obtener datos del FIFO
    fifoCount = mpu.getFIFOCount();

    // Controlar overflow
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        mpu.resetFIFO();
        SerialBT.println(F("FIFO overflow!"));
    }
    else if (mpuIntStatus & 0x02)
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize)
            fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // MMostrar Yaw, Pitch, Roll
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        yaw = ypr[0] * 180 / M_PI;
        pitch = ypr[1] * 180 / M_PI;
        roll = ypr[2] * 180 / M_PI;
        SerialBT.print("ypr\t");
        SerialBT.print(yaw);
        SerialBT.print("\t");
        SerialBT.print(pitch);
        SerialBT.print("\t");
        SerialBT.println(roll);

        delay(10);
        // Mostrar aceleracion
        // mpu.dmpGetQuaternion(&q, fifoBuffer);
        // mpu.dmpGetAccel(&aa, fifoBuffer);
        // mpu.dmpGetGravity(&gravity, &q);
        // mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        // SerialBT.print("areal\t");
        // SerialBT.print(aaReal.x);
        // SerialBT.print("\t");
        // SerialBT.print(aaReal.y);
        // SerialBT.print("\t");
        // SerialBT.println(aaReal.z);
    }
}