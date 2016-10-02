//Autor: Artur Átila Moraes
//Esse código é utilizado para controle de uma planta didática de um helicóptero com 1 grau de liberdade.

//Declarações referentes ao BLDC
#define KV      17640           //Velocidade máxima em Kv (RPM = Kv x Volts)
#define A1      12              //A1 controla saída Aout1 no driver
#define A2      13              //A2 controla saída Aout2 no driver
#define B1      11              //B1 controla saída Bout1 no driver
#define B2      10              //B2 controla saída Bout2 no driver
#define C1      8               //C1 controla saída Cout1 no driver
#define C2      9               //C2 controla saída Cout2 no driver
#define VALMIN  350             //Intervalo mínimo para troca de estado
#define VALMAX  500             //Intervalo máximo para troca de estado
#define START   5400            //Intervalo mínimo para iniciar o motor

double val;                     //Val modifica período da saída
double rotation = VALMAX;       //Velocidade de rotação
unsigned long startTime = 0;
volatile int aux = 1;

#include "TimerOne.h"

//Declarações referente ao MPU6050
#define ROLLMIN -22             //Menor leitura do sensor
#define ROLLMAX 23              //Maior leitura do sensor
#define XOFFSET 32              //Offset giroscópio X
#define YOFFSET 16              //Offset giroscópio Y
#define ZOFFSET -21             //Offset giroscópio Z
#define AOFFSET 5035            //Offset acelerômetro

double pitch = 0;               //Variável que recebe inclinação do sensor
uint16_t packetSize;            //Expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;             //Count of all bytes currently in FIFO
uint8_t fifoBuffer[64];         //FIFO storage buffer
float ypr[3];                   //[yaw, pitch, roll] yaw/pitch/roll container and gravity vector

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu;
Quaternion q;                   //[w, x, y, z] quaternion container
VectorFloat gravity;            //[x, y, z] gravity vector

//Declarações referentes ao PID
#define KP  1                   //Constante Kp para controle
#define KI  1                   //Constante Ki para controle
#define KD  4                   //Constante Kd para controle
#define SP  525                 //Valor de SetPoint para referência
#define SAMPLE 30               //Tempo de amostragem em ms do PID

double setpoint = SP;

#include <PID_v1.h>
PID myPID(&pitch, &rotation, &setpoint, KP, KI, KD, REVERSE);

void setup() { 
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(B1, OUTPUT);
  pinMode(B2, OUTPUT);
  pinMode(C1, OUTPUT);
  pinMode(C2, OUTPUT);

  Timer1.initialize(START);
  Timer1.attachInterrupt(Turning);

  for(val = START; val > VALMAX*4; val -= 2){
    delayMicroseconds(val);
    Timer1.setPeriod(val);
  }
  for(val = val; val > VALMAX; val --){
    delayMicroseconds(val);
    Timer1.setPeriod(val);
  }
 
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(SAMPLE);
  myPID.SetOutputLimits(VALMIN, VALMAX);
 
  Wire.begin();
  TWBR = 24;
  mpu.initialize();
  mpu.testConnection();
  mpu.dmpInitialize();
  mpu.setXGyroOffset(XOFFSET);
  mpu.setYGyroOffset(YOFFSET);
  mpu.setZGyroOffset(ZOFFSET);
  mpu.setZAccelOffset(AOFFSET);
  mpu.setYAccelOffset(-3064);
  mpu.setZAccelOffset(-303);
  mpu.setDMPEnabled(true);
  packetSize = mpu.dmpGetFIFOPacketSize();

  Serial.begin(115200);
}

void loop() {
  fifoCount = mpu.getFIFOCount();
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  mpu.getFIFOBytes(fifoBuffer, packetSize);
  fifoCount -= packetSize;
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  
  pitch = ypr[2]*180/M_PI;
  Serial.print(pitch);
  pitch = map(pitch, ROLLMIN, ROLLMAX, 0, 1023);
  myPID.Compute();
  Serial.print("\t");

  if( rotation > val) val ++;
  if( rotation < val) val --; 
  Timer1.setPeriod(val); 
  Serial.println(val);
}

void Turning(){
  switch(aux){
    case 1:
      digitalWrite(B1, LOW);                   
      digitalWrite(A1, HIGH);
      aux ++;
      break;
    case 2:
      digitalWrite(C2, LOW);
      digitalWrite(B2, HIGH);
      aux ++;
      break;
    case 3:
      digitalWrite(A1, LOW);
      digitalWrite(C1, HIGH);
      aux ++;
      break;
    case 4:
      digitalWrite(B2, LOW);
      digitalWrite(A2, HIGH);
      aux ++;
      break;
    case 5:
      digitalWrite(C1, LOW);
      digitalWrite(B1, HIGH);
      aux ++;
      break;
    case 6:
      digitalWrite(A2, LOW);
      digitalWrite(C2, HIGH);
      aux = 1;
      break;
  }
}
