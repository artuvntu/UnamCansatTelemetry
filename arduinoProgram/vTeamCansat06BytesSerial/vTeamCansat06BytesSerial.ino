//LIBRERIAS
#include<Servo.h>
#include "DHT.h"
#include <Wire.h>
//SERIAL
#define ACELEROMETROX 224
#define ACELEROMETROY 225
#define ACELEROMETROZ 226
#define MAGNETOMETROX 228
#define MAGNETOMETROY 229
#define MAGNETOMETROZ 230
#define GIROSCOPIOX 232
#define GIROSCOPIOY 233
#define GIROSCOPIOZ 234
#define DHT22T 244
#define DHT22H 245
#define BAROMETROT 248
#define BAROMETROP 249
#define VALOR 240
//BUZZER
#define BUZZER 10
//BOTONES
#define BOTON 3
//SERVOS
#define PINSERVO1 A0
#define PINSERVO2 A1
#define PINSERVO3 A2
//LEDS
#define LEDR 5
#define LEDV 7
//DHT22
#define TIPO_DHT DHT22
#define PIN_DHT22  A3
//BAROMETRO
#define REGISTRO_BAROMETRO 0x77
#define REGISTRO_BAROMETRO_DIRECCION_LOCAL 0xF4
#define REGISTRO_BAROMETRO_TEMPERATURA_VALOR 0x2E
#define REGISTRO_BAROMETRO_RESPUESTA 0xF6
#define REGISTRO_BAROMETRO_PRESION_VALOR 0x34
#define REGISTRO_BAROMETRO_VALOR_ACalibracionBarometro1 0xAA
#define REGISTRO_BAROMETRO_VALOR_ACalibracionBarometro2 0xAC
#define REGISTRO_BAROMETRO_VALOR_ACalibracionBarometro3 0xAE
#define REGISTRO_BAROMETRO_VALOR_ACalibracionBarometro4 0xB0
#define REGISTRO_BAROMETRO_VALOR_ACalibracionBarometro5 0xB2
#define REGISTRO_BAROMETRO_VALOR_ACalibracionBarometro6 0xB4
#define REGISTRO_BAROMETRO_VALOR_BCalibracionBarometro1 0xB6
#define REGISTRO_BAROMETRO_VALOR_BCalibracionBarometro2 0xB8
#define REGISTRO_BAROMETRO_VALOR_MBCalibracionBarometro 0xBA
#define REGISTRO_BAROMETRO_VALOR_MCCalibracionBarometro 0xBC
#define REGISTRO_BAROMETRO_VALOR_MDCalibracionBarometro 0xBE
//MAGNETOMETRO
#define REGISTRO_MAGNETOMETRO 0x1E
#define MODO_REGISTRO_MAGNETOMETRO 0x02
#define MEDICION_CONTINUA_REGISTRO_MAGNETOMETRO 0X00
#define REGISTRO_MAGNETOMETRO_X 0X03
#define REGISTRO_MAGNETOMETRO_Y 0X05
#define REGISTRO_MAGNETOMETRO_Z 0X07
//ACELEROMETRO
#define REGISTRO_ACELEROMETRO  0x53
#define REGISTRO_DE_CONTROL_ACELEROMETRO 0x2D
#define REGISTRO_ACELEROMETRO_X 0x32
#define REGISTRO_ACELEROMETRO_Y 0x34
#define REGISTRO_ACELEROMETRO_Z 0x36
//GIROSCOPIO
#define REGISTRO_GIROSCOPIO 0x69
#define REGISTRO_DE_CONTROL_GIROSCOPIO1 0x20
#define REGISTRO_DE_CONTROL_GIROSCOPIO2 0x21
#define REGISTRO_DE_CONTROL_GIROSCOPIO3 0x22
#define REGISTRO_DE_CONTROL_GIROSCOPIO4 0x23
#define REGISTRO_DE_CONTROL_GIROSCOPIO5 0x24
#define REGISTRO_DE_CONTROL_GIROSCOPIO_VALOR1 0x0F
#define REGISTRO_DE_CONTROL_GIROSCOPIO_VALOR2 0x00
#define REGISTRO_DE_CONTROL_GIROSCOPIO_VALOR3 0x08
#define REGISTRO_DE_CONTROL_GIROSCOPIO_VALOR4 0x30
#define REGISTRO_DE_CONTROL_GIROSCOPIO_VALOR5 0x00
#define REGISTRO_GIROSCOPIO_X 0x29
#define REGISTRO_GIROSCOPIO_Y 0x2B
#define REGISTRO_GIROSCOPIO_Z 0x2D
//VARIABLES
//SERIAL
byte auxiliarSerial[5];
//SERVO
Servo MotorA, MotorB, MotorC;
//VALOR ENVIADO
unsigned int valorEnviado = 0;
//DHT22
DHT dht(PIN_DHT22, TIPO_DHT);
float HumedadDht22, TemperaturaDht22;
//ACELEROMETRO
int XRespuestaAcelerometro, YRespuestaAcelerometro, ZRespuestaAcelerometro;
//GIROSCOPIO
int XRespuestaGiroscopio, YRespuestaGiroscopio, ZRespuestaGiroscopio;
//MAGNETOMETRO
int XRespuestaMagnetrometro, YRespuestaMagnetrometro, ZRespuestaMagnetrometro;
//BAROMETRO
float TemperaturaBarometro, PresionBarometro;
// VALORES DE CALIBRACION
int ACalibracionBarometro1, ACalibracionBarometro2, ACalibracionBarometro3;
unsigned int ACalibracionBarometro4, ACalibracionBarometro5, ACalibracionBarometro6;
int BCalibracionBarometro1, BCalibracionBarometro2;
int MBCalibracionBarometro, MCCalibracionBarometro, MDCalibracionBarometro;
long BCalibracionBarometro5;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial.println("+++");
  configurarServo();
  configurarLeds();
  configurarDht22();
  configurarGiroscopio();
  configurarAcelerometro();
  configurarMagnetometro();
  configurarBarometro();
  Serial.println("B");
  //PRUEBAS
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDV, HIGH);
  MotorB.write(320);
  Garrabierta();
}
int suma = 0;
bool para = false;
int contadorSoltarse = 0;
void loop() {
  enviarVariableInt(VALOR, (byte*)&valorEnviado);
  valorEnviado++;
  obtenerValoresDelAcelerometro();
  obtenerValoresDelMagnetometro();
  obtenerValoresDelGiroscopio();
  obtenerValoresDelDht22();
  obtenerValoresDelBarometro();
  
  enviarInformacionAcelerometro();
  enviarInformacionMagnetometro();
  enviarInformacionGiroscopio();
  enviarInformacionDht22();
  enviarInformacionBarometro();


  if (valorEnviado == 21){
    Garracerrada();
    digitalWrite(LEDR,LOW);
  }
// 81327 87662
  if (PresionBarometro <= 82359){
    contadorSoltarse++;
  }else{
    contadorSoltarse = 0;
  }
  if(contadorSoltarse >= 21){
    Garrabierta();
    digitalWrite(LEDV,LOW);
  }
  if (para == true){
    suma++;
    if (suma >= 21){
      
    }
  }
  
}

//FUNCIONES
//SERIAL FUNCIONES
void enviarVariableInt(byte tipo, byte * variable) {
  auxiliarSerial[0] = tipo;
  auxiliarSerial[1] = variable[0];
  auxiliarSerial[2] = variable[1];
  Serial.write(auxiliarSerial, 3);
}
void enviarVariableFloat(byte tipo, byte * variable) {
  auxiliarSerial[0] = tipo;
  auxiliarSerial[1] = variable[0];
  auxiliarSerial[2] = variable[1];
  auxiliarSerial[3] = variable[2];
  auxiliarSerial[4] = variable[3];
  Serial.write(auxiliarSerial, 5);
}
//WIRE FUINCIONES
void escribirRegistro(int direccionDispositivo, byte direccionLocal, byte valor) {
  Wire.beginTransmission(direccionDispositivo);
  Wire.write(direccionLocal);
  Wire.write(valor);
  Wire.endTransmission();
}

void leerRegistro(int direccionDispositivo, byte direccionLocal, int * buff) {
  Wire.beginTransmission(direccionDispositivo);
  Wire.write(direccionLocal);
  Wire.write(direccionLocal + 1);
  Wire.endTransmission();
  Wire.requestFrom(direccionDispositivo, 2);
  while (Wire.available() < 2);
  *buff = Wire.read();
  *buff  =  *buff << 8 | Wire.read();
  return;
}
//SERVO
void configurarServo() {
  MotorA.attach(PINSERVO1);
  MotorB.attach(PINSERVO2);
  MotorC.attach(PINSERVO3);
  MotorA.write(100);
}

//LEDS
void configurarLeds() {
  pinMode(LEDR, OUTPUT);
  pinMode(LEDV, OUTPUT);
}
//DHT22
void configurarDht22() {
  dht.begin();
}
void obtenerValoresDelDht22() {
  HumedadDht22 = dht.readHumidity();
  TemperaturaDht22 = dht.readTemperature();
}
void enviarInformacionDht22() {
  enviarVariableFloat(DHT22T, (byte*)&TemperaturaDht22);
  enviarVariableFloat(DHT22H, (byte*)&HumedadDht22);
}
//GIROSCOPIO
void configurarGiroscopio() {
  escribirRegistro(REGISTRO_GIROSCOPIO, REGISTRO_DE_CONTROL_GIROSCOPIO1, REGISTRO_DE_CONTROL_GIROSCOPIO_VALOR1);
  escribirRegistro(REGISTRO_GIROSCOPIO, REGISTRO_DE_CONTROL_GIROSCOPIO2, REGISTRO_DE_CONTROL_GIROSCOPIO_VALOR2);
  escribirRegistro(REGISTRO_GIROSCOPIO, REGISTRO_DE_CONTROL_GIROSCOPIO3, REGISTRO_DE_CONTROL_GIROSCOPIO_VALOR3);
  escribirRegistro(REGISTRO_GIROSCOPIO, REGISTRO_DE_CONTROL_GIROSCOPIO4, REGISTRO_DE_CONTROL_GIROSCOPIO_VALOR4);
  escribirRegistro(REGISTRO_GIROSCOPIO, REGISTRO_DE_CONTROL_GIROSCOPIO5, REGISTRO_DE_CONTROL_GIROSCOPIO_VALOR5);
}
void obtenerValoresDelGiroscopio() {
  leerRegistro(REGISTRO_GIROSCOPIO, REGISTRO_GIROSCOPIO_X, &XRespuestaGiroscopio);
  leerRegistro(REGISTRO_GIROSCOPIO, REGISTRO_GIROSCOPIO_Y, &YRespuestaGiroscopio);
  leerRegistro(REGISTRO_GIROSCOPIO, REGISTRO_GIROSCOPIO_Z, &ZRespuestaGiroscopio);
}
void enviarInformacionGiroscopio() {
  enviarVariableInt(GIROSCOPIOX, (byte*)&XRespuestaGiroscopio);
  enviarVariableInt(GIROSCOPIOY, (byte*)&YRespuestaGiroscopio);
  enviarVariableInt(GIROSCOPIOZ, (byte*)&ZRespuestaGiroscopio);
}
//MAGNETOMETRO

void configurarMagnetometro() {
  escribirRegistro(REGISTRO_MAGNETOMETRO, MODO_REGISTRO_MAGNETOMETRO, MEDICION_CONTINUA_REGISTRO_MAGNETOMETRO);
}
//Y y Z Invertidos
void obtenerValoresDelMagnetometro() {
  leerRegistro(REGISTRO_MAGNETOMETRO, REGISTRO_MAGNETOMETRO_X, &XRespuestaMagnetrometro);
  leerRegistro(REGISTRO_MAGNETOMETRO, REGISTRO_MAGNETOMETRO_Y, &YRespuestaMagnetrometro);
  leerRegistro(REGISTRO_MAGNETOMETRO, REGISTRO_MAGNETOMETRO_Z, &ZRespuestaMagnetrometro);
}
void enviarInformacionMagnetometro() {
  enviarVariableInt(MAGNETOMETROX, (byte*)&XRespuestaMagnetrometro);
  enviarVariableInt(MAGNETOMETROY, (byte*)&YRespuestaMagnetrometro);
  enviarVariableInt(MAGNETOMETROZ, (byte*)&ZRespuestaMagnetrometro);
}
//ACELEROMETRO
void configurarAcelerometro() {
  escribirRegistro(REGISTRO_ACELEROMETRO, REGISTRO_DE_CONTROL_ACELEROMETRO, 0B00001000);
}
void obtenerValoresDelAcelerometro() {
  leerRegistro(REGISTRO_ACELEROMETRO, REGISTRO_ACELEROMETRO_X, &XRespuestaAcelerometro);
  leerRegistro(REGISTRO_ACELEROMETRO, REGISTRO_ACELEROMETRO_Y, &YRespuestaAcelerometro);
  leerRegistro(REGISTRO_ACELEROMETRO, REGISTRO_ACELEROMETRO_Z, &ZRespuestaAcelerometro);
}
void enviarInformacionAcelerometro() {
  enviarVariableInt(ACELEROMETROX, (byte*)&XRespuestaAcelerometro);
  enviarVariableInt(ACELEROMETROY, (byte*)&YRespuestaAcelerometro);
  enviarVariableInt(ACELEROMETROZ, (byte*)&ZRespuestaAcelerometro);
}
//BAROMETRO
void configurarBarometro() {
  leerRegistro(REGISTRO_BAROMETRO, REGISTRO_BAROMETRO_VALOR_ACalibracionBarometro1, &ACalibracionBarometro1);
  leerRegistro(REGISTRO_BAROMETRO, REGISTRO_BAROMETRO_VALOR_ACalibracionBarometro2, &ACalibracionBarometro2);
  leerRegistro(REGISTRO_BAROMETRO, REGISTRO_BAROMETRO_VALOR_ACalibracionBarometro3, &ACalibracionBarometro3);
  leerRegistro(REGISTRO_BAROMETRO, REGISTRO_BAROMETRO_VALOR_ACalibracionBarometro4, &ACalibracionBarometro4);
  leerRegistro(REGISTRO_BAROMETRO, REGISTRO_BAROMETRO_VALOR_ACalibracionBarometro5, &ACalibracionBarometro5);
  leerRegistro(REGISTRO_BAROMETRO, REGISTRO_BAROMETRO_VALOR_ACalibracionBarometro6, &ACalibracionBarometro6);
  leerRegistro(REGISTRO_BAROMETRO, REGISTRO_BAROMETRO_VALOR_BCalibracionBarometro1, &BCalibracionBarometro1);
  leerRegistro(REGISTRO_BAROMETRO, REGISTRO_BAROMETRO_VALOR_BCalibracionBarometro2, &BCalibracionBarometro2);
  leerRegistro(REGISTRO_BAROMETRO, REGISTRO_BAROMETRO_VALOR_MBCalibracionBarometro, &MBCalibracionBarometro);
  leerRegistro(REGISTRO_BAROMETRO, REGISTRO_BAROMETRO_VALOR_MCCalibracionBarometro, &MCCalibracionBarometro);
  leerRegistro(REGISTRO_BAROMETRO, REGISTRO_BAROMETRO_VALOR_MDCalibracionBarometro, &MDCalibracionBarometro);
}
void obtenerValoresDelBarometro() {
  //TEMPERATURA
  unsigned int RespuestaLecturaBarometroTemperatura ;
  long auxiliar1, auxiliar2, auxiliar3, BCalibracionBarometro3, BCalibracionBarometro6, temporal;
  unsigned long BCalibracionBarometro4, BCalibracionBarometro7, RespuestaLecturaBarometroPresion = 0;
  escribirRegistro(REGISTRO_BAROMETRO, REGISTRO_BAROMETRO_DIRECCION_LOCAL, REGISTRO_BAROMETRO_TEMPERATURA_VALOR);
  delay(5);
  leerRegistro(REGISTRO_BAROMETRO, REGISTRO_BAROMETRO_RESPUESTA, (int*)&RespuestaLecturaBarometroTemperatura);
  auxiliar1 = (((long)RespuestaLecturaBarometroTemperatura - (long)ACalibracionBarometro6) * (long)ACalibracionBarometro5) >> 15;
  auxiliar2 = ((long)MCCalibracionBarometro << 11) / (auxiliar1 + MDCalibracionBarometro);
  BCalibracionBarometro5 = auxiliar1 + auxiliar2;
  TemperaturaBarometro = ((BCalibracionBarometro5 + 8) >> 4);
  //PRESION
  escribirRegistro(REGISTRO_BAROMETRO, REGISTRO_BAROMETRO_DIRECCION_LOCAL, REGISTRO_BAROMETRO_PRESION_VALOR);
  delay(5);
  auxiliar1 = 0;
  auxiliar2 = 0;
  leerRegistro(REGISTRO_BAROMETRO, REGISTRO_BAROMETRO_RESPUESTA, (int*)&RespuestaLecturaBarometroPresion);
  BCalibracionBarometro6 = BCalibracionBarometro5 - 4000;
  auxiliar1 = (BCalibracionBarometro2 * (BCalibracionBarometro6 * BCalibracionBarometro6) >> 12) >> 11;
  auxiliar2 = (ACalibracionBarometro2 * BCalibracionBarometro6) >> 11;
  auxiliar3 = auxiliar1 + auxiliar2;
  BCalibracionBarometro3 = ((((long)ACalibracionBarometro1) * 4 + auxiliar3) + 2) >> 2;
  auxiliar1 = (ACalibracionBarometro3 * BCalibracionBarometro6) >> 13;
  auxiliar2 = (BCalibracionBarometro1 * ((BCalibracionBarometro6 * BCalibracionBarometro6) >> 12)) >> 16;
  auxiliar3 = ((auxiliar1 + auxiliar2) + 2) >> 2;
  BCalibracionBarometro4 = (ACalibracionBarometro4 * (unsigned long)(auxiliar3 + 32768)) >> 15;
  BCalibracionBarometro7 = ((unsigned long)(RespuestaLecturaBarometroPresion - BCalibracionBarometro3) * 50000);
  if (BCalibracionBarometro7 < 0x80000000) {
    temporal = (BCalibracionBarometro7 << 1) / BCalibracionBarometro4;
  } else {
    temporal = (BCalibracionBarometro7 / BCalibracionBarometro4) << 1;
  }
  auxiliar1 = (temporal >> 8) * (temporal >> 8);
  auxiliar1 = (auxiliar1 * 3038) >> 16;
  auxiliar2 = (-7357 * temporal) >> 16;
  temporal += (auxiliar1 + auxiliar2 + 3791) >> 4;
  PresionBarometro = temporal;
}
void enviarInformacionBarometro() {
  enviarVariableFloat(BAROMETROT, (byte*)&TemperaturaBarometro);
  enviarVariableFloat(BAROMETROP, (byte*)&PresionBarometro);
}
float calcularAltitud(float pressure)
{
  float A = pressure/101325;
  float B = 1/5.25588;
  float C = pow(A,B);
  C = 1 - C;
  C = C /0.0000225577;

  return C;
}
void Garracerrada() {
MotorA.write(100); 
}

void Garrabierta() {
MotorA.write(10);  
}
