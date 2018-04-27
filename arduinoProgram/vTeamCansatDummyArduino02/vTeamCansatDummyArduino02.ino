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
//VALOR ENVIADO
unsigned int valorEnviado = 1818;
//DHT22
float HumedadDht22 = 0, TemperaturaDht22 = 0;
//ACELEROMETRO
int XRespuestaAcelerometro = 0, YRespuestaAcelerometro = 0, ZRespuestaAcelerometro = 0;
//GIROSCOPIO
int XRespuestaGiroscopio = 0, YRespuestaGiroscopio = 0, ZRespuestaGiroscopio = -1;
//MAGNETOMETRO
int XRespuestaMagnetrometro = 23807, YRespuestaMagnetrometro = 30465, ZRespuestaMagnetrometro = -3075;
//BAROMETRO
float TemperaturaBarometro = 24.10, PresionBarometro = 0;
float contador = 0;
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("+++");
  delay(500);
  Serial.println("B");
  valorEnviado = 0;
}
void loop() {

  obtenerValoresDelBarometro();
  valorEnviado++;
  enviarInformacionValor();
  enviarInformacionAcelerometro();
  enviarInformacionMagnetometro();
  enviarInformacionGiroscopio();
  enviarInformacionDht22();
  enviarInformacionBarometro();
//  delay(71);
  delay(142);
  if (Serial.available()){
    if (Serial.read() == 255){
      valorEnviado = 0;
      contador = 0;
    }
  }
}

void obtenerValoresDelBarometro() {
  HumedadDht22 = sin(contador);
  TemperaturaDht22 = -sin(contador);
  TemperaturaBarometro = -cos(contador);
  PresionBarometro = cos(contador);
//  XRespuestaAcelerometro = sin(contador) * 1;
//  YRespuestaAcelerometro = sin(contador) * 100;
//  ZRespuestaAcelerometro = sin(contador) * 100;
  contador += 0.1;
}


byte auxiliarSerial[5];
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
void enviarInformacionValor() {
  enviarVariableInt(VALOR, (byte*)&valorEnviado);
}
void enviarInformacionGiroscopio() {
  enviarVariableInt(GIROSCOPIOX, (byte*)&XRespuestaGiroscopio);
  enviarVariableInt(GIROSCOPIOY, (byte*)&YRespuestaGiroscopio);
  enviarVariableInt(GIROSCOPIOZ, (byte*)&ZRespuestaGiroscopio);
}
void enviarInformacionMagnetometro() {
  enviarVariableInt(MAGNETOMETROX, (byte*)&XRespuestaMagnetrometro);
  enviarVariableInt(MAGNETOMETROY, (byte*)&YRespuestaMagnetrometro);
  enviarVariableInt(MAGNETOMETROZ, (byte*)&ZRespuestaMagnetrometro);
}
void enviarInformacionAcelerometro() {
  enviarVariableInt(ACELEROMETROX, (byte*)&XRespuestaAcelerometro);
  enviarVariableInt(ACELEROMETROY, (byte*)&YRespuestaAcelerometro);
  enviarVariableInt(ACELEROMETROZ, (byte*)&ZRespuestaAcelerometro);
}
void enviarInformacionDht22() {
  enviarVariableFloat(DHT22T, (byte*)&TemperaturaDht22);
  enviarVariableFloat(DHT22H, (byte*)&HumedadDht22);
}
void enviarInformacionBarometro() {
  enviarVariableFloat(BAROMETROT, (byte*)&TemperaturaBarometro);
  enviarVariableFloat(BAROMETROP, (byte*)&PresionBarometro);
}

