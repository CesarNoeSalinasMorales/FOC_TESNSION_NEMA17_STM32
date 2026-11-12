#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// =====================================================
// STM32F411 BLACK PILL - NEMA 17
// 2 BTS7960 + AS5600
// Control experimental de posicion con PWM sinosoidal
// =====================================================

// BTS7960 - Bobina A
#define A_RPWM PA0
#define A_LPWM PA1

// BTS7960 - Bobina B
#define B_RPWM PA6
#define B_LPWM PA7

// AS5600
#define AS5600_ADDR 0x36

// =====================================================
// PARAMETROS
// =====================================================

const int PWM_MAXIMO = 100;       // Iniciar con potencia reducida
const int PWM_MINIMO = 40;

const float KP_POSICION = 2.0;
const float CORRECCION_MAX = 30.0;
const float TOLERANCIA = 1.0;


//  50 pares de polos sea correcto para
// el motor y la definicion electrica utilizada.
const float POLE_PAIRS = 50.0;

// Offset del campo. Debe calibrarse experimentalmente.
// No asumir que 180 grados soluciona el problema.
float OFFSET_ELECTRICO = 0.0;

// Convencion de las fases.
// Cambiar solo despues de validar el sentido del campo.
const float PHASE_B_SIGN = -1.0;

// =====================================================
// VARIABLES
// =====================================================

float setpoint = 140.0;
float anguloActual = 0.0;
float error = 0.0;
float correccion = 0.0;
float thetaElectrico = 0.0;

int pwmA = 0;
int pwmB = 0;

bool sensorValido = false;

// =====================================================
// NORMALIZAR ANGULO A 0...360
// =====================================================

float normalizar360(float angulo) {
  while (angulo >= 360.0) angulo -= 360.0;
  while (angulo < 0.0) angulo += 360.0;
  return angulo;
}

// =====================================================
// ERROR ANGULAR MINIMO: -180...180
// =====================================================

float calcularError(float objetivo, float actual) {
  float e = objetivo - actual;

  while (e > 180.0) e -= 360.0;
  while (e < -180.0) e += 360.0;

  return e;
}

// =====================================================
// LEER AS5600
// =====================================================
float leerAS5600() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0C);
  if (Wire.endTransmission(false) != 0) {
    return -1.0;
  }
  if (Wire.requestFrom(AS5600_ADDR, 2) != 2) {
    return -1.0;
  }
  uint8_t highByte = Wire.read();
  uint8_t lowByte = Wire.read();
  uint16_t rawAngle =
      ((uint16_t)(highByte & 0x0F) << 8) | lowByte;

  return rawAngle * (360.0f / 4096.0f);
}

// =====================================================
// DETENER SALIDAS
// =====================================================

void detenerMotor() {
  analogWrite(A_RPWM, 0);
  analogWrite(A_LPWM, 0);
  analogWrite(B_RPWM, 0);
  analogWrite(B_LPWM, 0);

  pwmA = 0;
  pwmB = 0;
}

// =====================================================
// GENERAR CAMPO SINUSOIDAL
// thetaElectrico se expresa en grados electricos
// =====================================================

void generarCampo(float angulo, int amplitudPWM) {
  float rad = normalizar360(angulo) * PI / 180.0f;

  float faseA = sinf(rad);
  float faseB = PHASE_B_SIGN * cosf(rad);

  pwmA = (int)(fabsf(faseA) * amplitudPWM);
  pwmB = (int)(fabsf(faseB) * amplitudPWM);

  if (faseA >= 0.0f) {
    analogWrite(A_RPWM, pwmA);
    analogWrite(A_LPWM, 0);
  } else {
    analogWrite(A_RPWM, 0);
    analogWrite(A_LPWM, pwmA);
  }

  if (faseB >= 0.0f) {
    analogWrite(B_RPWM, pwmB);
    analogWrite(B_LPWM, 0);
  } else {
    analogWrite(B_RPWM, 0);
    analogWrite(B_LPWM, pwmB);
  }
}

// =====================================================
// CONTROL DE POSICION EXPERIMENTAL
// =====================================================

void controlPosicion() {
  float lectura = leerAS5600();

  if (lectura < 0.0f) {
    sensorValido = false;
    detenerMotor();
    return;
  }
  sensorValido = true;
  anguloActual = lectura;
  error = calcularError(setpoint, anguloActual);
  if (fabsf(error) <= TOLERANCIA) {
    detenerMotor();
    correccion = 0.0f;
    return;
  }
  // Accion proporcional limitada
  correccion = KP_POSICION * error;
  if (correccion > CORRECCION_MAX)
    correccion = CORRECCION_MAX;
  if (correccion < -CORRECCION_MAX)
    correccion = -CORRECCION_MAX;
  // Referencia electrica basada en posicion mecanica.
  // La correccion se agrega como un desplazamiento
  // limitado; no se acumula en cada iteracion.
  float anguloBaseElectrico =
      POLE_PAIRS * anguloActual + OFFSET_ELECTRICO;

  thetaElectrico = normalizar360(
      anguloBaseElectrico + correccion
  );
  // PWM proporcional a la magnitud del error.
  int pwm = (int)(fabsf(error) * 2.0f);

  if (pwm < PWM_MINIMO)
    pwm = PWM_MINIMO;

  if (pwm > PWM_MAXIMO)
    pwm = PWM_MAXIMO;

  generarCampo(thetaElectrico, pwm);
}

// =====================================================
// SERIAL
// Enviar un angulo entre 0 y 360 seguido de Enter.
// Otras ordenes:
// P = imprimir estado
// S = detener salidas
// =====================================================
void procesarSerial() {
  if (!Serial.available()) return;
  String entrada = Serial.readStringUntil('\n');
  entrada.trim();
  if (entrada.length() == 0) return;
  if (entrada.equalsIgnoreCase("S")) {
    detenerMotor();
    Serial.println("Salidas PWM detenidas.");
    return;
  }
  if (entrada.equalsIgnoreCase("P")) {
    Serial.print("SP=");
    Serial.print(setpoint, 2);
    Serial.print(", MEC=");
    Serial.print(anguloActual, 2);
    Serial.print(", ERR=");
    Serial.print(error, 2);
    Serial.print(", TH_E=");
    Serial.print(thetaElectrico, 2);
    Serial.print(", PWM_A=");
    Serial.print(pwmA);
    Serial.print(", PWM_B=");
    Serial.println(pwmB);
    return;
  }

  char *fin;
  float nuevoSetpoint = strtof(entrada.c_str(), &fin);

  if (fin != entrada.c_str() && *fin == '\0' &&
      nuevoSetpoint >= 0.0f && nuevoSetpoint <= 360.0f) {
    setpoint = normalizar360(nuevoSetpoint);
    Serial.print("Nuevo setpoint: ");
    Serial.println(setpoint, 2);
  } else {
    Serial.println("Comando invalido. Usa 0...360, P o S.");
  }
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);

  pinMode(A_RPWM, OUTPUT);
  pinMode(A_LPWM, OUTPUT);
  pinMode(B_RPWM, OUTPUT);
  pinMode(B_LPWM, OUTPUT);

  detenerMotor();

  Wire.setSCL(PB8);
  Wire.setSDA(PB9);
  Wire.begin();
  Wire.setClock(400000);

  delay(500);

  float inicial = leerAS5600();

  if (inicial >= 0.0f) {
    anguloActual = inicial;
    sensorValido = true;
  } else {
    sensorValido = false;
    Serial.println("ERROR: no se detecta el AS5600.");
  }

  Serial.println("Control experimental inicializado.");
  Serial.println("Enviar angulo 0...360; P=estado; S=detener.");
}

// =====================================================
// LOOP
// =====================================================

void loop() {
  procesarSerial();

  static unsigned long ultimoControl = 0;
  static unsigned long ultimoReporte = 0;

  unsigned long ahora = millis();

  if (ahora - ultimoControl >= 20) {
    ultimoControl = ahora;
    controlPosicion();
  }

  if (ahora - ultimoReporte >= 100) {
    ultimoReporte = ahora;

    Serial.print(setpoint, 2);
    Serial.print(",");
    Serial.print(anguloActual, 2);
    Serial.print(",");
    Serial.print(error, 2);
    Serial.print(",");
    Serial.print(thetaElectrico, 2);
    Serial.print(",");
    Serial.print(pwmA);
    Serial.print(",");
    Serial.println(pwmB);
   
  }
}
