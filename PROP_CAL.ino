
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

/* =========================================================
   STM32F411 BLACK PILL
   NEMA17 17HS8501
   50 pares de polos
   2 x BTS7960
   AS5600

   CONTROL DE POSICION + FOC DE TENSION
   Correccion: inversion del sentido del componente Vq

   BOBINA A:
      PA0 -> RPWM
      PA1 -> LPWM

   BOBINA B:
      PA6 -> RPWM
      PA7 -> LPWM

   AS5600:
      PB8 -> SCL
      PB9 -> SDA

   COMANDOS:
      C       Calibrar
      A       Mostrar estado
      F       Activar FOC
      S       Detener FOC
      +       Aumentar potencia de calibracion
      -       Disminuir potencia de calibracion
      0-360   Cambiar setpoint mecanico
   ========================================================= */


/* ======================== PINES ======================== */

#define A_RPWM PA0
#define A_LPWM PA1

#define B_RPWM PA6
#define B_LPWM PA7


/* ======================== AS5600 ======================= */

#define AS5600_ADDR     0x36
#define AS5600_ANGLE_H  0x0C


/* ======================== MOTOR ======================== */

#define POLE_PAIRS 50.0f


/* ========================= PWM ========================= */

#define PWM_MAX        255
#define PWM_FREQUENCY  20000


/* =================== CONTROL DE POSICION ================ */

float Kp = 0.020f;

float VQ_MINIMO = 0.25f;
float VQ_MAX = 0.80f;
float VD = 0.0f;

float TOLERANCIA = 1.0f;


/* ======================== SETPOINT ====================== */

float setpointObjetivo = 140.0f;
float setpoint = 140.0f;

float SETPOINT_RAMP_SPEED = 20.0f;

unsigned long ultimoUpdateRampa = 0;


/* ======================== VARIABLES ===================== */

float anguloActual = 0.0f;
float error = 0.0f;

float thetaElectrical = 0.0f;

float electricalSign = 0.0f;
float electricalOffset = 0.0f;

bool calibrated = false;
bool focActivo = false;

float Vq = 0.0f;

float phaseACommand = 0.0f;
float phaseBCommand = 0.0f;

int pwmA = 0;
int pwmB = 0;


/* ====================== CALIBRACION ===================== */

#define ALIGN_ANGLE_E       45.0f

float testPower = 0.25f;

#define POWER_STEP          0.05f
#define POWER_MIN           0.05f
#define POWER_MAX           0.35f

#define ALIGN_TIME_MS       1200
#define DIRECTION_TEST_MS   900
#define OFFSET_SAMPLES      100


/* ====================== ANGULOS ========================= */

float wrap360(float angle)
{
  while (angle >= 360.0f) angle -= 360.0f;
  while (angle < 0.0f) angle += 360.0f;
  return angle;
}


// Devuelve a-b en el intervalo [-180, 180].
// En el caso exacto de 180 grados, elige -180.
float angleDiff(float a, float b)
{
  float d = a - b;

  while (d > 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;

  return d;
}


/* ======================= RAMPA ========================== */

void actualizarSetpointRampa()
{
  unsigned long ahora = millis();

  if (ahora - ultimoUpdateRampa < 10) return;

  float dt = (ahora - ultimoUpdateRampa) / 1000.0f;
  ultimoUpdateRampa = ahora;

  float delta = angleDiff(setpointObjetivo, setpoint);

  if (fabsf(delta) <= 0.01f)
  {
    setpoint = setpointObjetivo;
    return;
  }

  float paso = SETPOINT_RAMP_SPEED * dt;

  if (fabsf(delta) <= paso)
  {
    setpoint = setpointObjetivo;
    return;
  }

  if (delta > 0.0f)
    setpoint += paso;
  else
    setpoint -= paso;

  setpoint = wrap360(setpoint);
}


/* ======================== AS5600 ======================== */

uint16_t leerAS5600Raw()
{
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_ANGLE_H);

  if (Wire.endTransmission(false) != 0)
    return 0xFFFF;

  if (Wire.requestFrom(AS5600_ADDR, 2) != 2)
    return 0xFFFF;

  uint8_t highByte = Wire.read();
  uint8_t lowByte = Wire.read();

  uint16_t raw = ((uint16_t)highByte << 8) | lowByte;
  raw &= 0x0FFF;

  return raw;
}


float leerAS5600()
{
  uint16_t raw = leerAS5600Raw();

  if (raw == 0xFFFF)
    return NAN;

  return ((float)raw * 360.0f) / 4096.0f;
}


float circularMeanDegrees(int samples)
{
  float sumSin = 0.0f;
  float sumCos = 0.0f;
  int valid = 0;

  for (int i = 0; i < samples; i++)
  {
    float angle = leerAS5600();

    if (!isnan(angle))
    {
      float rad = angle * PI / 180.0f;
      sumSin += sinf(rad);
      sumCos += cosf(rad);
      valid++;
    }

    delay(5);
  }

  if (valid == 0)
    return NAN;

  float meanRad = atan2f(sumSin, sumCos);

  return wrap360(meanRad * 180.0f / PI);
}


/* ======================== FASE A ======================== */

void setPhaseA_FOC(float value)
{
  value = constrain(value, -1.0f, 1.0f);

  int pwm = (int)(fabsf(value) * PWM_MAX);
  pwm = constrain(pwm, 0, PWM_MAX);

  if (value > 0.0f)
  {
    analogWrite(A_LPWM, 0);
    analogWrite(A_RPWM, pwm);
  }
  else if (value < 0.0f)
  {
    analogWrite(A_RPWM, 0);
    analogWrite(A_LPWM, pwm);
  }
  else
  {
    analogWrite(A_RPWM, 0);
    analogWrite(A_LPWM, 0);
  }

  pwmA = pwm;
}


/* ======================== FASE B ======================== */

void setPhaseB_FOC(float value)
{
  value = constrain(value, -1.0f, 1.0f);

  int pwm = (int)(fabsf(value) * PWM_MAX);
  pwm = constrain(pwm, 0, PWM_MAX);

  if (value > 0.0f)
  {
    analogWrite(B_LPWM, 0);
    analogWrite(B_RPWM, pwm);
  }
  else if (value < 0.0f)
  {
    analogWrite(B_RPWM, 0);
    analogWrite(B_LPWM, pwm);
  }
  else
  {
    analogWrite(B_RPWM, 0);
    analogWrite(B_LPWM, 0);
  }

  pwmB = pwm;
}


/* ==================== FASES DE CALIBRACION =============== */

void setPhaseA_Calibration(float value)
{
  value = constrain(value, -1.0f, 1.0f);
  int pwm = (int)(fabsf(value) * PWM_MAX);

  if (value > 0.0f)
  {
    analogWrite(A_LPWM, 0);
    analogWrite(A_RPWM, pwm);
  }
  else if (value < 0.0f)
  {
    analogWrite(A_RPWM, 0);
    analogWrite(A_LPWM, pwm);
  }
  else
  {
    analogWrite(A_RPWM, 0);
    analogWrite(A_LPWM, 0);
  }
}


void setPhaseB_Calibration(float value)
{
  value = constrain(value, -1.0f, 1.0f);
  int pwm = (int)(fabsf(value) * PWM_MAX);

  if (value > 0.0f)
  {
    analogWrite(B_LPWM, 0);
    analogWrite(B_RPWM, pwm);
  }
  else if (value < 0.0f)
  {
    analogWrite(B_RPWM, 0);
    analogWrite(B_LPWM, pwm);
  }
  else
  {
    analogWrite(B_RPWM, 0);
    analogWrite(B_LPWM, 0);
  }
}


/* ====================== DETENER MOTOR =================== */

void detenerMotor()
{
  analogWrite(A_RPWM, 0);
  analogWrite(A_LPWM, 0);
  analogWrite(B_RPWM, 0);
  analogWrite(B_LPWM, 0);

  pwmA = 0;
  pwmB = 0;

  phaseACommand = 0.0f;
  phaseBCommand = 0.0f;

  Vq = 0.0f;
}


/* ================= VECTOR DE CALIBRACION ================ */

// Convencion usada para calibrar:
// A = cos(thetaE)
// B = -sin(thetaE)

void setElectricalVectorCalibration(float thetaE, float amplitude)
{
  float rad = wrap360(thetaE) * PI / 180.0f;

  float phaseA = amplitude * cosf(rad);
  float phaseB = -amplitude * sinf(rad);

  setPhaseA_Calibration(phaseA);
  setPhaseB_Calibration(phaseB);
}


/* ====================== CALIBRACION ===================== */

void calibrarSensor()
{
  focActivo = false;
  detenerMotor();

  calibrated = false;
  electricalSign = 0.0f;
  electricalOffset = 0.0f;

  Serial.println();
  Serial.println("========================================");
  Serial.println(" CALIBRACION DE ANGULO ELECTRICO");
  Serial.println("========================================");

  Serial.print("Pares de polos = ");
  Serial.println(POLE_PAIRS, 1);

  Serial.print("Potencia de calibracion = ");
  Serial.println(testPower, 3);


  // PASO 1: alinear a 45 grados electricos
  Serial.println();
  Serial.println("PASO 1: ALINEACION A 45 GRADOS E");

  setElectricalVectorCalibration(ALIGN_ANGLE_E, testPower);
  delay(ALIGN_TIME_MS);

  float pos1 = circularMeanDegrees(OFFSET_SAMPLES);

  if (isnan(pos1))
  {
    Serial.println("ERROR: AS5600 NO RESPONDE");
    detenerMotor();
    return;
  }

  Serial.print("Posicion inicial = ");
  Serial.print(pos1, 6);
  Serial.println(" deg");


  // PASO 2: mover 90 grados electricos
  Serial.println();
  Serial.println("PASO 2: MOVER +90 ELECTRICOS");

  setElectricalVectorCalibration(ALIGN_ANGLE_E + 90.0f, testPower);
  delay(DIRECTION_TEST_MS);

  float pos2 = circularMeanDegrees(OFFSET_SAMPLES);

  if (isnan(pos2))
  {
    Serial.println("ERROR: AS5600 NO RESPONDE");
    detenerMotor();
    return;
  }

  Serial.print("Posicion final = ");
  Serial.print(pos2, 6);
  Serial.println(" deg");

  float deltaM = angleDiff(pos2, pos1);

  Serial.print("Delta mecanico = ");
  Serial.print(deltaM, 6);
  Serial.println(" deg");

  float expected = 90.0f / POLE_PAIRS;

  Serial.print("Esperado = ");
  Serial.print(expected, 6);
  Serial.println(" deg");


  // Determinar sentido
  if (fabsf(deltaM) < 0.5f)
  {
    Serial.println();
    Serial.println("NO HAY MOVIMIENTO SUFICIENTE.");
    Serial.println("NO SE CALCULA OFFSET.");
    Serial.println("Aumenta TEST_POWER con '+'.");

    detenerMotor();
    calibrated = false;
    electricalSign = 0.0f;
    return;
  }

  electricalSign = (deltaM > 0.0f) ? +1.0f : -1.0f;

  Serial.println();
  Serial.println("MOVIMIENTO DETECTADO");
  Serial.print("SIGNO ELECTRICO = ");
  Serial.println((electricalSign > 0.0f) ? "+1" : "-1");


  // PASO 3: regresar a 45 grados electricos
  Serial.println();
  Serial.println("PASO 3: REGRESAR A 45 GRADOS E");

  setElectricalVectorCalibration(ALIGN_ANGLE_E, testPower);
  delay(ALIGN_TIME_MS);

  float thetaM = circularMeanDegrees(OFFSET_SAMPLES);

  if (isnan(thetaM))
  {
    Serial.println("ERROR: AS5600 NO RESPONDE");
    detenerMotor();
    calibrated = false;
    return;
  }


  // Calcular offset
  electricalOffset = wrap360(
      ALIGN_ANGLE_E
      - electricalSign * POLE_PAIRS * thetaM
  );

  float thetaCheck = wrap360(
      electricalSign * POLE_PAIRS * thetaM
      + electricalOffset
  );

  float calibrationError = fabsf(
      angleDiff(thetaCheck, ALIGN_ANGLE_E)
  );

  calibrated = true;
  thetaElectrical = thetaCheck;


  // Mostrar resultados
  Serial.println();
  Serial.println("========================================");
  Serial.println(" RESULTADO DE CALIBRACION");
  Serial.println("========================================");

  Serial.print("Theta mecanico = ");
  Serial.print(thetaM, 6);
  Serial.println(" deg");

  Serial.print("Signo electrico = ");
  Serial.println((electricalSign > 0.0f) ? "+1" : "-1");

  Serial.print("OFFSET_FOC = ");
  Serial.print(electricalOffset, 6);
  Serial.println(" deg");

  Serial.print("Theta electrico = ");
  Serial.print(thetaCheck, 6);
  Serial.println(" deg");

  Serial.print("Error de comprobacion = ");
  Serial.print(calibrationError, 6);
  Serial.println(" deg");

  if (calibrationError < 0.05f)
    Serial.println("CALIBRACION VALIDA");
  else
  {
    Serial.println("CALIBRACION NO VALIDA");
    calibrated = false;
  }

  Serial.println();

  setElectricalVectorCalibration(ALIGN_ANGLE_E, testPower);
  delay(300);
  detenerMotor();

  Serial.println("FOC DESACTIVADO");
  Serial.println("Ejecuta F para iniciar.");
}


/* ===================== ANGULO ELECTRICO ================= */

float getElectricalAngle()
{
  if (!calibrated)
    return NAN;

  float thetaM = leerAS5600();

  if (isnan(thetaM))
    return NAN;

  float thetaE =
      electricalSign * POLE_PAIRS * thetaM
      + electricalOffset;

  return wrap360(thetaE);
}


/* ========================= FOC ========================== */

/*
   CAMBIO PRINCIPAL:
   Se invierte el sentido del componente Vq.

   Antes:
     phaseA = vd*cos - sign*vq*sin
     phaseB = -vd*sin - sign*vq*cos

   Ahora:
     phaseA = vd*cos + sign*vq*sin
     phaseB = -vd*sin + sign*vq*cos

   Se conserva la convencion de calibracion:
     A = cos(thetaE)
     B = -sin(thetaE)

   IMPORTANTE:
   La direccion debe verificarse experimentalmente.
*/

void aplicarFOC(float thetaE, float vd, float vq)
{
  float rad = thetaE * PI / 180.0f;

  float c = cosf(rad);
  float s = sinf(rad);

  // Componente D sin cambios
  float phaseA = vd * c;

  float phaseB = -vd * s;

  // Componente Q con sentido invertido
  phaseA += electricalSign * vq * s;
  phaseB += electricalSign * vq * c;

  phaseA = constrain(phaseA, -1.0f, 1.0f);
  phaseB = constrain(phaseB, -1.0f, 1.0f);

  phaseACommand = phaseA;
  phaseBCommand = phaseB;

  setPhaseA_FOC(phaseA);
  setPhaseB_FOC(phaseB);
}


/* ================= CONTROL DE POSICION ================== */

void controlPosicionFOC()
{
  if (!focActivo || !calibrated)
  {
    detenerMotor();
    return;
  }

  float thetaM = leerAS5600();

  if (isnan(thetaM))
  {
    detenerMotor();
    focActivo = false;
    Serial.println("ERROR: LECTURA AS5600. FOC DESACTIVADO.");
    return;
  }

  anguloActual = thetaM;

  thetaElectrical = wrap360(
      electricalSign * POLE_PAIRS * thetaM
      + electricalOffset
  );

  error = angleDiff(setpoint, anguloActual);

  if (fabsf(error) <= TOLERANCIA)
  {
    detenerMotor();
    return;
  }

  float VqCalculado = Kp * error;

  VqCalculado = constrain(
      VqCalculado,
      -VQ_MAX,
      VQ_MAX
  );

  // Minimo de Vq para vencer friccion y mantener movimiento
  if (VqCalculado > 0.0f && VqCalculado < VQ_MINIMO)
    VqCalculado = VQ_MINIMO;

  if (VqCalculado < 0.0f && VqCalculado > -VQ_MINIMO)
    VqCalculado = -VQ_MINIMO;

  Vq = VqCalculado;

  aplicarFOC(thetaElectrical, VD, Vq);
}


/* ===================== ACTIVAR FOC ======================= */

void activarFOC()
{
  if (!calibrated || fabsf(electricalSign) < 0.5f)
  {
    Serial.println("ERROR: PRIMERO EJECUTA C Y VERIFICA LA CALIBRACION.");
    return;
  }

  float thetaM = leerAS5600();

  if (isnan(thetaM))
  {
    Serial.println("ERROR: AS5600");
    return;
  }

  anguloActual = thetaM;
  thetaElectrical = getElectricalAngle();
  error = angleDiff(setpoint, anguloActual);
  Vq = 0.0f;

  ultimoUpdateRampa = millis();
  focActivo = true;

  Serial.println();
  Serial.println("========================================");
  Serial.println(" FOC ACTIVADO");
  Serial.println("========================================");

  Serial.print("SETPOINT OBJETIVO = ");
  Serial.println(setpointObjetivo, 3);

  Serial.print("SETPOINT RAMPA = ");
  Serial.println(setpoint, 3);

  Serial.print("MEC = ");
  Serial.println(anguloActual, 3);

  Serial.print("THETA_E = ");
  Serial.println(thetaElectrical, 3);

  Serial.print("ERROR = ");
  Serial.println(error, 3);

  Serial.print("SIGN = ");
  Serial.println((electricalSign > 0.0f) ? "+1" : "-1");

  Serial.print("OFFSET_FOC = ");
  Serial.println(electricalOffset, 6);

  Serial.print("Kp = ");
  Serial.println(Kp, 4);

  Serial.print("VQ_MIN = ");
  Serial.println(VQ_MINIMO, 3);

  Serial.print("VQ_MAX = ");
  Serial.println(VQ_MAX, 3);

  Serial.print("RAMP = ");
  Serial.print(SETPOINT_RAMP_SPEED, 2);
  Serial.println(" deg/s");

  Serial.println();
}


/* ===================== DESACTIVAR FOC ==================== */

void desactivarFOC()
{
  focActivo = false;
  detenerMotor();

  Serial.println();
  Serial.println("FOC DESACTIVADO");
  Serial.println();
}


/* ======================== ESTADO ======================== */

void mostrarAngulos()
{
  float thetaM = leerAS5600();

  if (isnan(thetaM))
  {
    Serial.println("ERROR AS5600");
    return;
  }

  float thetaE = calibrated ? getElectricalAngle() : NAN;
  float currentError = angleDiff(setpoint, thetaM);

  Serial.println();
  Serial.println("========================================");
  Serial.println(" ESTADO");
  Serial.println("========================================");

  Serial.print("MEC = ");
  Serial.print(thetaM, 6);
  Serial.println(" deg");

  Serial.print("SETPOINT OBJETIVO = ");
  Serial.print(setpointObjetivo, 6);
  Serial.println(" deg");

  Serial.print("SETPOINT RAMPA = ");
  Serial.print(setpoint, 6);
  Serial.println(" deg");

  Serial.print("ERROR = ");
  Serial.print(currentError, 6);
  Serial.println(" deg");

  Serial.print("CALIBRACION = ");
  Serial.println(calibrated ? "SI" : "NO");

  if (calibrated)
  {
    Serial.print("SIGN = ");
    Serial.println((electricalSign > 0.0f) ? "+1" : "-1");

    Serial.print("OFFSET_FOC = ");
    Serial.println(electricalOffset, 6);

    Serial.print("THETA_E = ");
    Serial.print(thetaE, 6);
    Serial.println(" deg");
  }

  Serial.print("VQ = ");
  Serial.println(Vq, 4);

  Serial.print("RAMP = ");
  Serial.print(SETPOINT_RAMP_SPEED, 2);
  Serial.println(" deg/s");

  Serial.print("FOC = ");
  Serial.println(focActivo ? "ON" : "OFF");

  Serial.println();
}


/* =================== POTENCIA CALIBRACION ================ */

void aumentarPotencia()
{
  testPower += POWER_STEP;

  if (testPower > POWER_MAX)
    testPower = POWER_MAX;

  Serial.print("TEST_POWER = ");
  Serial.println(testPower, 3);
}


void disminuirPotencia()
{
  testPower -= POWER_STEP;

  if (testPower < POWER_MIN)
    testPower = POWER_MIN;

  Serial.print("TEST_POWER = ");
  Serial.println(testPower, 3);
}


/* ===================== COMANDOS SERIAL =================== */

void procesarSerial()
{
  if (!Serial.available())
    return;

  String entrada = Serial.readStringUntil('\n');
  entrada.trim();

  if (entrada.length() == 0)
    return;

  if (entrada.length() == 1)
  {
    char c = entrada.charAt(0);

    if (c == 'C' || c == 'c')
    {
      calibrarSensor();
      return;
    }

    if (c == 'A' || c == 'a')
    {
      mostrarAngulos();
      return;
    }

    if (c == 'F' || c == 'f')
    {
      activarFOC();
      return;
    }

    if (c == 'S' || c == 's')
    {
      desactivarFOC();
      return;
    }

    if (c == '+')
    {
      aumentarPotencia();
      return;
    }

    if (c == '-')
    {
      disminuirPotencia();
      return;
    }
  }

  float nuevoSetpoint = entrada.toFloat();

  if (nuevoSetpoint >= 0.0f && nuevoSetpoint <= 360.0f)
  {
    setpointObjetivo = wrap360(nuevoSetpoint);

    Serial.print("SETPOINT OBJETIVO = ");
    Serial.print(setpointObjetivo, 3);
    Serial.println(" deg");

    Serial.print("SETPOINT ACTUAL RAMPA = ");
    Serial.print(setpoint, 3);
    Serial.println(" deg");
  }
  else
  {
    Serial.println("ENTRADA NO VALIDA. USA 0...360 O UN COMANDO.");
  }
}


/* ============================ SETUP ====================== */

void setup()
{
  Serial.begin(115200);
  delay(1500);

  pinMode(A_RPWM, OUTPUT);
  pinMode(A_LPWM, OUTPUT);
  pinMode(B_RPWM, OUTPUT);
  pinMode(B_LPWM, OUTPUT);

  analogWriteResolution(8);
  analogWriteFrequency(PWM_FREQUENCY);

  detenerMotor();

  Wire.setSDA(PB9);
  Wire.setSCL(PB8);
  Wire.begin();
  Wire.setClock(400000);

  delay(500);

  Serial.println();
  Serial.println("========================================");
  Serial.println(" STM32F411 BLACK PILL");
  Serial.println(" NEMA17 17HS8501");
  Serial.println(" 50 PARES DE POLOS");
  Serial.println(" 2 x BTS7960");
  Serial.println(" AS5600");
  Serial.println(" POSICION + FOC DE TENSION");
  Serial.println(" RAMPA DE SETPOINT");
  Serial.println("========================================");

  Serial.print("POLE_PAIRS = ");
  Serial.println(POLE_PAIRS, 1);

  Serial.print("Kp = ");
  Serial.println(Kp, 4);

  Serial.print("VQ_MINIMO = ");
  Serial.println(VQ_MINIMO, 3);

  Serial.print("VQ_MAX = ");
  Serial.println(VQ_MAX, 3);

  Serial.print("VD = ");
  Serial.println(VD, 3);

  Serial.print("SETPOINT_RAMP_SPEED = ");
  Serial.print(SETPOINT_RAMP_SPEED, 2);
  Serial.println(" deg/s");

  Serial.print("PWM_FREQUENCY = ");
  Serial.print(PWM_FREQUENCY);
  Serial.println(" Hz");

  float inicial = leerAS5600();

  if (isnan(inicial))
  {
    Serial.println("ERROR: AS5600 NO RESPONDE");
  }
  else
  {
    anguloActual = inicial;

    // Iniciar la rampa desde la posicion actual
    setpoint = anguloActual;
    setpointObjetivo = anguloActual;

    Serial.print("AS5600 inicial = ");
    Serial.print(inicial, 6);
    Serial.println(" deg");
  }

  Serial.println();
  Serial.println("COMANDOS:");
  Serial.println("0...360 -> SETPOINT OBJETIVO");
  Serial.println("C       -> CALIBRAR");
  Serial.println("A       -> MOSTRAR ESTADO");
  Serial.println("F       -> ACTIVAR FOC");
  Serial.println("S       -> DETENER FOC");
  Serial.println("+       -> AUMENTAR TEST_POWER");
  Serial.println("-       -> DISMINUIR TEST_POWER");

  Serial.println();
  Serial.println("SECUENCIA:");
  Serial.println("1) C + ENTER");
  Serial.println("2) INTRODUCIR SETPOINT + ENTER");
  Serial.println("3) F + ENTER");
  Serial.println();

  detenerMotor();
}


/* ============================= LOOP ===================== */

void loop()
{
  procesarSerial();

  actualizarSetpointRampa();

  // Control de posicion cada 2 ms (500 Hz)
  static unsigned long ultimoControl = 0;

  if (millis() - ultimoControl >= 2)
  {
    ultimoControl = millis();
    controlPosicionFOC();
  }

  // Telemetria cada 100 ms
  static unsigned long ultimoReporte = 0;

  if (millis() - ultimoReporte >= 100)
  {
    ultimoReporte = millis();

    Serial.print(setpointObjetivo, 2);
    Serial.print(",");

    Serial.print(setpoint, 2);
    Serial.print(",");

    Serial.print(anguloActual, 2);
    Serial.print(",");

    Serial.print(error, 2);
    Serial.print(",");

    Serial.print(calibrated ? thetaElectrical : 0.0f, 2);
    Serial.print(",");

    Serial.print(Vq, 4);
    Serial.print(",");

    Serial.print(VD, 4);
    Serial.print(",");

    Serial.print(phaseACommand, 4);
    Serial.print(",");

    Serial.print(phaseBCommand, 4);
    Serial.print(",");

    Serial.print(pwmA);
    Serial.print(",");

    Serial.println(pwmB);
  }
}
