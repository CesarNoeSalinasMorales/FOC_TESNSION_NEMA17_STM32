#include <Wire.h>
#include <math.h>
/* =========================================================
   VALIDACION 4 CUADRANTES
   STM32F411 BLACK PILL
   NEMA17 17HS8501
   2 BTS7960
   AS5600
   ========================================================= */
/* ---------------- MOTOR ---------------- */
#define POLE_PAIRS 50.0f
/* ---------------- PWM ---------------- */
#define PWM_MAX 255
#define PWM_FREQUENCY 20000

float TEST_POWER = 0.25f;

/* ---------------- TIEMPOS ---------------- */
#define SETTLE_TIME_MS 800
#define SAMPLES 80
/* ---------------- BTS7960 A ---------------- */
#define A_RPWM PA0
#define A_LPWM PA1
/* ---------------- BTS7960 B ---------------- */
#define B_RPWM PA6
#define B_LPWM PA7
/* ---------------- AS5600 ---------------- */
#define AS5600_ADDRESS 0x36

#define AS5600_ANGLE_H 0x0C
#define AS5600_ANGLE_L 0x0D
/* =========================================================
   CONVENCION ELECTRICA VALIDADA
   =========================================================
   A =  cos(theta)
   B = -sin(theta)
   IMPORTANTE:
   este signo corresponde solamente a la fase B.
   La relación sensor -> ángulo eléctrico es:
   thetaE = +50*thetaM + OFFSET
   ========================================================= */

#define PHASE_B_SIGN (1.0f)
/* =========================================================
   UTILIDADES
   ========================================================= */
float wrap360(float x)
{
  while (x >= 360.0f)
    x -= 360.0f;
  while (x < 0.0f)
    x += 360.0f;
  return x;
}
float angleDiff(float a, float b)
{
  float d = a - b;

  while (d > 180.0f)
    d -= 360.0f;

  while (d < -180.0f)
    d += 360.0f;

  return d;
}
/* =========================================================
   AS5600
   ========================================================= */
uint16_t readAS5600Raw()
{
  Wire.beginTransmission(AS5600_ADDRESS);

  Wire.write(AS5600_ANGLE_H);

  if (Wire.endTransmission(false) != 0)
    return 0xFFFF;

  if (Wire.requestFrom(AS5600_ADDRESS, 2) != 2)
    return 0xFFFF;

  uint8_t highByte = Wire.read();
  uint8_t lowByte  = Wire.read();

  uint16_t raw =
      ((uint16_t)highByte << 8) |
      lowByte;

  raw &= 0x0FFF;

  return raw;
}
float readAS5600()
{
  uint16_t raw = readAS5600Raw();

  if (raw == 0xFFFF)
    return NAN;

  return ((float)raw * 360.0f) / 4096.0f;
}
/* =========================================================
   PROMEDIO CIRCULAR
   ========================================================= */
float readAngleAverage(int samples)
{
  float sumSin = 0.0f;
  float sumCos = 0.0f;
  int valid = 0;
  for (int i = 0; i < samples; i++)
  {
    float angle = readAS5600();
    if (!isnan(angle))
    {
      float rad = angle * DEG_TO_RAD;
      sumSin += sinf(rad);
      sumCos += cosf(rad);
      valid++;
    }
    delay(3);
  }
  if (valid == 0)
    return NAN;
  float mean =
      atan2f(sumSin, sumCos) * RAD_TO_DEG;
  return wrap360(mean);
}
/* =========================================================
   FASE A
   ========================================================= */
void setPhaseA(float v)
{
  v = constrain(v, -1.0f, 1.0f);

  int pwm =
      (int)(fabsf(v) * PWM_MAX);

  if (v > 0.0f)
  {
    analogWrite(A_LPWM, 0);
    analogWrite(A_RPWM, pwm);
  }
  else if (v < 0.0f)
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
/* =========================================================
   FASE B
   ========================================================= */
void setPhaseB(float v)
{
  v = constrain(v, -1.0f, 1.0f);

  int pwm =
      (int)(fabsf(v) * PWM_MAX);

  if (v > 0.0f)
  {
    analogWrite(B_LPWM, 0);
    analogWrite(B_RPWM, pwm);
  }
  else if (v < 0.0f)
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


/* =========================================================
   STOP
   ========================================================= */

void motorStop()
{
  setPhaseA(0.0f);
  setPhaseB(0.0f);
}


/* =========================================================
   GENERAR VECTOR ELECTRICO
   ========================================================= */

void setElectricalVector(float thetaE)
{
  thetaE = wrap360(thetaE);

  float rad = thetaE * DEG_TO_RAD;
  float phaseA = TEST_POWER * cosf(rad);
  float phaseB = PHASE_B_SIGN * TEST_POWER * sinf(rad);
  setPhaseA(phaseA);
  setPhaseB(phaseB);
}


/* =========================================================
   VALIDACION 4 CUADRANTES
   ========================================================= */

void validateFourQuadrants()
{
  Serial.println();
  Serial.println("========================================");
  Serial.println(" VALIDACION DE 4 CUADRANTES");
  Serial.println("========================================");

  Serial.println();

  Serial.print("POLE_PAIRS = ");
  Serial.println(POLE_PAIRS, 1);

  Serial.print("TEST_POWER = ");
  Serial.println(TEST_POWER, 3);

  Serial.println();

  /*
     -----------------------------------------
     POSICION INICIAL = 45°
     -----------------------------------------
  */

  setElectricalVector(45.0f);
  delay(SETTLE_TIME_MS);
  float previous = readAngleAverage(SAMPLES);
  if (isnan(previous))
  {
    Serial.println("ERROR AS5600");
    motorStop();
    return;
  }

  float start = previous;

  Serial.print("Inicio 45E = ");
  Serial.print(start, 4);
  Serial.println(" deg");


  /*
     -----------------------------------------
     CUADRANTE 1
     45 -> 135
     -----------------------------------------
  */

  setElectricalVector(135.0f);

  delay(SETTLE_TIME_MS);

  float p1 = readAngleAverage(SAMPLES);

  float d1 = angleDiff(p1, previous);

  Serial.println();
  Serial.println("CUADRANTE 1");
  Serial.println("45E -> 135E");

  Serial.print("Mecanico = ");
  Serial.print(p1, 4);
  Serial.println(" deg");

  Serial.print("Delta = ");
  Serial.print(d1, 4);
  Serial.println(" deg");

  previous = p1;
  /*
     -----------------------------------------
     CUADRANTE 2
     135 -> 225
     -----------------------------------------
  */

  setElectricalVector(225.0f);

  delay(SETTLE_TIME_MS);

  float p2 = readAngleAverage(SAMPLES);

  float d2 = angleDiff(p2, previous);

  Serial.println();
  Serial.println("CUADRANTE 2");
  Serial.println("135E -> 225E");

  Serial.print("Mecanico = ");
  Serial.print(p2, 4);
  Serial.println(" deg");

  Serial.print("Delta = ");
  Serial.print(d2, 4);
  Serial.println(" deg");

  previous = p2;
  /*
     -----------------------------------------
     CUADRANTE 3
     225 -> 315
     -----------------------------------------
  */

  setElectricalVector(315.0f);

  delay(SETTLE_TIME_MS);

  float p3 = readAngleAverage(SAMPLES);

  float d3 = angleDiff(p3, previous);

  Serial.println();
  Serial.println("CUADRANTE 3");
  Serial.println("225E -> 315E");

  Serial.print("Mecanico = ");
  Serial.print(p3, 4);
  Serial.println(" deg");

  Serial.print("Delta = ");
  Serial.print(d3, 4);
  Serial.println(" deg");

  previous = p3;

  /*
     -----------------------------------------
     CUADRANTE 4
     315 -> 405
     -----------------------------------------
  */

  setElectricalVector(405.0f);

  delay(SETTLE_TIME_MS);

  float p4 = readAngleAverage(SAMPLES);

  float d4 = angleDiff(p4, previous);

  Serial.println();
  Serial.println("CUADRANTE 4");
  Serial.println("315E -> 405E");

  Serial.print("Mecanico = ");
  Serial.print(p4, 4);
  Serial.println(" deg");

  Serial.print("Delta = ");
  Serial.print(d4, 4);
  Serial.println(" deg");

  /*
     -----------------------------------------
     RESULTADO TOTAL
     -----------------------------------------
  */

  float total = angleDiff(p4, start);

  float magnitude = fabsf(total);

  float expected = 360.0f / POLE_PAIRS;

  float error = magnitude - expected;

  float errorPercent = 100.0f * error/expected;

  motorStop();

  Serial.println();
  Serial.println("========================================");
  Serial.println(" RESULTADO 4 CUADRANTES");
  Serial.println("========================================");

  Serial.print("Inicial = ");
  Serial.print(start, 4);
  Serial.println(" deg");

  Serial.print("Final   = ");
  Serial.print(p4, 4);
  Serial.println(" deg");

  Serial.println();

  Serial.print("Q1 = ");
  Serial.print(d1, 4);
  Serial.println(" deg");

  Serial.print("Q2 = ");
  Serial.print(d2, 4);
  Serial.println(" deg");

  Serial.print("Q3 = ");
  Serial.print(d3, 4);
  Serial.println(" deg");

  Serial.print("Q4 = ");
  Serial.print(d4, 4);
  Serial.println(" deg");

  Serial.println();

  Serial.print("Delta total = ");
  Serial.print(total, 4);
  Serial.println(" deg");

  Serial.print("Magnitud = ");
  Serial.print(magnitude, 4);
  Serial.println(" deg");

  Serial.print("Esperado = ");
  Serial.print(expected, 4);
  Serial.println(" deg");

  Serial.print("Error = ");
  Serial.print(error, 4);
  Serial.println(" deg");

  Serial.print("Error % = ");
  Serial.print(errorPercent, 2);
  Serial.println(" %");

  Serial.println();

  if (magnitude > 0.0f)
  {
    Serial.println("El sentido mecanico observado es POSITIVO.");
  }

  if (fabsf(error) < 0.30f)
  {
    Serial.println("OK: los 4 cuadrantes son consistentes.");
  }
  else
  {
    Serial.println("AVISO: revisar seguimiento del rotor.");
  }

  Serial.println();
}

/* =========================================================
   SETUP
   ========================================================= */
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

  motorStop();
  /*
     AS5600

     PB9 = SDA
     PB8 = SCL
  */
  Wire.setSDA(PB9);
  Wire.setSCL(PB8);

  Wire.begin();

  Wire.setClock(400000);
  Serial.println();
  Serial.println("========================================");
  Serial.println(" STM32F411 + NEMA17 + 2x BTS7960");
  Serial.println(" VALIDACION 4 CUADRANTES");
  Serial.println("========================================");
  Serial.println();

  float angle = readAS5600();

  if (isnan(angle))
  {
    Serial.println("ERROR: AS5600 NO RESPONDE");
  }
  else
  {
    Serial.print("AS5600 = ");
    Serial.print(angle,4);
    Serial.println(" deg");
  }

  Serial.println();
  Serial.println("Comando:");
  Serial.println("D = iniciar validacion");
  Serial.println("S = stop");
  Serial.println();
}
/* =========================================================
   LOOP
   ========================================================= */

void loop()
{
  if (Serial.available())
  {
    char c =
        Serial.read();

    if (c == 'D' || c == 'd')
    {
      validateFourQuadrants();
    }

    else if (c == 'S' || c == 's')
    {
      motorStop();

      Serial.println("MOTOR STOP");
    }
  }
}