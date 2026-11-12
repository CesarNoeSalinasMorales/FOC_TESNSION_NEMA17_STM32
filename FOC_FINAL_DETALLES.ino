#include <Arduino.h>
#include <Wire.h>
#include <math.h>
/* =========================================================
   STM32F411 BLACK PILL
   NEMA17 17HS8501
   50 PARES DE POLOS
   2 x BTS7960
   AS5600
   CONTROL DE POSICION + FOC DE TENSION
   CABLEADO:
      BOBINA A -> BTS7960 A
      BOBINA B -> BTS7960 B
   BTS7960 A:
      PA0 -> RPWM
      PA1 -> LPWM
   BTS7960 B:
      PA6 -> RPWM
      PA7 -> LPWM
   AS5600:
      PB8 -> SCL
      PB9 -> SDA
   FOC:
      Vd = 0
      Vq = Kp * error
   ========================================================= */
/* =========================================================
   PINES BTS7960
   ========================================================= */
// BOBINA A
#define A_RPWM PA0
#define A_LPWM PA1
// BOBINA B
#define B_RPWM PA6
#define B_LPWM PA7
/* =========================================================
   AS5600
   ========================================================= */
#define AS5600_ADDR      0x36
#define AS5600_ANGLE_H   0x0C
/* =========================================================
   MOTOR
   ========================================================= */
#define POLE_PAIRS 50.0f
/* =========================================================
   PWM
   ========================================================= */
#define PWM_MAX        255
#define PWM_FREQUENCY  20000
/* =========================================================
   CONTROL PROPORCIONAL
   ========================================================= */
float Kp = 0.020f;
/* =========================================================
   VQ
   ========================================================= */
/*
   Minimo necesario para que el motor comience
   a moverse.
*/
float VQ_MINIMO = 0.25f;
/*
   Maximo de Vq.
*/
float VQ_MAX = 0.80f;
/*
   Eje D.
   Para esta prueba:
      Vd = 0
*/
float VD = 0.0f;
/* =========================================================
   SETPOINT
   ========================================================= */
/*
   Setpoint que recibe el usuario.
   Ejemplo:
      escribir 35
   entonces:
      setpointObjetivo = 35
*/
float setpointObjetivo = 140.0f;
/*
   Este es el setpoint que realmente utiliza
   el controlador FOC.
   Se mueve gradualmente hacia setpointObjetivo.
*/
float setpoint = 140.0f;
/* =========================================================
   VELOCIDAD DE RAMPA
   ========================================================= */
/*
   Grados mecanicos por segundo.
   Ejemplo:
    20 deg/s
   Para recorrer 40 grados:
      40 / 20 = 2 segundos
*/
float SETPOINT_RAMP_SPEED = 20.0f;
/*
   Variables de tiempo para la rampa.
*/
unsigned long ultimoUpdateRampa = 0;
/* =========================================================
   POSICION
   ========================================================= */
float anguloActual = 0.0f;
float error = 0.0f;
/* =========================================================
   ANGULO ELECTRICO
   ========================================================= */
float thetaElectrical = 0.0f;
/* =========================================================
   CALIBRACION
   ========================================================= */
float electricalSign = 0.0f;
float electricalOffset = 0.0f;
bool calibrated = false;
/* =========================================================
   FOC
   ========================================================= */
bool focActivo = false;
float Vq = 0.0f;
/* =========================================================
   SALIDAS FOC
   ========================================================= */
float phaseACommand = 0.0f;
float phaseBCommand = 0.0f;
int pwmA = 0;
int pwmB = 0;
/* =========================================================
   TOLERANCIA
   ========================================================= */
float TOLERANCIA = 1.0f;
/* =========================================================
   CALIBRACION
   ========================================================= */
#define ALIGN_ANGLE_E       45.0f
float testPower = 0.25f;
#define POWER_STEP          0.05f
#define POWER_MIN           0.05f
#define POWER_MAX           0.4f

#define ALIGN_TIME_MS       1200
#define DIRECTION_TEST_MS   900
#define OFFSET_SAMPLES      100
/* =========================================================
   WRAP ANGULAR
   ========================================================= */
float wrap360(float angle)
{
  while (angle >= 360.0f)
  {
    angle -= 360.0f;
  }
  while (angle < 0.0f)
  {
    angle += 360.0f;
  }
  return angle;
}
/* =========================================================
   DIFERENCIA ANGULAR
   RESULTADO: -180 ... +180
   ========================================================= */
float angleDiff(float a, float b)
{
  float d = a - b;
  while (d > 180.0f)
  {
    d -= 360.0f;
  }
  while (d < -180.0f)
  {
    d += 360.0f;
  }
  return d;
}
/* =========================================================
   ACTUALIZAR RAMPA DE SETPOINT
   =========================================================
   Esta función mueve:setpoint
   hacia:setpointObjetivo
   utilizando siempre el camino angular corto.
   ========================================================= */
void actualizarSetpointRampa()
{
  unsigned long ahora = millis();
 /*
     Limitar frecuencia de actualización de la rampa.
  */
  if (ahora - ultimoUpdateRampa < 10)
  {
    return;
  }
  /*
     Delta de tiempo en segundos.
  */
  float dt =
      (ahora - ultimoUpdateRampa) / 1000.0f;
  ultimoUpdateRampa = ahora;
  /*
     Error angular entre el setpoint objetivo
     y el setpoint actualmente aplicado.
  */
  float delta =
      angleDiff(setpointObjetivo,setpoint);
  /*
     Si ya estamos prácticamente en el objetivo,
     fijarlo exactamente.
  */
  float toleranciaRampa = 0.01f;
  if (fabsf(delta) <= toleranciaRampa)
  {
    setpoint = setpointObjetivo;

    return;
  }
  /*
     Distancia máxima que podemos recorrer
     durante este intervalo.
  */
  float paso = SETPOINT_RAMP_SPEED * dt;
  /*
     No sobrepasar el objetivo.
  */
  if (fabsf(delta) <= paso)
  {
    setpoint = setpointObjetivo;
    return;
  }
  /*
     Avanzar en la dirección correcta.
  */
  if (delta > 0.0f)
  {
    setpoint += paso;
  }
  else
  {
    setpoint -= paso;
  }
  /*
     Normalizar.
  */
  setpoint =  wrap360(setpoint);
}
/* =========================================================
   LEER AS5600 RAW
   ========================================================= */
uint16_t leerAS5600Raw()
{
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_ANGLE_H);
  if (Wire.endTransmission(false) != 0)
  {
    return 0xFFFF;
  }

  if (Wire.requestFrom(AS5600_ADDR, 2) != 2)
  {
    return 0xFFFF;
  }

  uint8_t highByte = Wire.read();
  uint8_t lowByte  = Wire.read();

  uint16_t raw = ((uint16_t)highByte << 8) | lowByte;
  raw &= 0x0FFF;
  return raw;
}

/* =========================================================
   LEER AS5600 EN GRADOS
   ========================================================= */
float leerAS5600()
{
  uint16_t raw = leerAS5600Raw();
  if (raw == 0xFFFF)
  {
    return NAN;
  }

  return
      ((float)raw * 360.0f) /
      4096.0f;
}
/* =========================================================
   PROMEDIO CIRCULAR
   ========================================================= */
float circularMeanDegrees(int samples)
{
  float sumSin = 0.0f;
  float sumCos = 0.0f;

  int valid = 0;

  for (int i = 0; i < samples; i++)
  {
    float angle =leerAS5600();

    if (!isnan(angle))
    {
      float rad = angle * PI /  180.0f;
      sumSin += sinf(rad);
      sumCos += cosf(rad);
      valid++;
    }
    delay(5);
  }

  if (valid == 0)
  {
    return NAN;
  }
  float meanRad = atan2f(sumSin,sumCos);
  return wrap360(meanRad *180.0f /PI);
}
/* =========================================================
   FASE A - FOC
   ========================================================= */
void setPhaseA_FOC(float value)
{
  value =constrain(value, -1.0f, 1.0f);
  int pwm =(int)( fabsf(value) *  PWM_MAX);
  pwm = constrain(  pwm, 0,PWM_MAX );

  if (value > 0.0f)
  {
    analogWrite( A_LPWM, 0);
    analogWrite( A_RPWM,pwm );
  }
  else if (value < 0.0f)
  {
    analogWrite( A_RPWM,0 );
    analogWrite(  A_LPWM, pwm);
  }
  else
  {
    analogWrite(A_RPWM,0);
    analogWrite( A_LPWM,0);
  }
  pwmA = pwm;
}
/* =========================================================
   FASE B - FOC
   ========================================================= */
void setPhaseB_FOC(float value)
{
  value =constrain( value, -1.0f,  1.0f );

  int pwm = (int)( fabsf(value) * PWM_MAX);


  pwm =constrain( pwm, 0, PWM_MAX);


  if (value > 0.0f)
  {
    analogWrite( B_LPWM, 0);
    analogWrite(B_RPWM, pwm);
  }
  else if (value < 0.0f)
  {
    analogWrite( B_RPWM,0);
    analogWrite(B_LPWM,pwm);
  }
  else
  {
    analogWrite( B_RPWM, 0);
    analogWrite(B_LPWM,0);
  }

  pwmB = pwm;
}
/* =========================================================
   FASE A - CALIBRACION
   ========================================================= */
void setPhaseA_Calibration(float value)
{
  value = constrain(  value, -1.0f, 1.0f );
  int pwm =(int)( fabsf(value) *  PWM_MAX );

  if (value > 0.0f)
  {
    analogWrite( A_LPWM, 0 );
    analogWrite(  A_RPWM,   pwm);
  }
  else if (value < 0.0f)
  {
    analogWrite( A_RPWM,  0);
    analogWrite( A_LPWM, pwm );
  }
  else
  {
    analogWrite( A_RPWM,  0 );
    analogWrite( A_LPWM,  0);
  }
}
/* =========================================================
   FASE B - CALIBRACION
   ========================================================= */
void setPhaseB_Calibration(float value)
{
  value = constrain( value, -1.0f, 1.0f );
  int pwm = (int)(  fabsf(value) * PWM_MAX );

  if (value > 0.0f)
  {
    analogWrite(  B_LPWM, 0);
    analogWrite( B_RPWM,  pwm );
  }
  else if (value < 0.0f)
  {
    analogWrite( B_RPWM, 0);
    analogWrite( B_LPWM, pwm);
  }
  else
  {
    analogWrite(B_RPWM, 0 );
    analogWrite( B_LPWM, 0);
  }
}
/* =========================================================
   STOP
   ========================================================= */
void detenerMotor()
{
  analogWrite( A_RPWM,0);
  analogWrite(A_LPWM, 0);
  analogWrite(B_RPWM,0 );
  analogWrite(B_LPWM, 0 );
  pwmA = 0;
  pwmB = 0;
  phaseACommand = 0.0f;
  phaseBCommand = 0.0f;
  Vq = 0.0f;
}
/* =========================================================
   VECTOR ELECTRICO DE CALIBRACION
   =========================================================
      A = cos(theta)
      B = -sin(theta)
   ========================================================= */
void setElectricalVectorCalibration(
    float thetaElectrical,
    float amplitude)
{
  thetaElectrical = wrap360(  thetaElectrical );
  float rad = thetaElectrical * PI / 180.0f;
  float phaseA = amplitude * cosf(rad);
  float phaseB =-amplitude *sinf(rad);
  setPhaseA_Calibration( phaseA);
  setPhaseB_Calibration( phaseB);
}
/* =========================================================
   CALIBRACION AUTOMATICA
   ========================================================= */
void calibrarSensor()
{
  focActivo = false;
  detenerMotor();
  calibrated = false;
  electricalSign = 0.0f;
  Serial.println();
  Serial.println("========================================");
  Serial.println(" CALIBRACION DE ANGULO ELECTRICO");
  Serial.println( "========================================");
  Serial.println();
  Serial.print(  "POLE_PAIRS = ");
  Serial.println(  POLE_PAIRS, 1);
  Serial.print("TEST_POWER = ");
  Serial.println( testPower, 3);
  Serial.println();
  /* =====================================================
     PASO 1
     ===================================================== */
  Serial.println("PASO 1: ALINEACION A 45 GRADOS E");
  setElectricalVectorCalibration(ALIGN_ANGLE_E,testPower );
  delay(  ALIGN_TIME_MS);

  float pos1 = circularMeanDegrees( OFFSET_SAMPLES );
  if (isnan(pos1))
  {
    Serial.println( "ERROR: AS5600 NO RESPONDE");
    detenerMotor();
    return;
  }
  Serial.print("Posicion inicial = ");
  Serial.print(pos1,6);
  Serial.println( " deg");
  /* =====================================================
     PASO 2
     ===================================================== */
  Serial.println();
  Serial.println("PASO 2: MOVER +90 ELECTRICOS");
  setElectricalVectorCalibration( ALIGN_ANGLE_E + 90.0f, testPower);
  delay( DIRECTION_TEST_MS);
  float pos2 = circularMeanDegrees( OFFSET_SAMPLES );
  if (isnan(pos2))
  {
    Serial.println("ERROR: AS5600 NO RESPONDE" );
    detenerMotor();
    return;
  }
  Serial.print("Posicion final = " );
  Serial.print( pos2,6);
  Serial.println( " deg");
  /* =====================================================
     DELTA
     ===================================================== */
  float deltaM =angleDiff( pos2,pos1);
  Serial.print( "Delta mecanico = ");
  Serial.print( deltaM, 6);
  Serial.println( " deg");
  float expected = 90.0f / POLE_PAIRS;
  Serial.print("Esperado = ");

  Serial.print( expected, 6);
  Serial.println( " deg");
  /* =====================================================
     SIGNO
     ===================================================== */
  if (fabsf(deltaM) < 0.5f)
  {
    Serial.println();
    Serial.println("****************************************");
    Serial.println( "NO HAY MOVIMIENTO SUFICIENTE");
    Serial.println( "NO SE CALCULA OFFSET");
    Serial.println( "****************************************");
    Serial.println();
    Serial.println("Aumenta TEST_POWER con '+'");
    detenerMotor();
    calibrated = false;
    electricalSign = 0.0f;
    return;
  }
  if (deltaM > 0.0f)
  {
    electricalSign = +1.0f;
  }
  else
  {
    electricalSign = -1.0f;
  }
  Serial.println();
  Serial.println("MOVIMIENTO DETECTADO");
  Serial.print("SIGNO ELECTRICO = ");
  if (electricalSign > 0.0f)
  {
    Serial.println( "+1" );
  }
  else
  {
    Serial.println( "-1");
  }
  /* =====================================================
     PASO 3
     ===================================================== */
  Serial.println();
  Serial.println( "PASO 3: REGRESAR A 45 GRADOS E");
  setElectricalVectorCalibration(ALIGN_ANGLE_E,testPower);
  delay( ALIGN_TIME_MS);
  /* =====================================================
     POSICION
     ===================================================== */
  float thetaM =circularMeanDegrees( OFFSET_SAMPLES  );
  if (isnan(thetaM))
  {
    Serial.println( "ERROR: AS5600 NO RESPONDE" );
    detenerMotor();
    calibrated = false;
    return;
  }
  /* =====================================================
     OFFSET
     ===================================================== */
  electricalOffset =wrap360( ALIGN_ANGLE_E - electricalSign * POLE_PAIRS * thetaM);
  /* =====================================================
     VERIFICAR
     ===================================================== */
  float thetaCheck =wrap360( electricalSign * POLE_PAIRS *  thetaM + electricalOffset);
  calibrated = true;
  thetaElectrical = thetaCheck;
  float calibrationError =fabsf( angleDiff( thetaCheck, ALIGN_ANGLE_E ));
  /* =====================================================
     RESULTADO
     ===================================================== */
  Serial.println();
  Serial.println("========================================");
  Serial.println(" RESULTADO DE CALIBRACION");
  Serial.println("========================================");
  Serial.print("Theta mecanico = ");
  Serial.print( thetaM,6);
  Serial.println( " deg");
  Serial.print("Signo electrico = ");

  if (electricalSign > 0.0f)
  {
    Serial.println( "+1");
  }
  else
  {
    Serial.println( "-1");
  }

  Serial.print("OFFSET_FOC = ");
  Serial.print( electricalOffset,6);
  Serial.println(" deg");
  Serial.print("Theta electrico = ");
  Serial.print(thetaCheck,6);
  Serial.println(" deg");
  Serial.print("Error de comprobacion = ");
  Serial.print(calibrationError,6 );
  Serial.println(" deg");
  if (calibrationError < 0.05f)
  {
    Serial.println("CALIBRACION VALIDA");
  }
  else
  {
    Serial.println( "AVISO: REVISAR CALIBRACION");
  }
  Serial.println();
  setElectricalVectorCalibration( ALIGN_ANGLE_E, testPower);
  delay(300);
  detenerMotor();
  Serial.println("FOC DESACTIVADO" );
  Serial.println( "Ejecuta F para iniciar.");
  Serial.println();
}
/* =========================================================
   ANGULO ELECTRICO
   ========================================================= */
float getElectricalAngle()
{
  if (!calibrated)
  {
    return NAN;
  }
  float thetaM = leerAS5600();
  if (isnan(thetaM))
  {
    return NAN;
  }
  float thetaE = electricalSign * POLE_PAIRS * thetaM + electricalOffset;
  return wrap360( thetaE);
}
/* =========================================================
   INVERSE PARK
   =========================================================
   D:
      A = cos(thetaE)
      B = -sin(thetaE)
   Q corregido para el sentido mecanico:
      A = -SIGN*sin(thetaE)
      B = -SIGN*cos(thetaE)
   ========================================================= */
void aplicarFOC(float thetaE,float vd,float vq)
{
  float rad =thetaE *PI /180.0f;
  float c =cosf(rad);
  float s =sinf(rad);
  /* =====================================================
     FASE A
     ===================================================== */
  float phaseA = vd * c - electricalSign *  vq * s;

  /* =====================================================
     FASE B
     ===================================================== */
  float phaseB = -vd * s - electricalSign *  vq * c;
  /* =====================================================
     LIMITAR
     ===================================================== */
  phaseA = constrain(  phaseA,  -1.0f, 1.0f);
  phaseB =constrain( phaseB,-1.0f,1.0f);
  /* =====================================================
     TELEMETRIA
     ===================================================== */
  phaseACommand =phaseA;
  phaseBCommand =phaseB;
  /* =====================================================
     PWM
     ===================================================== */
  setPhaseA_FOC( phaseA);
  setPhaseB_FOC( phaseB);
}
/* =========================================================
   CONTROL PROPORCIONAL + FOC
   ========================================================= */
void controlPosicionFOC()
{
  /* =====================================================
     FOC OFF
     ===================================================== */
  if (!focActivo)
  {
    detenerMotor();
    return;
  }
  /* =====================================================
     SIN CALIBRACION
     ===================================================== */
  if (!calibrated)
  {
    detenerMotor();
    return;
  }
  /* =====================================================
     LEER AS5600
     ===================================================== */
  float thetaM =
      leerAS5600();
  if (isnan(thetaM))
  {
    detenerMotor();
    return;
  }
  anguloActual = thetaM;
  /* =====================================================
     ACTUALIZAR ANGULO ELECTRICO
     ===================================================== */
  thetaElectrical =wrap360(  electricalSign *  POLE_PAIRS *  thetaM  + electricalOffset);
  /* =====================================================
     ERROR DE POSICION
     AHORA EL ERROR SE CALCULA CONTRA EL SETPOINT
     QUE YA PASO POR LA RAMPA.
     ===================================================== */
  error = angleDiff(setpoint, anguloActual );
  /* =====================================================
     ZONA DE TOLERANCIA
     ===================================================== */
  if (fabsf(error) <= TOLERANCIA)
  {
    Vq = 0.0f;
    detenerMotor();
    return;
  }
  /* =====================================================
     VQ PROPORCIONAL
     ===================================================== */
  float VqCalculado =Kp *error;
  /* =====================================================
     LIMITE MAXIMO
     ===================================================== */
  if (VqCalculado > VQ_MAX)
  {
    VqCalculado = VQ_MAX;
  }
  if (VqCalculado < -VQ_MAX)
  {
    VqCalculado = -VQ_MAX;
  }
  /* =====================================================
     MINIMO DE VQ
     ===================================================== */

  if (VqCalculado > 0.0f &&
      VqCalculado < VQ_MINIMO)
  {
    VqCalculado = VQ_MINIMO;
  }

  if (VqCalculado < 0.0f &&
      VqCalculado > -VQ_MINIMO)
  {
    VqCalculado =-VQ_MINIMO;
  }
  Vq =VqCalculado;
  /* =====================================================
     FOC
     ===================================================== */
  aplicarFOC( thetaElectrical, VD, Vq );
}
/* =========================================================
   ACTIVAR FOC
   ========================================================= */
void activarFOC()
{
  if (!calibrated)
  {
    Serial.println();
    Serial.println( "ERROR: PRIMERO EJECUTA C.");
    Serial.println();
    return;
  }
  float thetaM = leerAS5600();
  if (isnan(thetaM))
  {
    Serial.println( "ERROR: AS5600");
    return;
  }
  anguloActual = thetaM;
  thetaElectrical = getElectricalAngle();
  /*
     Al activar FOC, mantenemos el setpoint de rampa
     que ya teniamos.
  */
  error = angleDiff( setpoint, anguloActual );
  Vq = 0.0f;
  focActivo = true;
  Serial.println();
  Serial.println("========================================");
  Serial.println( " FOC ACTIVADO");
  Serial.println("========================================");
  Serial.println();
  Serial.print("SETPOINT OBJETIVO = ");
  Serial.print( setpointObjetivo, 3);
  Serial.println(" deg");
  Serial.print( "SETPOINT RAMPA = ");
  Serial.print( setpoint, 3 );
  Serial.println(" deg" );
  Serial.print( "MEC = ");
  Serial.print( anguloActual,3);
  Serial.println(" deg");
  Serial.print("THETA_E = ");
  Serial.print( thetaElectrical,3);
  Serial.println(" deg");
  Serial.print("ERROR = ");
  Serial.print(error, 3);
  Serial.println(" deg");
  Serial.print( "Kp = ");
  Serial.println( Kp, 4);
  Serial.print("VQ_MIN = " );
  Serial.println( VQ_MINIMO, 3 );
  Serial.print(  "VQ_MAX = " );
  Serial.println( VQ_MAX, 3);
  Serial.print( "RAMP = ");
  Serial.print( SETPOINT_RAMP_SPEED, 2);
  Serial.println( " deg/s");
  Serial.println();
}
/* =========================================================
   DETENER FOC
   ========================================================= */
void desactivarFOC()
{
  focActivo = false;
  detenerMotor();
  Serial.println();
  Serial.println( "FOC DESACTIVADO" );
  Serial.println();
}
/* =========================================================
   MOSTRAR ESTADO
   ========================================================= */
void mostrarAngulos()
{
  float thetaM =leerAS5600();
  if (isnan(thetaM))
  {
    Serial.println( "ERROR AS5600"
    );
    return;
  }
  float thetaE = calibrated ? getElectricalAngle() : NAN;
  float currentError =angleDiff(  setpoint, thetaM );
  Serial.println();
  Serial.println("========================================");
  Serial.println( " ESTADO");
  Serial.println( "========================================");
  Serial.print( "MEC = ");
  Serial.print(thetaM,6);
  Serial.println(" deg");
  Serial.print("SETPOINT OBJETIVO = ");
  Serial.print( setpointObjetivo,6);
  Serial.println( " deg");
  Serial.print( "SETPOINT RAMPA = ");
  Serial.print(setpoint,6 );
  Serial.println(" deg" );
  Serial.print( "ERROR = " );
  Serial.print(  currentError, 6);
  Serial.println( " deg");

  if (!calibrated)
  {
    Serial.println(  "CALIBRACION = NO");
    Serial.println();
    return;
  }
  Serial.println( "CALIBRACION = SI");
  Serial.print("SIGN = ");
  if (electricalSign > 0.0f)
  {
    Serial.println( "+1");
  }
  else
  {
    Serial.println( "-1");
  }
  Serial.print( "OFFSET_FOC = ");
  Serial.println( electricalOffset, 6);
  Serial.print("THETA_E = ");
  Serial.print( thetaE, 6);
  Serial.println(" deg");
  Serial.print( "VQ = ");
  Serial.println(Vq,4);
  Serial.print( "RAMP = ");
  Serial.print( SETPOINT_RAMP_SPEED, 2);
  Serial.println( " deg/s");
  Serial.print( "FOC = " );
  if (focActivo)
  {
    Serial.println("ON" );
  }
  else
  {
    Serial.println("OFF");
  }
  Serial.println();
}
/* =========================================================
   AUMENTAR POTENCIA DE CALIBRACION
   ========================================================= */
void aumentarPotencia()
{
  testPower += POWER_STEP;
  if (testPower > POWER_MAX)
  {
    testPower = POWER_MAX;
  }
  Serial.print("TEST_POWER = ");
  Serial.println(testPower,3);
}
/* =========================================================
   DISMINUIR POTENCIA
   ========================================================= */
void disminuirPotencia()
{
  testPower -= POWER_STEP;
  if (testPower < POWER_MIN)
  {
    testPower =POWER_MIN;
  }
  Serial.print("TEST_POWER = ");
  Serial.println( testPower, 3 );
}
/* =========================================================
   PROCESAR SERIAL
   =========================================================
   C       = calibrar
   A       = mostrar estado
   F       = activar FOC
   S       = detener
   +       = aumentar TEST_POWER
   -       = disminuir TEST_POWER
   Numero  = SETPOINT OBJETIVO
   ========================================================= */
void procesarSerial()
{
  if (!Serial.available())
  {
    return;
  }

  String entrada = Serial.readStringUntil('\n');
  entrada.trim();
  if (entrada.length() == 0)
  {
    return;
  }
  /* =====================================================
     COMANDOS
     ===================================================== */
  if (entrada.length() == 1)
  {
    char c =  entrada.charAt(0);
    /* ---------------------------------------------------
       C
       --------------------------------------------------- */
    if (c == 'C' || c == 'c')
    {
      calibrarSensor();
      return;
    }
    /* ---------------------------------------------------
       A
       --------------------------------------------------- */
    if (c == 'A' || c == 'a')
    {
      mostrarAngulos();
      return;
    }
    /* ---------------------------------------------------
       F
       --------------------------------------------------- */
    if (c == 'F' || c == 'f')
    {
      activarFOC();
      return;
    }
    /* ---------------------------------------------------
       S
       --------------------------------------------------- */
    if (c == 'S' || c == 's')
    {
      desactivarFOC();
      return;
    }
    /* ---------------------------------------------------
       +
       --------------------------------------------------- */
    if (c == '+')
    {
      aumentarPotencia();
      return;
    }
    /* ---------------------------------------------------
       -
       --------------------------------------------------- */
    if (c == '-')
    {
      disminuirPotencia();
      return;
    }
  }
  /* =====================================================
     SETPOINT OBJETIVO
     ===================================================== */
  float nuevoSetpoint = entrada.toFloat();
  if (nuevoSetpoint >= 0.0f &&
      nuevoSetpoint <= 360.0f)
  {
    setpointObjetivo = wrap360( nuevoSetpoint );
    Serial.print("SETPOINT OBJETIVO = ");
    Serial.print( setpointObjetivo, 3);
    Serial.println( " deg");
    Serial.print( "SETPOINT ACTUAL RAMPA = ");
    Serial.print(  setpoint, 3);
    Serial.println( " deg");
  }
}
/* =========================================================
   SETUP
   ========================================================= */
void setup()
{
  Serial.begin(  115200);
  delay(1500);
  /* =====================================================
     GPIO
     ===================================================== */
  pinMode( A_RPWM, OUTPUT);
  pinMode(A_LPWM,OUTPUT);
  pinMode( B_RPWM, OUTPUT);
  pinMode( B_LPWM,  OUTPUT);
  /* =====================================================
     PWM
     ===================================================== */
  analogWriteResolution(8);
  analogWriteFrequency(  PWM_FREQUENCY );
  /* =====================================================
     MOTOR OFF
     ===================================================== */
  detenerMotor();
  /* =====================================================
     I2C
     ===================================================== */
  Wire.setSDA( PB9);
  Wire.setSCL( PB8 );
  Wire.begin();
  Wire.setClock( 400000);
  delay(500);
  /* =====================================================
     INFORMACION
     ===================================================== */
  Serial.println();
  Serial.println( "========================================");
  Serial.println(" STM32F411 BLACK PILL");
  Serial.println( " NEMA17 17HS8501");
  Serial.println(" 50 PARES DE POLOS");
  Serial.println( " 2 x BTS7960");
  Serial.println( " AS5600");
  Serial.println( " POSICION + FOC DE TENSION");
  Serial.println( " CON RAMPA DE SETPOINT" );
  Serial.println( "========================================" );
  Serial.println();
  Serial.print( "POLE_PAIRS = " );
  Serial.println(POLE_PAIRS,1);
  Serial.print("Kp = ");
  Serial.println( Kp, 4);
  Serial.print("VQ_MINIMO = ");
  Serial.println( VQ_MINIMO, 3);
  Serial.print("VQ_MAX = ");
  Serial.println( VQ_MAX, 3);
  Serial.print("VD = ");
  Serial.println(VD,3);
  Serial.print( "SETPOINT_RAMP_SPEED = ");
  Serial.print(SETPOINT_RAMP_SPEED,2 );
  Serial.println(" deg/s");
  Serial.print( "PWM_FREQUENCY = ");
  Serial.print(  PWM_FREQUENCY);
  Serial.println( " Hz");
  /* =====================================================
     AS5600
     ===================================================== */
  float inicial = leerAS5600();
  if (isnan(inicial))
  {
    Serial.println();
    Serial.println( "ERROR: AS5600 NO RESPONDE");
  }
  else
  {
    anguloActual =inicial;
    /*
       Al arrancar, colocar el setpoint de rampa
       en el mismo punto inicial.
       Asi no existe un salto inesperado.
    */
    setpoint = anguloActual;
    setpointObjetivo = anguloActual;
    Serial.println();
    Serial.print( "AS5600 inicial = ");
    Serial.print(inicial,6);
    Serial.println(" deg");
  }
  /* =====================================================
     COMANDOS
     ===================================================== */
  Serial.println();
  Serial.println( "COMANDOS:"
  );
  Serial.println();
  Serial.println("0...360 -> SETPOINT OBJETIVO");
  Serial.println( "C       -> CALIBRAR" );
  Serial.println("A       -> MOSTRAR ESTADO");
  Serial.println( "F       -> ACTIVAR FOC");
  Serial.println( "S       -> STOP" );
  Serial.println("+       -> AUMENTAR TEST_POWER");
  Serial.println("-       -> DISMINUIR TEST_POWER");
  Serial.println();
  Serial.println("SECUENCIA:" );
  Serial.println("1) C + ENTER");
  Serial.println("2) SETPOINT + ENTER");
  Serial.println("3) F + ENTER");
  Serial.println();
  detenerMotor();
}
/* =========================================================
   LOOP
   ========================================================= */
void loop()
{
  /* =====================================================
     SERIAL
     ===================================================== */
  procesarSerial();
  /* =====================================================
     RAMPA DE SETPOINT
     ===================================================== */
  actualizarSetpointRampa();
  /* =====================================================
     FOC
     2 ms = 500 Hz
     ===================================================== */
  static unsigned long ultimoControl = 0;
  if ( millis() -ultimoControl >= 2 )
  {
    ultimoControl = millis();
    controlPosicionFOC();
  }
  /* =====================================================
     TELEMETRIA
     Cada 100 ms
     ===================================================== */
  static unsigned long ultimoReporte = 0;
  if ( millis() - ultimoReporte >=  100)
  {
    ultimoReporte = millis();
    float thetaEPlot = calibrated ? thetaElectrical : 0.0f;
    /*
       COLUMNAS:
       1  SETPOINT OBJETIVO
       2  SETPOINT RAMPA
       3  MEC
       4  ERROR
       5  THETA_E
       6  VQ
       7  VD
       8 PWM_A
       9 PWM_B
       10  PHASE_A
       11 PHASE_B
      
    */
    Serial.print(setpointObjetivo,2);
    Serial.print(",");
    Serial.print(setpoint,2);
    Serial.print(",");
    Serial.print(anguloActual,2);
    Serial.print(",");
    Serial.print(error,2);
    Serial.print(",");
    Serial.print(thetaEPlot,2);
    Serial.print(",");
    Serial.print(Vq,4);
    Serial.print(",");
    Serial.print(VD,4);
    Serial.print(",");
    Serial.print(pwmA);
    Serial.print(",");
    Serial.print(pwmB);
    Serial.print(",");
    Serial.print(phaseACommand,4);
    Serial.print(",");
    Serial.println(phaseBCommand,4);
  }
}