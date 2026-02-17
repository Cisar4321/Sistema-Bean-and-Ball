#include <Servo.h>

Servo miServo;
const int pinServo = 9;
const int pinIR = A0;

float Kp = 3.5152;
float Ki = 5.0719;
float Kd = 1.38654;

float setpoint = 20.0;
float error = 0, error_ant = 0, integral = 0, derivada = 0;

const int SERVO_MIN = 60;
const int SERVO_MAX = 100;
const int SERVO_EQ = 80;
const float INTEGRAL_MAX = 15.0;

float dist_filtrada = 0.0;
const float alpha = 0.6;

unsigned long t_anterior = 0;
const float Ts = 0.05;

float leerDistancia() {
  long suma = 0;
  for (int i = 0; i < 5; i++) {
    suma += analogRead(pinIR);
  }
  float adc = suma / 5.0;
  float dist = 17569.7 * pow(adc, -1.2062);
  dist_filtrada = alpha * dist + (1.0 - alpha) * dist_filtrada;
  return dist_filtrada;
}

void setup() {
  Serial.begin(9600);
  miServo.attach(pinServo);
  miServo.write(SERVO_EQ);

  for (int i = 0; i < 20; i++) {
    leerDistancia();
    delay(10);
  }

  t_anterior = millis();
}

void loop() {
  if (millis() - t_anterior >= (Ts * 1000)) {
    t_anterior = millis();

    float distancia = leerDistancia();

    error = setpoint - distancia;

    integral += error * Ts;
    if (integral > INTEGRAL_MAX) integral = INTEGRAL_MAX;
    if (integral < -INTEGRAL_MAX) integral = -INTEGRAL_MAX;

    derivada = (error - error_ant) / Ts;
    error_ant = error;

    float salida_pid = Kp * error + Ki * integral + Kd * derivada;

    if (abs(error) < 1.0) {
      salida_pid = 0;
      integral = 0;
    }

    int angulo = SERVO_EQ - (int)salida_pid;

    if (angulo > SERVO_MAX) angulo = SERVO_MAX;
    if (angulo < SERVO_MIN) angulo = SERVO_MIN;

    miServo.write(angulo);

    Serial.print("Setpoint:");
    Serial.print(setpoint);
    Serial.print("\t");
    Serial.print("Distancia:");
    Serial.print(distancia);
    Serial.print("\t");
    Serial.print("Angulo:");
    Serial.println(angulo);
  }
}