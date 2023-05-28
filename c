#include <Arduino.h>
#include <math.h>

#define ENCODER_A       2 // Amarillo
#define ENCODER_B       3 // Verde
const int16_t SW1 = 7; 
const int16_t SW2 = 8; 
const int16_t SW3 = 9;

int16_t Vbotones[] = {SW1, SW2, SW3};
int16_t len_button = sizeof(Vbotones)/sizeof(int16_t);

const int16_t ENA = 6;
const int16_t IN1 = 5;
const int16_t IN2 = 4;
#define Pot_sp          A0 // Potenciómetro
// volatile
volatile int contadorA = 0;
volatile int contadorB = 0;
//timers
unsigned int now = 0;           // marca de tiempo actual
unsigned int last = 0;          // marca de tiempo anterior
unsigned int last_rele = 0;

/******CONDICIONES INICIALES*****/
float sp = 40;                    // set point
float rev = 0;
float rpm = 0;
float rpm_max = 0;
const int16_t t_muestreo = 50;     // tiempo de muestreo (ms)
unsigned int T_u = 1000;
boolean bandera_rele = false;
boolean bandera_inicio_rele = false;
int PWM = 96;
// rele
float a = 0;
float a1 = 100;
int h = 0;
int PWM_rele = 96;
int16_t i = 0;
int16_t j = 1;
int16_t k = 1;
int16_t l = 1;
/****Control PID*/
float cv;
float cv1;
float error;
float error1;
float error2;
// float Kp = 0.76;
// float Ki = 60;
// float Kd = 0.01;
float Ku;
float Kp = 1.07;
float Ki = 2.15;
float Kd = 0.54;
// float Kc = 1.07;
// float Ti = 2.15;
// float Td = 0.54;
float Tm = 0.1;
float Tu = 1000;

/****Declaración de funciones****/

void interrupcionA();
void interrupcionB();
void Adelante();
void Atras();
void Parar();
void Enviar_valores();
void PID();
void Rele();

void setup() {
  Serial.begin(9600);
  // Potenciómetro Set point
  pinMode (Pot_sp, INPUT);
  //Encoders como entradas
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  //Pulsadores
  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT_PULLUP);
  //Configura Motor
  // Declaramos todos los pines como salidas
  pinMode (ENA, OUTPUT);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);

  attachInterrupt(0,interrupcionA,CHANGE); // flanco de subida ENCODER_A
  attachInterrupt(1,interrupcionB,CHANGE); // flanco de subida ENCODER_B

  last = millis();
}

void loop() {

  // Cada periodo de muestreo
  now = millis();
  if (now - last > t_muestreo){
    rev = (float)(contadorA + contadorB) / (2.0f * 494.0f * 2.0f);
    rpm = (rev * 60.0f) * 1000.0f / t_muestreo;
    a = rpm;
    if (rpm > rpm_max && bandera_rele) {
      rpm_max = rpm;
    }

    Enviar_valores();

    contadorA = 0;
    contadorB = 0;    
    last = millis();
  }
  switch (j){
    case 1:
    // Potenciómetro varía sp
    sp = (analogRead(Pot_sp)) * (105.0 / 1023);      // referencia (max 105)
    break;
    case 2:
    // Potenciómetro varía PWM
    PWM = (analogRead(Pot_sp)) * (255.0 / 1023.0);      // referencia (max 255)
    analogWrite (ENA, PWM); //Velocidad motor
    break;
    case 3:
    j=1;
    break;
  }

  switch (k){
    case 1:
    // PID
    PID();
    break;
    case 2:
    bandera_rele = true;
    bandera_inicio_rele = true;
    Rele();
    break;
    case 3:
    // NADA
    break;
    case 4:
    k=1;
    break;
  }

  // GIRO
  switch (l){
    case 1:
    Parar();
    break;
    case 2:
    Adelante();
    break;
    case 3:
    Atras();
    break;
    case 4:
    l=1;
    break;
  }

  // Pulsador
  if(! digitalRead(SW1)){
    j++;
  }
  else if(! digitalRead(SW2)){
    k++;
  }
  else if(! digitalRead(SW3)){
    l++;
  }
}

// FUNCIONES

void interrupcionA() {
  contadorA++;
}

void interrupcionB() {
  contadorB++;
}

void Adelante()
{
  digitalWrite (IN1, HIGH);
  digitalWrite (IN2, LOW);
}

void Atras()
{
  digitalWrite (IN1, LOW);
  digitalWrite (IN2, HIGH);
}

void Parar()
{
 digitalWrite (IN1, LOW);
 digitalWrite (IN2, LOW);
}

void Enviar_valores()
{
  // Serial.print("rpm: ");
  Serial.println(rpm);
  // Serial.print("sp: ");
  Serial.println(sp);
  // Serial.print("PWM: ");
  Serial.println(PWM);
  Serial.println(PWM_rele);
  Serial.println(a1);
  // Serial.print("j, k, l: ");
  Serial.println(j);
  Serial.println(k);
  Serial.println(l);
  Serial.println(Kp);
  Serial.println(Ki);
  Serial.println(Kd);
  // Serial.println(Kc);
  // Serial.println(Ti);
  // Serial.println(Td);
  // Serial.println("----");
}

void PID()
{
  analogWrite (ENA, PWM); //Velocidad motor
  error = sp - rpm;
  /***Ecuación en diferencias***/
  cv = cv1 + (Kp + Kd/Tm)*error + (-Kp +Ki*Tm - 2*Kd/Tm)*error1 + (Kd/Tm)*error2;
  // cv = cv1 + Kc*(error - error1 + (Tm/Ti) * error + (Td/Tm)*(error - 2*error1 + error2));
  cv1=cv;
  error2 = error1;
  error1 = error;
  /**Saturamos la salida del PID**/
  if (cv > 105.0){
    cv = 105.0;
  }
  if (cv < 45){
    cv = 45;
  }
  PWM = (cv) * (255.0 / 105.0);
  PWM_rele = PWM;         // Guardo el último valor de PWM
}

// void Rele()
// {
//   if (now - last_rele > T_u){
//     i++;
//     if (i % 2 == 0){
//       Adelante();
//       last_rele = millis();
//     } else {
//       Atras();
//       last_rele = millis();
//     }
//   }
//   if (i>6) {
//     i = 0;
//     // bandera_rele = false;
//   }
//   Serial.println("Rele");

// }

void Rele()
{
  if (PWM_rele==255){
    h = 40;
  }
  if (PWM_rele>175){
    h = 255.0 - PWM_rele;
  }
  if (PWM_rele<175 && PWM_rele>96){
    h = 40.0;
  }
  if (PWM_rele<96){
    PWM_rele = 96;
    h = 40.0;
  }
  if (i==1 && (now - last_rele > 0.7 * T_u)){
    a1 = rpm;
  }
  if (now - last_rele > T_u){
    i++;
    if (i % 2 == 0){
      // if (rpm>= 0.95*a1 && now - last_rele<T_u && now - last_rele>2*t_muestreo){
      if (rpm>= 0.95*a1 && (now - last_rele < T_u)){
        T_u = now - last_rele;
      }
      // ALTO
      PWM = PWM_rele + h;
      analogWrite (ENA, PWM);
    } else {
      // BAJO
      PWM = PWM_rele - h;
      analogWrite (ENA, PWM);
    }
    last_rele = millis();
  }
  if (i>6) {
    i = 0;
    bandera_rele = false;
    k=1;
  }
  // -------------------

  // if (now - last_rele > T_u){
  //   last_rele = millis();
  // }

  // while(rpm<0.95*a){
  //   analogWrite (ENA, PWM_rele + h);
  // }
  // while(rpm<0.95*a){
  //   analogWrite (ENA, PWM_rele - h);
  // }
  // while(rpm<0.95*a){
  //   analogWrite (ENA, PWM_rele + h);
  // }
  // while(rpm<0.95*a){
  //   analogWrite (ENA, PWM_rele - h);
  //   // k=1;                        //Para pasar a PID
  // }
  Ku = 4*h/(a*M_PI);
  // Kc = Ku/2;
  // Ti = Tu/(2*1000);
  // Td = Tu/(8*1000);
  // Kp = 0.6 * Ku / 8;
  // Ti = T_u*2/(2*1000);
  // Td = T_u*2/(8*1000);
  // Ki = T_u*2/(2*1000);
  // Kd = T_u*2/(12*1000);
  // Kd = Kp *(8*1000) / T_u*2;
  // Ki = Kp *(2*1000) / T_u*2;
}
