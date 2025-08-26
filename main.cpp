//**********************************************************************/
// Universidad del Valle de Guatemala
// BE3029 - Electronica Digital 2
// Luis Rodriguez
// 21/07/2025
// Proyecto 1
// MCU: ESP32 dev kit 1.1
//**********************************************************************/

//******************************************/
// Librerias
//******************************************/
#include <Arduino.h>
#include <stdint.h>
#include <driver/ledc.h>
#include "config.h"
#include "Display7.h"

//******************************************/
// Definiciones
//******************************************/
#define display1 21
#define display2 22
#define display3 23

#define ADCPIN 32
#define servo 25

#define LED_R 13
#define LED_V 12
#define LED_A 27
#define BTN1 15

#define pwmChannel3 3
#define freqPWMS 50
#define resPWM 16

#define delayBounce 250

//******************************************/
// Prototipos de funciones
//******************************************/
void initBoton(void);
void IRAM_ATTR BTN1_ISR (void);

void initPWM(void);

void getADCEMA(void);


//******************************************/
// Variables globales
//******************************************/
AdafruitIO_Feed *tempCanal = io.feed("proyecto1-temp");

float ValorT=0;
volatile int32_t contadorS;

volatile bool btn1Pressed;
volatile uint32_t lastISRBtn1 = 0;

float temperatureC;
float alpha = 0.3;
float adcFiltered = 0.0;
int adcRaw = 0;

int decenas = 0;
int unidades = 0;
int decimas = 0;

int currentDisplay = 0;

//******************************************/
// ISRs Rutinas de Interrupcion
//******************************************/
void IRAM_ATTR BTN1_ISR (void){
  uint32_t tiempoActual = millis();

  if(tiempoActual - lastISRBtn1 > delayBounce){
    btn1Pressed = true;
    lastISRBtn1 = tiempoActual;
  }
}

hw_timer_t * timer = NULL;

void IRAM_ATTR onTimer() {
  // apagar todos
  digitalWrite(display1, LOW);
  digitalWrite(display2, LOW);
  digitalWrite(display3, LOW);

  switch(currentDisplay) {
    case 0:
      digitalWrite(display1, HIGH);
      desplegarNumero(decenas);
      desplegarPunto(0);
      break;
    case 1:
      digitalWrite(display2, HIGH);
      desplegarNumero(unidades);
      desplegarPunto(1);
      break;
    case 2:
      digitalWrite(display3, HIGH);
      desplegarNumero(decimas);
      desplegarPunto(0);
      break;
  }
  currentDisplay = (currentDisplay + 1) % 3;
}

void setup() {
  Serial.begin(115200);
  initBoton();
  configDisplay7();

  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000, true);
  timerAlarmEnable(timer);
  
  // wait for serial monitor to open
  while(! Serial);

  Serial.print("Connecting to Adafruit IO");

  // connect to io.adafruit.com
  io.connect();

  // wait for a connection
  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  // we are connected
  Serial.println();
  Serial.println(io.statusText());


  // Salidas
  pinMode(LED_R, OUTPUT);
  pinMode(LED_V, OUTPUT);
  pinMode(LED_A, OUTPUT);

  pinMode(display1, OUTPUT);
  pinMode(display2, OUTPUT);
  pinMode(display3, OUTPUT);
  
  // Inicializar estado de LEDs
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_V, LOW);
  digitalWrite(LED_A, LOW);

  digitalWrite(display1, LOW);
  digitalWrite(display2, LOW);
  digitalWrite(display3, LOW);

  initPWM();
}

void loop() { 
  io.run();
  getADCEMA();

  unsigned long currentMillis = millis();

  int tempC = ValorT * 10;
  decenas = tempC / 100;
  unidades = (tempC / 10) % 10;
  decimas = tempC % 10;

  if (btn1Pressed) {
    btn1Pressed = false;
    ValorT = temperatureC;

    Serial.print("sending -> ");
    Serial.println(ValorT);
    tempCanal->save(ValorT);

    if (ValorT < 22.0) {
    contadorS = 0;
    }
    else if (ValorT >= 22.0 && ValorT < 25.0) {
      contadorS = 1;
    }
    else if (ValorT >= 25.0) {
      contadorS = 2;
    }
  }
  
  switch(contadorS){
    case 0:
    ledcWrite(pwmChannel3, 3277);
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_V, LOW);
    digitalWrite(LED_A, LOW);
    break;

    case 1:
    ledcWrite(pwmChannel3, 4096);
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_V, HIGH);
    digitalWrite(LED_A, LOW);
    break;

    case 2:
    ledcWrite(pwmChannel3, 4915);
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_V, LOW);
    digitalWrite(LED_A, HIGH);
    break;
  }
}
//******************************************/
// Otras funciones
//******************************************/
void initBoton(void){
  pinMode(BTN1, INPUT_PULLUP);
  attachInterrupt(BTN1, &BTN1_ISR, FALLING);
}

void initPWM(void) {
    ledcSetup(pwmChannel3, freqPWMS, resPWM);
    ledcAttachPin(servo, pwmChannel3);
}

void getADCEMA(void){
  adcRaw = analogReadMilliVolts(ADCPIN);   
  adcFiltered = alpha * adcRaw + (1 - alpha) * adcFiltered;  
  
  temperatureC = (adcFiltered / 10.0)-8;
  
  temperatureC = roundf(temperatureC * 10) / 10.0;
}

