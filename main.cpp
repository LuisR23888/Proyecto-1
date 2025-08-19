//**********************************************************************/
// Universidad del Valle de Guatemala
// BE3029 - Electronica Digital 2
// Luis Rodriguez
// 21/07/2025
// Proyecto 1
// MCU: ESP32 dev kit 1.1
//**********************************************************************/


#include <Arduino.h>
#include <stdint.h>
#include <driver/ledc.h>
#include "config.h"

// Definciones
#define ADCPIN 34

#define servo 25

#define LED_R 13
#define LED_V 14
#define LED_A 26
#define BTN1 0

#define pwmChannel3 3

#define freqPWMS 50


#define resPWM 16

#define delayBounce 250


// Funciones

void initBoton(void);
void IRAM_ATTR BTN1_ISR (void);

void initPWM(void);

void getADCEMA(void);


// Variables globales

AdafruitIO_Feed *tempCanal = io.feed("proyecto1-temp");

float Temperatura;
float ValorT=0;


//volatile uint32_t contador;
volatile int32_t contadorS;


volatile bool btn1Pressed;
volatile uint32_t lastISRBtn1 = 0;

float adcRawEMA = 0;
float adcFiltrado = adcRawEMA;
float alpha = 0.05;

//ISRs - Rutinas de interrupcion
void IRAM_ATTR BTN1_ISR (void){
  uint32_t tiempoActual = millis();

  if(tiempoActual - lastISRBtn1 > delayBounce){
    btn1Pressed = true;
    ValorT=Temperatura;
    if (ValorT<22.0){
      contadorS = 0;
    }
    else if (ValorT>22.0 && ValorT<25.0)
    {
      contadorS = 1;
    }
    else if (ValorT>25.0){
      contadorS = 2;
    }
    lastISRBtn1 = tiempoActual;
  }
}

//Rutinas de interrupcion


void setup() {
  // put your setup code here, to run 
  
  Serial.begin(115200);
  initBoton();

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
  
  // Inicializar estado de LEDs
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_V, LOW);
  digitalWrite(LED_A, LOW);

  initPWM();
}

void loop() { 
  io.run();
  getADCEMA();

  Serial.print("sending -> ");
  Serial.println(ValorT);
  tempCanal->save(ValorT);

  delay(3000);

  Temperatura = map(adcFiltrado, 0, 4095, 0, 30);
  Serial.println(adcRawEMA);
  Serial.println(Temperatura);


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


void initBoton(void){
  pinMode(BTN1, INPUT_PULLUP);
  attachInterrupt(BTN1, &BTN1_ISR, FALLING);

}

void initPWM(void) {
    ledcAttachPin(servo, pwmChannel3);
}

void getADCEMA(void){
  adcRawEMA = analogRead(ADCPIN);
  adcFiltrado = (alpha * adcRawEMA) + ((1.0-alpha)*adcFiltrado);
}
