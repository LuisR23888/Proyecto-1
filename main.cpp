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
#include <Arduino.h>          // Librería principal de Arduino
#include <stdint.h>           // Define tipos de datos enteros
#include <driver/ledc.h>      // Librería para el control avanzado de PWM.
#include "config.h"           // Archivo para credenciales de WiFi y Adafruit IO).
#include "Display7.h"         // Archivo para las funciones del display de 7 segmentos

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

#define delayBounce 250       // Para el antirrebote del botón 

//******************************************/
// Prototipos de funciones
//******************************************/
void initBoton(void); 
void IRAM_ATTR BTN1_ISR (void); //

void initPWM(void);

void getADCEMA(void);


//******************************************/
// Variables globales
//******************************************/
AdafruitIO_Feed *tempCanal = io.feed("proyecto1-temp"); // interactuar con el "feed" de temperatura en Adafruit IO.

float ValorT=0;               // Almacena la última temperatura enviada a la nube.
volatile int32_t contadorS;   // Variable para el switch-case que controla los estados.

volatile bool btn1Pressed;    // Bandera que se activa en la ISR del botón
volatile uint32_t lastISRBtn1 = 0; // Almacena el tiempo de la última pulsación para el antirrebote.

float temperatureC;           // Almacena la temperatura actual calculada.
float alpha = 0.3;            // Factor de suavizado para el filtro EMA.
float adcFiltered = 0.0;      // Valor filtrado del ADC.
int adcRaw = 0;               // Valor sin filtrar del ADC.

int decenas = 0;              // Dígito de las decenas de la temperatura.
int unidades = 0;             // Dígito de las unidades.
int decimas = 0;              // Dígito de las décimas.

int currentDisplay = 0;       // Controla qué dígito del display se está mostrando.

//******************************************/
// ISRs Rutinas de Interrupcion
//******************************************/
// Lógica de antirrebote.
void IRAM_ATTR BTN1_ISR (void){
  uint32_t tiempoActual = millis();

  if(tiempoActual - lastISRBtn1 > delayBounce){
    btn1Pressed = true;
    lastISRBtn1 = tiempoActual;
  }
}
// Temporizador de hardware.
hw_timer_t * timer = NULL;

void IRAM_ATTR onTimer() {
  // apagar todos
  digitalWrite(display1, LOW);
  digitalWrite(display2, LOW);
  digitalWrite(display3, LOW);

  // Encender el display correspondiente y mostrar su número.
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
  // Pasar al siguiente display para el próximo ciclo.
  currentDisplay = (currentDisplay + 1) % 3;
}

void setup() {
  Serial.begin(115200); // Inicia la comunicación serial.
  initBoton();          // Configura el pin del botón y su interrupción.
  configDisplay7();     // Configura los pines del display de 7 segmentos.

  // Configuración del Timer de Hardware
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

  initPWM(); // Configura el PWM para el servo.
}

void loop() { 
  io.run();      // Mantiene la conexión con Adafruit IO activa.
  getADCEMA();   // Lee el sensor y actualiza la variable temperatureC.

  unsigned long currentMillis = millis();

  // Lógica de Visualización en Display
  int tempC = ValorT * 10;
  decenas = tempC / 100;
  unidades = (tempC / 10) % 10;
  decimas = tempC % 10;

  // Lógica del Botón
  if (btn1Pressed) {
    btn1Pressed = false;
    ValorT = temperatureC;

    Serial.print("sending -> ");
    Serial.println(ValorT);
    tempCanal->save(ValorT);

    // Clasifica la temperatura en tres rangos para el control del servo y LEDs.
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

  // Estados para Servo y LEDs
  // Actúa según el estado definido por 'contadorS'.
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
  pinMode(BTN1, INPUT_PULLUP); // Configura el pin como entrada con resistencia de pull-up interna.
  attachInterrupt(BTN1, &BTN1_ISR, FALLING);
}

void initPWM(void) {
  // Inicializa el sistema PWM (LEDC) para controlar el servo.
    ledcSetup(pwmChannel3, freqPWMS, resPWM);
    ledcAttachPin(servo, pwmChannel3);
}

void getADCEMA(void){
  // Lee el valor del ADC, aplica un filtro EMA y calcula la temperatura.
  adcRaw = analogReadMilliVolts(ADCPIN);   
  adcFiltered = alpha * adcRaw + (1 - alpha) * adcFiltered;  
  
  temperatureC = (adcFiltered / 10.0)-8;
  
  temperatureC = roundf(temperatureC * 10) / 10.0;
}


