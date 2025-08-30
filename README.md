Proyecto 1 - Luis Rodríguez

Sensor de temperatura conectado a AdaFruit el cual permite observar en tiempo real el dato de temperatura recolectado por un sensor LM35.
Este se obtiene gracias a la pulsación de un botón configurado con antirebote. 

Además, en base a la lectura obtenida, se encendará una led indicando un rango de temperatura y se accionará un servo que cambiará de posición en función de la misma. 

Ahora bien, para utilizar este codigo se deben incluir las librerías adjuntas y se debe crear una documento "config.h" en la carpeta "src" con el siguiente bloque de código: 

/************************ Adafruit IO Config *******************************/

// visit io.adafruit.com if you need to create an account,
// or if you need your Adafruit IO key.
#define IO_USERNAME ""
#define IO_KEY ""

/******************************* WIFI **************************************/

#define WIFI_SSID ""
#define WIFI_PASS ""

// comment out the following lines if you are using fona or ethernet
#include "AdafruitIO_WiFi.h"

AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);

Esto es indispensable para que el mismo se conecte a la plataforma AdaFruit y el sistema funcione. 
