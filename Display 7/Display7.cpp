#include "Display7.h"

#include <Arduino.h>

#include <stdint.h>

#define dA 5
#define dB 19
#define dC 16
#define dD 0
#define dE 2
#define dF 18
#define dG 17
#define dP 4

void configDisplay7(void){
    pinMode(dA, OUTPUT);
    pinMode(dB, OUTPUT);
    pinMode(dC, OUTPUT);
    pinMode(dD, OUTPUT);
    pinMode(dE, OUTPUT);
    pinMode(dF, OUTPUT);
    pinMode(dG, OUTPUT);
    pinMode(dP, OUTPUT);

    //CATODO COMUN
    digitalWrite(dA, LOW);
    digitalWrite(dB, LOW);
    digitalWrite(dC, LOW);
    digitalWrite(dD, LOW);
    digitalWrite(dE, LOW);
    digitalWrite(dF, LOW);
    digitalWrite(dG, LOW);
    digitalWrite(dP, LOW);


}

void desplegarNumero(uint8_t numero){
    switch(numero){
    case 0: // 0
        digitalWrite(dA, HIGH);
        digitalWrite(dB, HIGH);
        digitalWrite(dC, HIGH);
        digitalWrite(dD, HIGH);
        digitalWrite(dE, HIGH);
        digitalWrite(dF, HIGH);
        digitalWrite(dG, LOW);
        break;

    case 1: // 1
        digitalWrite(dA, LOW);
        digitalWrite(dB, HIGH);
        digitalWrite(dC, HIGH);
        digitalWrite(dD, LOW);
        digitalWrite(dE, LOW);
        digitalWrite(dF, LOW);
        digitalWrite(dG, LOW);
        break;

    case 2: // 2
        digitalWrite(dA, HIGH);
        digitalWrite(dB, HIGH);
        digitalWrite(dC, LOW);
        digitalWrite(dD, HIGH);
        digitalWrite(dE, HIGH);
        digitalWrite(dF, LOW);
        digitalWrite(dG, HIGH);
        break;

    case 3: // 3
        digitalWrite(dA, HIGH);
        digitalWrite(dB, HIGH);
        digitalWrite(dC, HIGH);
        digitalWrite(dD, HIGH);
        digitalWrite(dE, LOW);
        digitalWrite(dF, LOW);
        digitalWrite(dG, HIGH);
        break;

    case 4: // 4
        digitalWrite(dA, LOW);
        digitalWrite(dB, HIGH);
        digitalWrite(dC, HIGH);
        digitalWrite(dD, LOW);
        digitalWrite(dE, LOW);
        digitalWrite(dF, HIGH);
        digitalWrite(dG, HIGH);
        break;

    case 5: // 5
        digitalWrite(dA, HIGH);
        digitalWrite(dB, LOW);
        digitalWrite(dC, HIGH);
        digitalWrite(dD, HIGH);
        digitalWrite(dE, LOW);
        digitalWrite(dF, HIGH);
        digitalWrite(dG, HIGH);
        break;

    case 6: // 6
        digitalWrite(dA, HIGH);
        digitalWrite(dB, LOW);
        digitalWrite(dC, HIGH);
        digitalWrite(dD, HIGH);
        digitalWrite(dE, HIGH);
        digitalWrite(dF, HIGH);
        digitalWrite(dG, HIGH);
        break;

    case 7: // 7
        digitalWrite(dA, HIGH);
        digitalWrite(dB, HIGH);
        digitalWrite(dC, HIGH);
        digitalWrite(dD, LOW);
        digitalWrite(dE, LOW);
        digitalWrite(dF, LOW);
        digitalWrite(dG, LOW);
        break;

    case 8: // 8
        digitalWrite(dA, HIGH);
        digitalWrite(dB, HIGH);
        digitalWrite(dC, HIGH);
        digitalWrite(dD, HIGH);
        digitalWrite(dE, HIGH);
        digitalWrite(dF, HIGH);
        digitalWrite(dG, HIGH);
        break;

    case 9: // 9
        digitalWrite(dA, HIGH);
        digitalWrite(dB, HIGH);
        digitalWrite(dC, HIGH);
        digitalWrite(dD, HIGH);
        digitalWrite(dE, LOW);
        digitalWrite(dF, HIGH);
        digitalWrite(dG, HIGH);
        break;

    default: // Apagado
        digitalWrite(dA, LOW);
        digitalWrite(dB, LOW);
        digitalWrite(dC, LOW);
        digitalWrite(dD, LOW);
        digitalWrite(dE, LOW);
        digitalWrite(dF, LOW);
        digitalWrite(dG, LOW);
        break;
    }

}

void desplegarPunto(uint8_t punto){
    if(punto == 1){
        digitalWrite(dP, HIGH);
    }else{
        digitalWrite(dP, LOW);
    }
}