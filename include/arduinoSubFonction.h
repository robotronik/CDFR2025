#pragma once 
#include "constante.h"
#include "commandesAsservissement.hpp"
#include "config.h"
#include "arduino.hpp"


int pullpush(Arduino* arduino);
int catchPlant(Arduino* arduino);
int catchPlant2(Arduino* arduino);
int FastReleasePlant(Arduino* arduino);
int releasePlant(Arduino* arduino);
void blinkLed(Arduino* arduino,int LedNb,int periode);

