/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-06-28 14:55:26
 * @LastEditors: Changhua
 * @Description: conqueror robot tank
 * @FilePath: 
 */


#include <avr/wdt.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.h"

uint8_t vitesse_courante = 0;

void setup()
{
  Application_FunctionSet.ApplicationFunctionSet_Init();
}

void loop()
{
  delay(10);
  vitesse_courante = Application_FunctionSet.ApplicationFunctionSet_Platooning(vitesse_courante);
  vitesse_courante = Application_FunctionSet.ApplicationFunctionSet_Tracking(vitesse_courante); 
  Application_FunctionSet.ApplicationFunctionSet_SensorDataUpdate();
}
