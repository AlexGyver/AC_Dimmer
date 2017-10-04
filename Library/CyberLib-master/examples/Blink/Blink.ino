#include "CyberLib.h"

void setup()
{
  D13_Out; //Настраиваем пин D13 на выход
}

void loop()
{ 
 D13_Inv;
 delay_ms(1000);
 
}
