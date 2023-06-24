/*   
modified on Sep 21, 2020
Modified by MohammedDamirchi from https://github.com/sparkfun/HX711-Load-Cell-Amplifier base on https://www.instructables.com/id/Arduino-Scale-With-5kg-Load-Cell-and-HX711-Amplifi/
Home<iframe class="wp-embedded-content" sandbox="allow-scripts" security="restricted" style="position: absolute; clip: rect(1px, 1px, 1px, 1px);" title="&#8220;Home&#8221; &#8212; Electropeak" src="https://electropeak.com/learn/embed/#?secret=Nlkd2ttKnf" data-secret="Nlkd2ttKnf" width="600" height="338" frameborder="0" marginwidth="0" marginheight="0" scrolling="no"></iframe> 
*/ 


#include "HX711.h"

#define DOUT  5
#define CLK  6

byte command ,b1,b2,b3,b4;
HX711 scale;

float calibration_factor = -57500; //Change this for calibration your load cell

void setup() {
  Serial.begin(115200);

  scale.begin(DOUT, CLK);
  scale.set_scale();
  scale.tare(); //Reset the scale to 0
  
  long zero_factor = scale.read_average(); //Get a baseline reading
  
  scale.set_scale(calibration_factor); //Adjust to this calibration factor
}

void loop() {

  send_loadcell_force_to_stm32((int32_t)(scale.get_units(1)*1000.0 + random(-5, 5) ));
  delay(1);
  
//  scale.set_scale(calibration_factor); //Adjust to this calibration factor
  //Serial.print("Reading: ");
 // Serial.print((scale.get_units(5)), 4);
  // Serial.print(" kg"); //Change this to kg and re-adjust the calibration factor if you follow SI units like a sane person
  // Serial.print(" calibration_factor: ");
  // Serial.print(calibration_factor);
 // Serial.println();

//  if(Serial.available())
//  {
//    char temp = Serial.read();
//    if(temp == '+' || temp == 'a')
//      calibration_factor += 10;
//    else if(temp == '-' || temp == 'z')
//      calibration_factor -= 10;
//  }
} 

void send_loadcell_force_to_stm32(int32_t a)
{
  b1 = (byte)((a >> 24) & 0xFF);
  b2 = (byte)((a >> 16) & 0xFF);
  b3 = (byte)((a >> 8) & 0xFF);
  b4 = (byte)((a) & 0xFF);
  command = 1;
  byte cmd[]={0xAA ,0xAA,command, b1 , b2 , b3 , b4 ,0xBB};
  Serial.write(cmd,sizeof(cmd));
}
