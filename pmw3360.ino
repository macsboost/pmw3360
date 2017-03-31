#include <SPI.h>
#include <avr/pgmspace.h>
#include <LiquidCrystal.h>
#include "PMW3360_LCD.h"

adns_ctrl ac;

void setup() {
    Serial.begin(230400);
    ac.setup();
    delay(3000);
    
}

void loop() {
    ac.loop();
    
    //delay(100);
    //update_motion_data()
    //ac.printMotionData();
}


