#include "Mot_Ctrl.h"

Motor Mot_A('A', 0x40, 6, 5); 
Motor Mot_B('B', 0x44, 11, 10);

const int DRV_Sleep = 7;

enum{
  fwd=1,
  rev=2,
};

enum{
  speed_ctrl=false,
  trque_ctrl=true,
};

void setup(void) 
{
  
  Serial.begin(115200);
  while (!Serial) {
      delay(1);                             // will pause until serial console opens
  }
  
  Mot_A.setup();
  Mot_B.setup();

  Roll(fwd);    // for testing

  pinMode(LED_BUILTIN, OUTPUT);
}


unsigned long last_time=0;
bool Watchdog_LED_state=LOW;
//unsigned long loop_time;    // for measuring loop rate

void loop(void) 
{
// Watchdog LED (to detect system crash)
  if ( millis() > last_time + 1000){
    last_time = millis();
    digitalWrite(LED_BUILTIN, Watchdog_LED_state);
    Watchdog_LED_state=!Watchdog_LED_state;
  }

  // measure loop rate for Arduino performance testing
//  Serial.println(micros() - loop_time);                        // to check actual delay
//  loop_time = micros();

  
 Mot_B.debug();         // for testing

// Functions to be looped to maintain current limiting
  if (Mot_A.limit_curr){
    Mot_A.curr_limit();
  }
  if (Mot_B.limit_curr){
    Mot_B.curr_limit();
  }

}

void Roll(byte dir){
  digitalWrite(DRV_Sleep,HIGH);     // enable DRV
  if (dir == fwd ){
    Mot_A.drive(255, dir, speed_ctrl);
    Mot_B.drive(255, dir, trque_ctrl);
  } else if (dir == rev){
    Mot_A.drive(255, dir, trque_ctrl);
    Mot_B.drive(255, dir, speed_ctrl);
  }

}


void dvr_sleep(){
  digitalWrite(DRV_Sleep,LOW);
}







