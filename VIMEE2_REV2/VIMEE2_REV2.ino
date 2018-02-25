/* VIMEE V2
// Communication based on ros bridge
// to start communication, type in command "rosrun rosserial_python serial_node.py /dev/ttyACM0"


todo:
setup actual watchdog
pseudo multithread?
|-> finish async Rx I2C


*/



#include "Mot_Ctrl.h"
#include <ros.h>
#include <std_msgs/Int8.h>

//#define DEBUG_PERF

/* param type, I2C address, pin1, pin2, curent_lim */
Motor Mot_A('A', 0x40, 6, 5, 69.0); 
Motor Mot_B('B', 0x44, 11, 10, 60.0);

// ROS Motors
std_msgs::Int8 motors_msg;
void motorsCallback( const std_msgs::Int8& toggle_msg){
  switch(toggle_msg.data) {
    case 0 :                         
      Mot_A.stop(); 
      Mot_B.stop();   
      dvr_sleep();
//      Serial.println("stopping motors");
		break;
    case 1 :			
      digitalWrite(DRV_Sleep,HIGH);     // enable DRV
//      Roll_dir_state = fwd;                 
      Roll(fwd);                      
//      Serial.println("calling roll fwd");
		break;
    case 2 :					
      digitalWrite(DRV_Sleep,HIGH);     // enable DRV
//      Roll_dir_state = rev;		          
      Roll(rev);                      
//      Serial.println("calling roll rev");
		break;
    
  }
    
}

ros::Subscriber<std_msgs::Int8> motors("motors", &motorsCallback );


void setup(void) 
{
  #ifdef DEBUG_MOT
    Serial.begin(115200);
    while (!Serial) {
        delay(1);                             // will pause until serial console opens
    }
  #endif

// Enable internal pullups for twi. Necessary for non-MegaX28 brds as I2C/Wire library doesn't support 
// Mega2560 
  #if defined(__AVR_ATmega2560__)
      pinMode(SDA, INPUT);    
      pinMode(SCL, INPUT);
      digitalWrite(SDA, 1);
      digitalWrite(SCL, 1);
//      Serial.println("I'm a mega with I2C pins: ");
//      Serial.println(SDA);    //20 
//      Serial.println(SCL);    //21 
  #endif

// Due 
  #if defined(__SAM3X8E__)
    pinMode(SDA, INPUT);    
    pinMode(SCL, INPUT);
    digitalWrite(SDA, 1);
    digitalWrite(SCL, 1);
//    Serial.println("I'm a due with I2C pins: ");
//    Serial.println(SDA);    //IDK  checkme
//    Serial.println(SCL);    //IDK  checkme
  #endif

  ROS_setup();
  sensors_setup();
  
  Mot_A.setup();
  Mot_B.setup();
  Mot_A.stop();
  Mot_B.stop();

  // for watchdog LED
  pinMode(LED_BUILTIN, OUTPUT);

}

// Communication
byte incomingByte;

// Maintenance and Performance
unsigned long last_time = 0;
bool Watchdog_LED_state = LOW;
unsigned long loop_time;    // for measuring loop rate

void loop(void) 
{
// Watchdog Blinker (to detect system crash [happens often])
  if ( millis() > last_time + 1000){
    last_time = millis();
    digitalWrite(LED_BUILTIN, Watchdog_LED_state);
    Watchdog_LED_state=!Watchdog_LED_state;
  }

  sensors_loop();
  Mot_A.ctrl_loop();
  Mot_B.ctrl_loop();


  // measure loop rate for Arduino performance testing
  #ifdef DEBUG_PERF
    Serial.println(micros() - loop_time);                        // to check actual delay
    loop_time = micros();
  #endif 
  
  #ifdef DEBUG_MOT
  
  // debug plot
    #ifdef DEBUG_PLOT_B
      if (!debug_called){
        Roll(fwd); 
        Mot_B.debug_enable = true;
        debug_called = true;
      }
    #endif
    #ifdef DEBUG_PLOT_A
      if (!debug_called){
        Roll(rev); 
        Mot_A.debug_enable = true;
        debug_called = true;
      }
    #endif
    
    if (Mot_A.debug_enable == true)
  	  Mot_A.debug();         // for printing drive PWM, I [mA] filtered I
    else if (Mot_B.debug_enable == true)
  	Mot_B.debug();         

  // serial commands for motor control
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    Serial.print(incomingByte-48, DEC);
	  Serial.print(' ');
    
    switch(incomingByte-48) {
      case 0 :                         // 48 = ascii for '0'
        Mot_A.stop(); 
        Mot_B.stop();   
        dvr_sleep();
        Roll_dir_state = 0;
        Serial.println("stopping motors");
		break;
      case 1 :			
        digitalWrite(DRV_Sleep,HIGH);     // enable DRV
        Roll_dir_state = fwd;                 
        Roll(fwd);                      
        Serial.println("calling roll fwd");
		break;
      case 2 :					
        digitalWrite(DRV_Sleep,HIGH);     // enable DRV
        Roll_dir_state = rev;		          
        Roll(rev);                      
        Serial.println("calling roll rev");
		break;
	case 3:							
		  Mot_B.debug_enable = true;
		  Mot_A.debug_enable = false;
		  Serial.println("enabled debug mot B");
		  break;
	  case 4:
		  Mot_A.debug_enable = true;           
		  Mot_B.debug_enable = false;
		  Serial.println("enabled debug mot A");
		  break;
	  case 5:
		  Mot_B.debug_enable = false;           
		  Mot_A.debug_enable = false;
		  Serial.println("disable debug");
		  break;
    case 6:                                
      Mot_A.ina_.scanBus();
      Serial.println("Scan I2C bus for devices complete");
      break;
    }

  }
  
  #endif

}

void Roll(int8_t dir){
  if (dir == fwd ){
    Mot_A.drive(Default_speed, Mot_Polarity*dir, speed_ctrl);
    Mot_B.drive(Default_speed, Mot_Polarity*dir, trque_ctrl);
  } else if (dir == rev){
    Mot_A.drive(Default_speed, Mot_Polarity*dir, trque_ctrl);
    Mot_B.drive(Default_speed, Mot_Polarity*dir, speed_ctrl);
  }
}













