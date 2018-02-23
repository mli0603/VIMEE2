//******** Header Only Class for driving DRV8833 Duel Motor Driver and INA219 Current Sense x2 ***********//
/*
 *  
 */
#ifndef MOT_CTRL_H
#define MOT_CTRL_H

//#include <Wire.h>
#include <Adafruit_INA219.h>
#include "LP_filter.h"

#define DEBUG_CURR_LIM true


class  Motor
{
  private:
  const int Mot_Sleep = 7;
  const int Roll_Speed_Max = 255;                 //speed ranges from 0 - 255 for Arduino 8-bit 
  const int Samp_Freq=333;                        // Arduino UNO absolute maximum ~500hz,or 2200us before overutilization and missing deadlines  
  int pin1;
  int pin2;
  
  
  public:
  char type;
  enum{
    fwd=1,
    rev=2,
  };

  // current limiting vars
  bool limit_curr = false;
//  bool curr_thresh_trig = false;
  float current_limit = 50.0;                   // mA
  float CL_LB = current_limit*0.85;              // current limit lower bound
  byte last_PWM = 0;       
  byte PWM_slew = 1;   
  byte direct = 0;

  Adafruit_INA219 ina_;
  LP_filter filter;
  unsigned long last_CL_time;
  int debug_rate = 50;        //hz
  unsigned long last_debug_time;
  
  Motor(char type_, uint8_t addr, const int pin1_, const int pin2_)
  {
    type=type_;
    pin1=pin1_;
    pin2=pin2_;
    ina_.setAddr(addr);                   //setup INA219 current sensor I2C address, Note: Adafruit_INA219.h is modified to do this
  }

  void setup(){
    // Initialize the INA219 current sensor and set calibration for 0.1mA resolution and 400mA, 16V maximum
    ina_.begin();
    ina_.setCalibration_16V_400mA();

    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
    pinMode(Mot_Sleep, OUTPUT); 
  }

  void drive(byte speed, byte dir, bool limit_current){
    speed = (byte)constrain(speed, 0, 255);
    
    if (dir == fwd){
      analogWrite(pin1, speed);
      analogWrite(pin2,0);
    }
    else if (dir == rev){
      analogWrite(pin2, speed);
      analogWrite(pin1,0);
    }
    
    last_PWM = speed;
    direct = dir;
    limit_curr = limit_current;
    
  }

  void stop(){
    analogWrite(pin1, 0);
    analogWrite(pin2,0);

    last_PWM = 0;
    direct = 0;
    limit_curr = false;
  }

  /**
   * @param mot_ 1 specifies motor A, 2 specifies motor B
   * 
   * @returns current in mA otherwise 2^10 if error 
   * (any use in detecting open circuit? disabled for now... only valid on non-current limited motor)
   */
  float Get_Current(){
//    float shmV = ina_.getShuntVoltage_mV();   
//    if (shmV > 0.3 || shmV < -0.3 ){      
//      return filter.filt(ina_.getCurrent_mA());
//    } 
//    else 
//      return 1024;

      return filter.filt(ina_.getCurrent_mA());
  }

/**
 * @brief limits current within a range defined by current limit and current limit lower bound
 * 
 */
  void curr_limit(){

    if ( micros() > last_CL_time + int(1.0/(float)Samp_Freq*1000000)){
//      Serial.println(int(1.0/(float)Samp_Freq*1000000));
//      Serial.println(micros() - last_CL_time);                                   // check actual delay, if meeting deadlines
      last_CL_time = micros();
      float curr = Get_Current();
      
      if ( curr > current_limit && curr != 1024.0){
//        curr_thresh_trig = true;
        drive(last_PWM - PWM_slew, direct, limit_curr);

//        Serial.print("current limiting down: "); 
//        Serial.println(curr); 

      } else if( curr <  CL_LB){
        drive(last_PWM + PWM_slew, direct, limit_curr);

//        Serial.print("current limiting up: "); 
//        Serial.println(curr); 
      } else {
        // inside current range, maintain PWM
        drive(last_PWM, direct, limit_curr);
        
//        Serial.print("limit current in range: ");
//        Serial.println(curr); 
      }
      
    }
    
  }

  void debug(){   
    if ( millis() > last_debug_time + int(1.0/(float)debug_rate*1000)){
      last_debug_time = millis();
      float shuntvoltage = 0;
      float busvoltage = 0;
      float current_mA = 0;
      float loadvoltage = 0;
      float power_mW = 0;
    
      shuntvoltage = ina_.getShuntVoltage_mV();
      busvoltage = ina_.getBusVoltage_V();
      current_mA = ina_.getCurrent_mA();
      power_mW = ina_.getPower_mW();
      loadvoltage = busvoltage + (shuntvoltage / 1000);

       
//      Serial.print(busvoltage); 
//      Serial.print(" ");
//      Serial.print(shuntvoltage); 
//      Serial.print(" ");
//      Serial.print(loadvoltage); 
//      Serial.print(" ");
//      Serial.println(current_mA); 
//      Serial.print(" ");
//      Serial.println(power_mW);
      
      // comparing raw current vs processed current
//      Serial.print(shuntvoltage); 
//      Serial.print(" ");
      Serial.print(last_PWM); 
      Serial.print(" ");
      Serial.print(current_mA); 
      Serial.print(" ");
      Serial.println(Get_Current());

      
  
  //    delay(int(1.0/(float)Samp_Freq*1000));
    //  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
    //  Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
    //  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
    //  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
    //  Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
    //  Serial.println("");
    //  delay(2000);
    
    }
  }

};

#endif
