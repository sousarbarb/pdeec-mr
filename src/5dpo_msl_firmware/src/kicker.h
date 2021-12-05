#ifndef KICKER_H
#define KICKER_H

#include <Arduino.h>

#define PIN_KICKER_KICK          26  // Kicker kick signal
#define PIN_KICKER_CLEVEL        A1  // Kicker Capacitor level
#define PIN_KICKER_CHARGE_SIGNAL 9   // kicker charge signal
#define PIN_KICKER_SOLENOID      24  // Kicker Solenoid signal

const int tempo_max_kick   = 80;                    //milliseconds

const float Vcap_Min       = 25.0;
const float Vcap_Min_Med   = 50.0;
const float Vcap_Med       = 60.0;
const float Vcap_Med_Max   = 80.0;
const float Vcap_Max       = 90.0;

const int PWM_Min          = 20;                    // 7.8% de PWM (255)
const int PWM_Min_Med      = 40;                    // 11% de PWM (255)
const int PWM_Med          = 60;                    // 19% de PWM (255)
const int PWM_Med_Max      = 80;                    // 23% de PWM (255)
const int PWM_Max          = 90;                    // 27% de PWM (255)

//NOTE: the differential ampop that reads the Vcapacitors gives values from 2.5v to 3.8v
//      therefore, considering full scale for ADC, it gives 512 to 777 quantums
const float diff_amp_min         = 0.0;             //Vc = 0 ---> Vdiff = 2.5v
const float diff_amp_max         = 90.0;            //Vc = 0 ---> Vdiff = 3.8v
const float diff_amp_min_quantum = 512.0;
const float diff_amp_max_quantum = 800.0;           //approx.

class Kicker
{
public:
  float Vcap;
  
  Kicker();
  void init(void (*serialWriteFunction)(char c, int32_t v));
  void readCapacitor(); 
  void chargeCapacitor();
  void kick();
  void lowKick();
  void highKick();
  void testActivateSolenoid();
  void setKickerPulse(int32_t value); // Caution - kicker time MUST BE < 40
  void sendCLevel();

private:
  int Mode;
  long Vquantum;
  int cur_PWM;
  bool flag_charge;
  bool flag_correct_measure;  //flag that detects problems in the measure circuit.
  int time_kick;              // Caution! This time SHOULD NOT be greater than 40
  bool Sol_activate;
  void (*serialWrite)(char c, int32_t v);

  void createPWM();     // Set the PWM ratio according to the Kicker.Vcap
};

#endif
