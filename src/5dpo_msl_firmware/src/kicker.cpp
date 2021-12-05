#include "kicker.h"

Kicker::Kicker()
{
  Mode       = 0;
  Vquantum   = 0;
  Vcap       = 0;
  cur_PWM    = 0;
  flag_charge          = false;
  flag_correct_measure = true;  // flag that detects problems in the measure circuit.
  time_kick = 0;                // Caution! This time SHOULD NOT be greater than 40
}

void Kicker::init(void (*serialWriteFunction)(char c, int32_t v))
{
  serialWrite = serialWriteFunction;
  
  //PWM Variables:
 
  int myEraser = 7;        // stop timer (0b0111)
  TCCR2B &= ~myEraser;     
  int myPrescaler = 1;     // 1 - no prescaling
  TCCR2B |= myPrescaler;

  // Initialize arduino pins
  pinMode(PIN_KICKER_KICK, OUTPUT);
  pinMode(PIN_KICKER_CHARGE_SIGNAL, OUTPUT);
  pinMode(PIN_KICKER_CLEVEL, INPUT);
  pinMode(PIN_KICKER_SOLENOID, OUTPUT);
  digitalWrite(PIN_KICKER_SOLENOID, LOW);

  flag_charge  = true;
  cur_PWM      = 0;
  time_kick    = 10;
}

void Kicker::readCapacitor()
{
  Vquantum = analogRead(PIN_KICKER_CLEVEL);  // read capacitors tension

  if(Vquantum > diff_amp_min_quantum )
  {
    float aux_toVolt   = (float) Vquantum * 0.004887;  //to volts
    Vcap = (aux_toVolt - 2.5) / (3.9 - 2.5) * 90;
    flag_correct_measure = true;
  }
  else
  {
    flag_correct_measure = false;
  }
}

void Kicker::createPWM()
{
  cur_PWM = PWM_Min;

  // MAX voltage --> STOP CHARGE
  if (Vcap >= Vcap_Max) {
    flag_charge = false;
    cur_PWM     = 0;
    return;
  }
  else
    // High tension ---> high PWM.
    if ((Vcap >= Vcap_Med_Max) && (Vcap < Vcap_Max)) {
      if (flag_charge) {
        cur_PWM     = PWM_Max;
        flag_charge = true;
      }
      else {
        //histereses
        if (Vcap >= Vcap_Med_Max) {  //not charge
          flag_charge = false;
          cur_PWM     = 0;
        }
        else {
          cur_PWM     = PWM_Max;
          flag_charge = true;
        }
      }
    } else
      //Medium to Max tension ---> PWM_Med_Max.
      if ((Vcap >= Vcap_Med) && (Vcap < Vcap_Med_Max)) {
        cur_PWM     = PWM_Med_Max;
        flag_charge = true;
      } else

        //Medium tension ---> PWM_Med_Max.
        if ((Vcap >= Vcap_Min_Med) && (Vcap < Vcap_Med)) {
          cur_PWM     = PWM_Med;
          flag_charge = true;
        } else
          //Min to Medium tension ---> PWM_Med.
          if ((Vcap >= Vcap_Min) && (Vcap < Vcap_Min_Med)) {
            cur_PWM     = PWM_Min_Med;
            flag_charge = true;
          }
  //Low tension ---> low PWM for safety reasons.
          else {
            cur_PWM     = PWM_Min_Med;
            flag_charge = true;
          }


  //If the measure system of the Vcap has problems.
  if (flag_correct_measure == false)
    cur_PWM = PWM_Min;         //should be zero but in the game the kicker will not work... therefore is the minimum for safety reasons.
}

void Kicker::chargeCapacitor()
{
  createPWM();
  
  if (flag_charge == true) { // if allowed, give order to charge
    analogWrite(PIN_KICKER_CHARGE_SIGNAL, cur_PWM);
  }
  else {
    analogWrite(PIN_KICKER_CHARGE_SIGNAL, 0);
  }
}

void Kicker::kick()
{
  analogWrite(PIN_KICKER_CHARGE_SIGNAL, 0);  // stop charge
  delay(1);

  digitalWrite(PIN_KICKER_KICK, HIGH);  // send kick order
  delay(time_kick);                     // this will set kick intensity
  digitalWrite(PIN_KICKER_KICK, LOW);   // disable kicker

  //Robot.Kicker.flag_charge = true;    // Begin Charge
}

void Kicker::lowKick()
{
  digitalWrite(PIN_KICKER_SOLENOID, HIGH);  // activate solenoid
  delay(400);
  kick();
  delay(250);
  digitalWrite(PIN_KICKER_SOLENOID, LOW);  // solenoid on natural position
  Sol_activate = false;
}

void Kicker::highKick()
{
  digitalWrite(PIN_KICKER_SOLENOID, LOW);  // solenoid on natural position
  kick();
}

void Kicker::testActivateSolenoid()
{
  digitalWrite(PIN_KICKER_SOLENOID, HIGH);  // activate solenoid
  delay(1000);
  digitalWrite(PIN_KICKER_SOLENOID, LOW);  // solenoid on natural position
}

void Kicker::setKickerPulse(int32_t value)
{
  if(value <= 40 && value >=0) {  // Caution - kicker time MUST BE < 40
    time_kick = value;
  }
  else {
    time_kick = 0;
  }
}

void Kicker::sendCLevel()
{
  (*serialWrite)('q', (int32_t) Vcap*100); // *100 (2 decimal cases)
}
