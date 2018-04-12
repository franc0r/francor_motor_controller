#include "Arduino.h"

const int TIME_OUT_MS = 250;
const int TIME_OUT_BRAKE_MS = 100;


class Motorcontroller {
public:
  Motorcontroller(int pin_speed, int pin_rot, int pin_break, bool reverse = false)
  {
    _pin_speed = pin_speed;
    _pin_rot   = pin_rot  ;
    _pin_break = pin_break;

    _reverse = reverse ? -1 : 1;
  }
  ~Motorcontroller()
  { }

  void init()
  {
    //set output
    ::pinMode(_pin_speed , OUTPUT);
    ::pinMode(_pin_rot   , OUTPUT);
    ::pinMode(_pin_break , OUTPUT);

    ::digitalWrite(_pin_speed , LOW);
    ::digitalWrite(_pin_rot   , LOW);
    ::digitalWrite(_pin_break , LOW);

  }

  /**
   * @param speed : -255 .. 255;
   */
  void setSpeed(const int speed)
  {
    //set rotation
    if(speed * _reverse > 0)
    {
      ::digitalWrite(_pin_rot, true);
    }
    else
    {
      ::digitalWrite(_pin_rot, false);
    }

    //set speed
    uint8_t sp = (uint8_t)abs(speed);

    ::analogWrite(_pin_speed, sp);
  }

  void brk(const bool brk)
  {
    ::digitalWrite(_pin_break, brk);
  }


private:
  int _pin_speed;
  int _pin_rot  ;
  int _pin_break;

  int _reverse;
};


Motorcontroller g_mc_front_left (11, 10, 12, false);
Motorcontroller g_mc_back_left  (6, 10, 9, false);
Motorcontroller g_mc_front_right(5, 8, 7, true);
Motorcontroller g_mc_back_right (3, 8, 4, true);



#define SERIAL_DATA_MAX  (25)
char g_serial_brake[SERIAL_DATA_MAX];
char g_serial_speed_left[SERIAL_DATA_MAX];
char g_serial_speed_right[SERIAL_DATA_MAX];

volatile int g_serial_brake_idx = -1;
volatile int g_serial_speed_left_idx = -1;
volatile int g_serial_speed_right_idx = -1;

volatile bool g_serial_brake_rdy = false;
volatile bool g_serial_speed_rdy = false;

volatile bool g_serial_read_speed = false;
volatile bool g_serial_read_brake   = false;


int g_curr_speed_cmd_left = 0;
int g_curr_speed_cmd_right = 0;

int g_millis = 0;


int g_cnt_timeout = 0;

int g_cnt_got_speed = 0;

void setup()
{
  g_mc_front_left .init();
  g_mc_back_left  .init();
  g_mc_front_right.init();
  g_mc_back_right .init();

  Serial.begin(9600);

  Serial.println("Init");
}

void loop()
{

  if(g_serial_speed_rdy)
  {
    g_millis = ::millis();
    g_curr_speed_cmd_left = ::atoi(g_serial_speed_left);
    g_curr_speed_cmd_right = ::atoi(g_serial_speed_right);

    if(g_cnt_got_speed++ %20 == 0)
    {
      Serial.print(g_curr_speed_cmd_left);
      Serial.print(',');
      Serial.println(g_curr_speed_cmd_right);

//      Serial.println("---");
//      Serial.println(g_serial_speed_left);
//      Serial.println("---");

//      Serial.print(g_serial_speed_left);
//      Serial.print(',');
//      Serial.println(g_serial_speed_right);

    }

    g_serial_speed_rdy = false;
  }

  int tmp_mil = ::millis();
  int diff = tmp_mil - g_millis;
  if(diff < 0) //cover overflow
  {
    Serial.println("got millis overflow... cover it");
    g_millis = ::millis();
  }

  if(diff > TIME_OUT_MS)
  {
    //if(g_cnt_timeout++ %0 == 0)
    {
      Serial.println("Got Timeout");
//      Serial.print(tmp_mil);
//      Serial.print(", ");
//      Serial.println(g_millis);
    }
    g_curr_speed_cmd_left = 0;
    g_curr_speed_cmd_right = 0;
    //todo break
  }
  g_mc_front_left .setSpeed(g_curr_speed_cmd_left);
  g_mc_back_left  .setSpeed(g_curr_speed_cmd_left);
  g_mc_front_right.setSpeed(g_curr_speed_cmd_right);
  g_mc_back_right .setSpeed(g_curr_speed_cmd_right);

  //check brake
  if(g_serial_brake_rdy)
  {
    int brake_val = ::atoi(g_serial_brake);
    g_mc_front_left .brk(brake_val);
    g_mc_back_left  .brk(brake_val);
    g_mc_front_right.brk(brake_val);
    g_mc_back_right .brk(brake_val);

    g_serial_brake_rdy = false;

    //Serial.println("brake_val: ");
    //Serial.println(g_serial_brake);
  }


  return;
}


void serialEvent()
{
  //Serial.println("Serial Event");
  while (Serial.available())
  {
    // get the new byte:
    char msg = (char)Serial.read();
    if(msg == '#')
    {
      g_serial_speed_left_idx = 0;
      g_serial_speed_right_idx = -1;
      g_serial_speed_left[0] = 0;
      g_serial_speed_right[0] = 0;

      g_serial_read_speed = true;
    }
    else if(msg == 'B')
    {//start brake message
      g_serial_brake[0] = 0;
      g_serial_brake_idx = 0;
      g_serial_read_brake = true;
    }
    else if(msg == 0 && g_serial_read_speed)
    {
      g_serial_speed_right[g_serial_speed_right_idx] = msg;

      g_serial_speed_left_idx  = -1;
      g_serial_speed_right_idx = -1;
      //g_serial_idx = -1;
      //g_serial_rdy = true;
      g_serial_speed_rdy = true;

      g_serial_read_speed = false;
    }
    else if(g_serial_speed_left_idx >= 0 && g_serial_read_speed)
    {
      if(msg == ',')
      {
        g_serial_speed_right_idx = 0;
        //close up string
        g_serial_speed_left[g_serial_speed_left_idx] = 0;
      }
      else if(g_serial_speed_right_idx == -1)
      {
        g_serial_speed_left[g_serial_speed_left_idx++] = msg;
      }
      else
      {
        g_serial_speed_right[g_serial_speed_right_idx++] = msg;
      }
    }
    else if(msg == 0 && g_serial_read_brake)
    {//close up brake msg;
      g_serial_brake[g_serial_brake_idx] = msg; //close up string
      g_serial_brake_idx = -1;
      g_serial_brake_rdy = true;

      g_serial_read_brake = false;

    }
    else if(g_serial_brake_idx  >= 0 && g_serial_read_brake)
    {//fill brake msg;
      g_serial_brake[g_serial_brake_idx++] = msg;
    }

  }
}
