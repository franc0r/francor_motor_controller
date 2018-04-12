
#include "MotorController6wd_hack.h"

MotorController6wd::MotorController6wd() :
  _serial_left("/dev/ttyUSB0", apps::B_38400, apps::F_8N1),
  _serial_right("/dev/ttyUSB1", apps::B_38400, apps::F_8N1)
{
    //rosParam
    ros::NodeHandle privNh("~");
//    std::string string_val;
//    double      double_val;
//    int         int_val;
//    bool        bool_val;
//
//    privNh.param(         "string_val" ,    string_val,   std::string("string"));
//    privNh.param<double>( "double_val" ,    double_val,   100.0);
//    privNh.param<int>(    "int_val"    ,    int_val   ,   1.0);
//    privNh.param<bool>(   "bool_val"   ,    bool_val  ,   true);


    //init publisher
    //_pub = _nh.advertise<std_msgs::Bool>("pub_name",1);

    //inti subscriber
    _subTwist = _nh.subscribe("cmd_vel", 1, &MotorController6wd::sub_twist_callback, this);

    _run = true;
}

MotorController6wd::~MotorController6wd()
{
}

void MotorController6wd::start(double duration)
{
   //create timer
   _loopTimer = _nh.createTimer(ros::Duration(duration), &MotorController6wd::loop_callback, this);
   this->run();
}

void MotorController6wd::run()
{
  if(_serial_left.connect() != apps::DeviceSucces)
  {
    std::cout << "unable to open serial port left_mc ... program will exit" << std::endl;
    ::exit(-1);
  }

  if(_serial_right.connect() != apps::DeviceSucces)
  {
    std::cout << "unable to open serial port " << _serial_right.getDeviceName()  << " ... program will exit" << std::endl;
    ::exit(-1);
  }

  //start thread
  _thrd_left = std::thread(std::bind(&MotorController6wd::serial_read_thrd_left, this));
  _thrd_right = std::thread(std::bind(&MotorController6wd::serial_read_thrd_right, this));

   ros::spin();
   _run = false;
}

void MotorController6wd::serial_read_thrd_left()
{
  ROS_INFO("enter serial read thread left");
  while(_run.load())
  {
    unsigned char data = 0;
    if(_serial_left.receive(data, 1000) != apps::DeviceSucces)
    {
      continue;
    }
    std::cout << (char)data;
    usleep(100);
  }
}

void MotorController6wd::serial_read_thrd_right()
{
  ROS_INFO("enter serial read thread right");
  while(_run.load())
  {
    unsigned char data = 0;
    if(_serial_right.receive(data, 100000) != apps::DeviceSucces)
    {
      continue;
    }
    std::cout << (char)data;
    usleep(1000);
  }
}


void MotorController6wd::loop_callback(const ros::TimerEvent& e)
{
   //do loop stuff here!!!

}


void MotorController6wd::sub_twist_callback(const geometry_msgs::Twist& msg)
{

  double angular = msg.angular.z;
  double linear  = msg.linear.x;

//  linear *= -1.0;

  //prove min max...
  this->prove_abs_max(angular, 1.0);
  this->prove_abs_max(linear, 1.0);

//  int val_lin = (int)std::round((double)0x0fff * linear);
//  int val_rot_diff = (int)std::round((double)0x0fff * angular);

  double val_right = linear + angular;
  double val_left = linear - angular;

  //prove min max again...
  this->prove_abs_max(val_right, 1.0);
  this->prove_abs_max(val_left, 1.0);

  std::string l = std::to_string(std::round(val_left * 255.0)); //todo prove if l or r must be changend
  std::string r = std::to_string(std::round(val_right * 255.0 * -1));


  std::cout << " - - - - - " << std::endl;
  std::cout << "right: " << r << std::endl;
  std::cout << "left: " << l << std::endl;

  auto cmd_l = std::string('#' + l + "," + l);
  auto cmd_r = std::string('#' + r + "," + r);
  
  std::cout << "cmd_r : " << cmd_r << std::endl;
  std::cout << "cmd_l : " << cmd_l << std::endl;
  _serial_left.transmit(cmd_l);
  _serial_right.transmit(cmd_r);
}








// ------------- main ---------------
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "motor_controller_node");
    ros::NodeHandle nh("~");

    MotorController6wd node;
    node.start();
}


