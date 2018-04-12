
#include "MotorController.h"

MotorController::MotorController() : _serial("/dev/ttyUSB0", apps::B_9600, apps::F_8N1)
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
    _subTwist = _nh.subscribe("cmd_vel", 1, &MotorController::sub_twist_callback, this);

    _run = true;
}

MotorController::~MotorController()
{
}

void MotorController::start(double duration)
{
   //create timer
   _loopTimer = _nh.createTimer(ros::Duration(duration), &MotorController::loop_callback, this);
   this->run();
}

void MotorController::run()
{
  if(_serial.connect() != apps::DeviceSucces)
  {
    std::cout << "unable to open serial port ... program will exit" << std::endl;
    ::exit(-1);
  }

  //start thread
  _thrd = std::thread(std::bind(&MotorController::serial_read_thrd, this));

   ros::spin();
   _run = false;
}

void MotorController::serial_read_thrd()
{
  while(_run.load())
  {
    unsigned char data = 0;
    if(_serial.receive(data, 100000) != apps::DeviceSucces)
    {
      continue;
    }
    std::cout << (char)data;
    usleep(1000);
  }
}

void MotorController::loop_callback(const ros::TimerEvent& e)
{
   //do loop stuff here!!!

}

void MotorController::sub_twist_callback(const geometry_msgs::Twist& msg)
{

  double angular = msg.angular.z;
  double linear  = msg.linear.x;

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

  std::string l = std::to_string(std::round(val_left * 255.0));
  std::string r = std::to_string(std::round(val_right * 255.0));

//  std::cout << " - - - - - " << std::endl;
//  std::cout << "right: " << r << std::endl;
//  std::cout << "left: " << l << std::endl;

  _serial.transmit(std::string('#' + l + "," + r));
}








// ------------- main ---------------
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "motor_controller_node");
    ros::NodeHandle nh("~");

    MotorController node;
    node.start();
}


