
#ifndef MOTORCONTROLLER_H_
#define MOTORCONTROLLER_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <thread>
#include <mutex>
#include <atomic>

#include "SerialCom.h"

namespace serial{
enum serial_format {
      START = 0,
      HI_RIGHT,
      LO_RIGHT,
      HI_LEFT,
      LO_LEFT,
      SIZE
     };
}
class MotorController
{

public:
  MotorController();
    virtual ~MotorController();

    /**
     *
     * @brief
     *
     * @return  void
     */
    void start(double duration = 0.01);

private:    //functions

    /**
     *
     * @brief this function containts the main working loop
     *
     * @param[in,out]  void
     *
     * @return 		   void
     */
    void run();

    void serial_read_thrd();

    void loop_callback(const ros::TimerEvent& e);

    //void subCallback(const ROS_PACK::MESSAGE& msg);
    void sub_twist_callback(const geometry_msgs::Twist& msg);

    void prove_abs_max(double& var, const double max_val)
    {
      if(std::abs(var) > max_val)
      {
        var = max_val * (std::abs(var) / var);
      }
    }

    void prove_abs_max(int& var, const int max_val)
    {
      if(std::abs(var) > max_val)
      {
        var = max_val * (std::abs(var) / var);
      }
    }

private:    //dataelements
    ros::NodeHandle _nh;

//    ros::Publisher _pub;
    ros::Subscriber _subTwist;

    ros::Timer _loopTimer;

    apps::SerialCom _serial;

    std::atomic<bool> _run;

    std::thread _thrd;

};

#endif /* MOTORCONTROLLER_H_ */
