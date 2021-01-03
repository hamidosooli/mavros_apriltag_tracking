#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
#include <stdlib.h> 


class test_pf
 {
 public:
test_pf(ros::NodeHandle & n_) : n(n_)
{
    vel_pub = n.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 100);
};

void move(double val)
{
    geometry_msgs::Twist msg;
            msg.linear.x = val;
            msg.linear.y =  0.0 ;
            msg.linear.z =  0.0;
            msg.angular.x = 0 ;
            msg.angular.y = 0 ;
            msg.angular.z = 0 ;
        vel_pub.publish(msg);
}

void turn(double val)
  {
      geometry_msgs::Twist msg;
     msg.linear.x = 1.0;
     msg.linear.y =  0.0;
     msg.linear.z =  0.0;
     msg.angular.x = 0.0;
     msg.angular.y = 0.0;
     msg.angular.z = val;
     vel_pub.publish(msg);
 }

  private:
  // ROS
  ros::NodeHandle n;
  ros::Publisher vel_pub ;
 } ;

int main(int argc, char **argv) {
     // ROS node
std:: cout << " MAIN " << std::endl ;
ros::init(argc, argv, "zigzag_husky_commands");
ros::NodeHandle n;
ros::NodeHandle n_priv("~");
ros::Rate loop_rate(50);

// create an object
test_pf move_robot_obj(n);

       while(ros::ok()) {
            for (int i=0 ;i <99999; i++)
        {
            std::cout << "turn left" << std::endl ;
            move_robot_obj.turn(1.2);
        }
        loop_rate.sleep();

        for (int i=0 ;i <99999; i++)
        {
            std::cout << "move forward" << std::endl ;
              move_robot_obj.move(1.0);
        }
        loop_rate.sleep();
        for (int i=0 ;i <99999; i++)
        {
            std::cout << "turn right" << std::endl ;
            move_robot_obj.turn(-1.2);
        }
        loop_rate.sleep();

        for (int i=0 ;i <99999; i++)
        {
            std::cout << "move forward" << std::endl ;
            move_robot_obj.move(1.0);
        }
         }
       return 0;
}

