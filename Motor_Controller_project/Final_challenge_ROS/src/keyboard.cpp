#include <ros/ros.h>
#include <std_msgs/String.h>
#include <termios.h>

int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

int main(int argc,char **argv){
    ros::init(argc,argv,"keyboard");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String> ("/key", 10);

    ros::Rate rate(100);

    std_msgs::String k;

    while (ros::ok())
    {
        // Direction
        int c = getch();   // call your non-blocking input function
        if (c == 'a'){
          k.data ='A';
        }else if (c == 'd'){
          k.data='D';
        }else{
           k.data='S';
        }


        pub.publish(k);
       // ROS_INFO(k.data);
       rate.sleep();
    }

    return 0;
}