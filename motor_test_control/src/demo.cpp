#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "iostream"
#include <termios.h>
#include <stdio.h>

static struct termios old, current;

/* Initialize new terminal i/o settings */
void initTermios(int echo) 
{
  tcgetattr(0, &old); /* grab old terminal i/o settings */
  current = old; /* make new settings same as old settings */
  current.c_lflag &= ~ICANON; /* disable buffered i/o */
  if (echo) {
      current.c_lflag |= ECHO; /* set echo mode */
  } else {
      current.c_lflag &= ~ECHO; /* set no echo mode */
  }
  tcsetattr(0, TCSANOW, &current); /* use these new terminal i/o settings now */
}

/* Restore old terminal i/o settings */
void resetTermios(void) 
{
  tcsetattr(0, TCSANOW, &old);
}

/* Read 1 character - echo defines echo mode */
char getch_(int echo) 
{
  char ch;
  initTermios(echo);
  ch = getchar();
  resetTermios();
  return ch;
}

/* Read 1 character without echo */
char getch(void) 
{
  return getch_(0);
}

/* Read 1 character with echo */
char getche(void) 
{
  return getch_(1);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("keyboard_robot_control");

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr msg_string;
  msg_string = node->create_publisher<std_msgs::msg::String>("/motor_speed", 1);

  auto message = std_msgs::msg::String();
  char key;

  while(rclcpp::ok()){
    key = getch();

    switch (key)
    {
    case 'w':
      message.data = "L1050 R0050";
      //std::cout << "Frente";
      break;

    case 'a':
      message.data = "L0050 R0050";
      //std::cout << "Esquerda";
      break;

    case 's':
      message.data = "L0050 R1050";
      //std::cout << "Tras";
      break;
    
    case 'd':
      message.data = "L1050 R1050";
      //std::cout << "Direita";
      break;

    default:
      message.data = "L0000 R0000";
      //std::cout << "Nada";
      break;
    }

    msg_string->publish(message);
    key = '0';

    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();
  return 0;
}
