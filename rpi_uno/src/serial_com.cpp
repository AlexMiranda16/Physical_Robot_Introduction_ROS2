// C library headers
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include "string.h"

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

//ROS2 headers
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


int serial_port;
std_msgs::msg::String left;// = std_msgs::msg::String();
std_msgs::msg::String right;// = std_msgs::msg::String();

uint8_t HexNibbleToByte(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  else if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  else return 0;
}

int getMessageValue(char *message){
    int value = (HexNibbleToByte(message[0]) << 12) +
                (HexNibbleToByte(message[1]) << 8)  +
                (HexNibbleToByte(message[2]) << 4)  +
                 HexNibbleToByte(message[3]);

    return value;
}

void receiveMessage(int serial_port){

    int n;
    char channel;
    char msg[4];
    int value;

    while(true){
            
        //Reads first letter and checks if it is a valid one
        n = read(serial_port, &channel, 1);
        if ( (channel >= 'g') && (channel <= 'z') && (n == 1) ){
            
            printf("%c",channel);
            //Stores the 4 bytes of the message
            for(int i = 0; i<4; i++){
                read(serial_port, &msg[i], 1);
                printf("%d", msg[i]);
            }
            printf("     ");

            //Converts the message to a value
            value = getMessageValue(msg);
            break;
        }
    }
}

//Subscriber receives the Values to insert on both motors
void motorCallback(const std_msgs::msg::String::SharedPtr msg){

    //printf("%s\n", msg->data.c_str());
 
    char left[6];
    char right[6];

    //Separates the left and right motor message
    strcpy(left, msg->data.substr(0,5).c_str());
    strcpy(right, msg->data.substr(6,5).c_str());

    write(serial_port, left, 5);
    write(serial_port, right, 5);
}


int main(int argc, char **argv){

    //Opening the serial port device (Arduino)
    serial_port = open("/dev/ttyUNO", O_RDWR);

    if (serial_port < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
        return -1;
    }

    struct termios tty;

    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return -2;
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    //tty.c_cflag |= PARENB;  // Set parity bit, enabling parity
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    //tty.c_cflag |= CSTOPB;  // Set stop field, two stop bits used in communication
    tty.c_cflag &= ~CSIZE; // Clear all the size bits, then use one of the statements below
    //tty.c_cflag |= CS5; // 5 bits per byte
    //tty.c_cflag |= CS6; // 6 bits per byte
    //tty.c_cflag |= CS7; // 7 bits per byte
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    //tty.c_cflag |= CRTSCTS;  // Enable RTS/CTS hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)


    tty.c_lflag &= ~ICANON; // Disable Canonical input mode
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP


    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    // convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    // no XON/XOFF software flow control
    tty.c_iflag &= ~(INPCK);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes


    // Output flags - Turn off output processing
    //
    // no CR to NL translation, no NL to CR-NL translation,
    // no NL to CR translation, no column 0 CR suppression,
    // no Ctrl-D suppression, no fill characters, no case mapping,
    // no local output processing
    //
    // config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
    //                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
    tty.c_oflag = 0;

//    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
//    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT IN LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT IN LINUX)

    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 9600
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return -3;
    }

    //Each message from arduino has a maximum of 5 bytes, 1 for the channel, 4 for the value
//    char channel;   //Channel for the arduino communication
//    char msg[4];    //Message to send/receive to/from arduino
//    int value = 0;  //Real value of the message received
    
//    char char_buf;
//    bool read_complete = false; //Flag to check if the read is complete
//    int n = 0;

    rclcpp::init(argc, argv);
    //rclcpp::Rate rate(50);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("Serial_com");

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr msg_string;
    rclcpp::SensorDataQoS qos;
    qos.keep_last(100);
    msg_string = node->create_subscription<std_msgs::msg::String>("/motor_speed", qos, motorCallback);

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_current;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_odo;
    pub_current = node->create_publisher<std_msgs::msg::String>("/motor_current", 1);
    pub_odo = node->create_publisher<std_msgs::msg::String>("/motor_odometry", 1);

    int read_state = 0; //aux to read the bytes
    int n_read = 0; //number of bytes read from buffer
    char b_read; //byte received

    char channel; //Byte read from buffer
    char msg[4]; //Channel value received
    int value = 0; //Number of the value received

    bool current_first = false; //The published msg is left motor info first, and then right. bool is to order the msg correctly
    bool current_ready = false; //To check if the msg is ready to be published
    auto current_msg = std_msgs::msg::String(); //String to publish

    bool odo_first = false; //The published msg is left motor info first, and then right. bool is to order the msg correctly
    bool odo_ready = false; //To check if the msg is ready to be published
    auto odo_msg = std_msgs::msg::String(); //String to publish

    while(rclcpp::ok()){

        rclcpp::spin_some(node);
        //rate.sleep();

        n_read = read(serial_port, &b_read, 1);

        //There is a byte to check
        if (n_read == 1){
/*
            //During a msg reading, an invalid byte was read. Discards the hole msg
            if ( (read_state > 0) && !((b_read >= '0') && (b_read <= 9)) && !((b_read >= 'A') && (b_read <= 'F')) ){
                read_state = 0;
            }
*/
            //First letter (channel)
            if ( (b_read >= 'g') && (b_read <= 'z') ){
                channel = b_read;
                read_state = 1;
            }
            //Get the values from channel
            else if ( (read_state >= 1) && (read_state < 5)){
                msg[read_state-1] = b_read;
                read_state++;
            }
            //Message is complete
            if ( (read_state == 5) ){

                //From Hexa to dec
                value = getMessageValue(msg);

                //Verify which channel was received, to prepare to publish the data
                //Current of left motor
                if ( channel == 'i' ){
                    current_msg.data = channel + std::to_string(value);
                    current_first = true;
                }
                //Current of the right motor
                else if ( (channel == 'j') && (current_first) ){
                    current_msg.data = current_msg.data + " " + channel + std::to_string(value);
                    current_first = false;
                    current_ready = true;
                }

                //Odometry of the left motor
                if ( channel == 'l' ){
                    odo_msg.data = channel + std::to_string(value);
                    odo_first = true;
                }
                //Odometry of the right motor
                else if ( (channel == 'r') && (odo_first) ){
                    odo_msg.data = odo_msg.data + " " + channel + std::to_string(value);
                    odo_first = false;
                    odo_ready = true;
                }
                read_state = 0;
            }
        }

        //Current message is ready to be published
        if ( current_ready ){
            pub_current->publish(current_msg);
            current_ready = false;
        }

        //Odometry message is ready to be published
        if ( odo_ready ){
            pub_odo->publish(odo_msg);
            odo_ready = false;
        }
    }

    rclcpp::shutdown();
    close(serial_port);

    return 0;
}
