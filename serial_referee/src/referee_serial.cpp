//
// Created by ckyf on 23-3-27.
//
#include <string>
#include <ros/ros.h>
#include <iostream>
#include "../include/uart.h"
#include "std_msgs/Float32.h"

ros::Publisher channel1_publisher;
ros::Publisher channel2_publisher;
ros::Publisher channel3_publisher;
ros::Publisher channel4_publisher;
ros::Publisher channel5_publisher;
ros::Publisher channel6_publisher;

int uart_com = 0;
int float_num = 9;
unsigned short buff_1[4]={0,0,0,0};
unsigned short buff_3[1000];
unsigned short buff_write_flag = 0;
unsigned char buff_2[80];

int rc_mode = 0;
int previous_rc_mode = 0;


void uart_timer_callback(const ros::TimerEvent& event){
    uint8_t frame_hex[1] = {0x00};

    if(read(uart_com, &frame_hex, 1)){
//        std::cout<<"HEX:"<<(int)frame_hex[0]<<std::endl;
//        uart_buff.push_back(frame_hex[0]);
//        scan();
        buff_3[buff_write_flag] = frame_hex[0];
		//std::cout<<std::hex<<buff_3[buff_write_flag]<<std::endl;
		if((buff_3[buff_write_flag]== 0x7F) && (buff_write_flag>((float_num +1)*4 -2))){
			if(buff_3[buff_write_flag-1]== 0x80){
				if(buff_3[buff_write_flag-2]== 0x00){
					if(buff_3[buff_write_flag-3]== 0x00){
						if(buff_write_flag!=((float_num +1)*4 -1)){
							//std::cout<<"error, "<<buff_write_flag<<std::endl;
							buff_write_flag=0;
							return;
						}
						for(int i = 0;i<=(float_num*4 -1);i++){
							buff_2[i]=buff_3[buff_write_flag-((float_num +1)*4 -1)+i];
						}
                        float channel[9];
						channel[0] = *((float*)(&buff_2));
						for(int i = 1;i<float_num;i++){
							float temp = *((float*)(&buff_2[4*i]));
							channel[i] = temp;
						}
						buff_write_flag=0;

						if(channel[7] > 200.0){
                        	rc_mode = 3;
						}else{
                        	if(channel[7] > -200.0){
                                  rc_mode = 2;
                        	}else{
                                  rc_mode = 1;
                        	}
						}
                        if(channel[8] < 200.0){
                          rc_mode = 0;
                        }
						std_msgs::Float32 channel_msg;
                        channel_msg.data = channel[1];
                        channel1_publisher.publish(channel_msg);
                        channel_msg.data = channel[2];
                        channel2_publisher.publish(channel_msg);
                        channel_msg.data = channel[3];
                        channel3_publisher.publish(channel_msg);
                        channel_msg.data = channel[4];
                        channel4_publisher.publish(channel_msg);
                        channel_msg.data = channel[5];
                        channel5_publisher.publish(channel_msg);
                        channel_msg.data = channel[6];
                        channel6_publisher.publish(channel_msg);

						if(rc_mode != previous_rc_mode){
							ros::NodeHandle nh;
							nh.setParam("/custom_debug/rc_mode",int(rc_mode));
                            std::cout<<rc_mode<<std::endl;
							ROS_INFO("Flight mode changed!");
						}
                        previous_rc_mode = rc_mode;
						return;
					}
				}
			}
		}
		buff_write_flag++;
		if(buff_write_flag>990){
			buff_write_flag = 0;
		}
    }else{
        //ROS_INFO("Nothing Received!");
    }
}

int main(int argc, char** argv) {
    std::string serial_name;
    ros::init(argc, argv, "serial_referee");
	ros::NodeHandle nh("~");
    nh.param<std::string>("serial_referee_name", serial_name, "/dev/ttyACM0");

    uart_com = open(serial_name.data(), O_RDWR);
    if(uart_com == -1){
        ROS_INFO("UART OPEN FAIL!");
        return 0;
    }
    struct termios Opt;
    tcgetattr(uart_com, &Opt);
    cfsetispeed(&Opt,B115200);
    cfsetospeed(&Opt,B115200);
    Opt.c_lflag  &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input*/
    Opt.c_oflag  &= ~OPOST;   /*Output*/
    tcsetattr(uart_com,TCSANOW,&Opt);

    ros::Timer timer = nh.createTimer(ros::Duration(0.0001), uart_timer_callback);
	channel1_publisher = nh.advertise<std_msgs::Float32>("/custom_debug/rc1", 100);
    channel2_publisher = nh.advertise<std_msgs::Float32>("/custom_debug/rc2", 100);
    channel3_publisher = nh.advertise<std_msgs::Float32>("/custom_debug/rc3", 100);
    channel4_publisher = nh.advertise<std_msgs::Float32>("/custom_debug/rc4", 100);
	channel5_publisher = nh.advertise<std_msgs::Float32>("/custom_debug/rc5", 100);
    channel6_publisher = nh.advertise<std_msgs::Float32>("/custom_debug/rc6", 100);
    ROS_INFO("RC Loop Start!");
    while(ros::ok()){
        ros::spin();
    }
    return 0;
}