#include <stdio.h>
#include <stdlib.h>
#include <stdio.h>
#include <cstring>
#include <unistd.h>

#include <signal.h>

#include "hangfa_uart_platform/hangfa_uart_platform_driver_node.h"
#include "hangfa_uart_platform/type.h"

//ros
#include <ros/ros.h>


using namespace hangfa_platform;


void onRosShutdown(int sig)
{
	//close uart first
	uart_uninit(&HangfaPlatfromUartDriver::m_uartHd);

	//close ros node
	ros::shutdown();
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"hangfa_platform_node",ros::init_options::NoSigintHandler);
	ros::NodeHandle nh;

	//設置串口的波特率
	std::string hangfa_uart_port = std::string("");
	int hangfa_uart_baudrate = 0;
	ros::param::get("~hangfa_uart_port",hangfa_uart_port);
	ros::param::get("~hangfa_uart_baudrate",hangfa_uart_baudrate);
	ROS_INFO("hangfa_uart_port: %s, hangfa_uart_baudrate: %d",hangfa_uart_port.c_str(),hangfa_uart_baudrate);

	//设置运动控制方式
	int hangfa_controller_type = 0;
	int hangfa_max_speed = 0;
	ros::param::get("~controller_type",hangfa_controller_type);
	ros::param::get("~max_speed",hangfa_max_speed);


	HangfaPlatfromUartDriver hangfaUartDriver(nh,hangfa_controller_type,hangfa_max_speed);
	int ret = 0;
	ret = uart_init(&HangfaPlatfromUartDriver::m_uartHd, hangfa_uart_port.c_str(),hangfa_uart_baudrate,&HangfaPlatfromUartDriver::uartRec, NULL);
	if (0 != ret)
	{
		printf("uart_init error ret = %d\n", ret);
	}

	signal(SIGINT,onRosShutdown);
	ros::spin();
	return 0;

}
