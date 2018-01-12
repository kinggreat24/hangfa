#include "hangfa_uart_platform/hangfa_uart_platform_driver_node.h"
#include "hangfa_uart_platform/tool.h"
#include "hangfa_uart_platform/type.h"

#include <boost/shared_ptr.hpp>
#include <boost/smart_ptr.hpp>
#include <iostream>

using namespace hangfa_platform;

UART_HANDLE HangfaPlatfromUartDriver::m_uartHd;

boost::mutex HangfaPlatfromUartDriver::m_ttsStatusMutex;

std::vector<char> HangfaPlatfromUartDriver::m_bigbuf;

int HangfaPlatfromUartDriver::m_recvIndex = 0;

int HangfaPlatfromUartDriver::m_packageLength = 0;

ros::Publisher HangfaPlatfromUartDriver::m_pubPlatformConfigRes;
ros::Publisher HangfaPlatfromUartDriver::m_pubErrorMessage;
ros::Publisher HangfaPlatfromUartDriver::m_pubSonar[16];
ros::Publisher HangfaPlatfromUartDriver::m_pubHangfaOdom;
ros::Publisher HangfaPlatfromUartDriver::m_pubAutoCharging;
ros::Publisher HangfaPlatfromUartDriver::m_pubPowerPcbStatus;
ros::Publisher HangfaPlatfromUartDriver::m_pubPowerInfo;
ros::Publisher HangfaPlatfromUartDriver::m_pubDeviceVersion;
ros::Publisher HangfaPlatfromUartDriver::m_pubDeviceSerialNumber;
ros::Publisher HangfaPlatfromUartDriver::m_pubSaveConfig;
ros::Publisher HangfaPlatfromUartDriver::m_pubMotorStatus;
ros::Publisher HangfaPlatfromUartDriver::m_pubUltrasonicConfig;
ros::Publisher HangfaPlatfromUartDriver::m_pubPlatformConfig;
ros::Publisher HangfaPlatfromUartDriver::m_pubEncoderConfig;
ros::Publisher HangfaPlatfromUartDriver::m_pubPowerConfig;


#define STR(s) #s  
#define CONN(a,b) a##.##b
#define CONNS(a,b,c,d) a##.##b##.##c##.##d

HangfaPlatfromUartDriver::HangfaPlatfromUartDriver(const ros::NodeHandle &nh,const int controller_type,const int max_speed)
	:m_nh(nh)
	,m_controllerType(controller_type)
	,m_maxSpeed(max_speed)
{
	//Subscriber
	m_subTwist     = m_nh.subscribe<geometry_msgs::Twist>("cmd_vel",1,&HangfaPlatfromUartDriver::onTwistCb,this);
	m_subCommand   = m_nh.subscribe<hangfa_uart_platform::HangfaCommand>("hangfa_command",1,&HangfaPlatfromUartDriver::onHangfaCommandCb,this);
	m_subSingleParamSetting = m_nh.subscribe<hangfa_uart_platform::SingleParamSetting>("singleParamSetting",1,&HangfaPlatfromUartDriver::onSingleParamSettingCb,this);
	m_subAutoCharging = m_nh.subscribe<std_msgs::Bool>("auto_charge",1,&HangfaPlatfromUartDriver::onAutoChargingCb,this);
	

	//Publisher  
	m_pubPlatformConfigRes = m_nh.advertise<hangfa_uart_platform::HandfaPlatformConfig>("hangfa_config_response",1);
	m_pubErrorMessage = m_nh.advertise<hangfa_uart_platform::ErrorMessage>("error_message",1);
	m_pubDeviceVersion = m_nh.advertise<hangfa_uart_platform::DeviceVersion>("device_version",1);
	m_pubDeviceSerialNumber = m_nh.advertise<hangfa_uart_platform::DeviceSerialNumber>("device_sn",1);
	m_pubSaveConfig = m_nh.advertise<hangfa_uart_platform::SaveConfigResult>("save_config_result",1);


	//超声波模块
	m_pubSonar[0] = m_nh.advertise<sensor_msgs::Range>("sonar_1",1);	
	m_pubSonar[1] = m_nh.advertise<sensor_msgs::Range>("sonar_2",1);
	m_pubSonar[2] = m_nh.advertise<sensor_msgs::Range>("sonar_3",1);
	m_pubSonar[3] = m_nh.advertise<sensor_msgs::Range>("sonar_4",1);
	m_pubSonar[4] = m_nh.advertise<sensor_msgs::Range>("sonar_5",1);	
	m_pubSonar[5] = m_nh.advertise<sensor_msgs::Range>("sonar_6",1);
	m_pubSonar[6] = m_nh.advertise<sensor_msgs::Range>("sonar_7",1);
	m_pubSonar[7] = m_nh.advertise<sensor_msgs::Range>("sonar_8",1);
	m_pubSonar[8] = m_nh.advertise<sensor_msgs::Range>("sonar_9",1);	
	m_pubSonar[9] = m_nh.advertise<sensor_msgs::Range>("sonar_10",1);
	m_pubSonar[10] = m_nh.advertise<sensor_msgs::Range>("sonar_11",1);
	m_pubSonar[11] = m_nh.advertise<sensor_msgs::Range>("sonar_12",1);
	m_pubSonar[12] = m_nh.advertise<sensor_msgs::Range>("sonar_13",1);	
	m_pubSonar[13] = m_nh.advertise<sensor_msgs::Range>("sonar_14",1);
	m_pubSonar[14] = m_nh.advertise<sensor_msgs::Range>("sonar_15",1);
	m_pubSonar[15] = m_nh.advertise<sensor_msgs::Range>("sonar_16",1);
	m_pubUltrasonicConfig = m_nh.advertise<hangfa_uart_platform::UltrasonicConfig>("ultrasonic_config",1);	



	//全场定位装置模块
	m_pubHangfaOdom = m_nh.advertise<hangfa_uart_platform::HangfaOdom>("hangfaOdom",1);
	m_pubEncoderConfig=m_nh.advertise<hangfa_uart_platform::EncoderConfig>("encoder_config",1);

	//自动充电模块
	m_pubAutoCharging = m_nh.advertise<hangfa_uart_platform::AutomaticCharging>("autoCharging",1);
	

	//电源模块
        m_subPowerManage = m_nh.subscribe<hangfa_uart_platform::PowerManage>("power_manage",1,&HangfaPlatfromUartDriver::onPowerManageCb,this);
	m_pubPowerPcbStatus = m_nh.advertise<hangfa_uart_platform::PowerManageStatus>("power_pcb_status",1);
	m_pubPowerInfo = m_nh.advertise<hangfa_uart_platform::PowerInfo>("power_info",1);
	m_pubPowerConfig = m_nh.advertise<hangfa_uart_platform::PowerConfig>("power_config",1);


	//运动控制模块
	m_subSetSonarAlarm = m_nh.subscribe<hangfa_uart_platform::SetSonarAlarm>("setSonarAlarm",1,&HangfaPlatfromUartDriver::onSonarAlarmCb,this);
	m_pubMotorStatus = m_nh.advertise<hangfa_uart_platform::MotorDriverStatus>("motor_status",1);
	m_pubPlatformConfig = m_nh.advertise<hangfa_uart_platform::HandfaPlatformConfig>("platform_config",1);



	//TimerEvent
	m_cmdvel_timer = m_nh.createTimer(ros::Duration(0.5),&HangfaPlatfromUartDriver::onCmdvelTimerEvent,this);



	//Param init
	m_current_cmdvel = boost::shared_ptr<const geometry_msgs::Twist>(new geometry_msgs::Twist());

	m_bigbuf.clear();
	m_isAutoCharging = false;
}


HangfaPlatfromUartDriver::~HangfaPlatfromUartDriver()
{

}



void HangfaPlatfromUartDriver::onMessageProcess(const char* buf, const int len)
{
	if(!cksCheck(buf,len))
		return;

	//解析数据
	unsigned char command_type = buf[3];
	//ROS_INFO("command_type: %d",command_type);
	switch(command_type)
	{
		case hangfa_uart_platform::HangfaCommand::Device_type:                             //COMMAD_TYPE ----  20　获取设备类型
		{
			ROS_INFO("Device type: %d.",buf[5]);
			break;
		}
		case hangfa_uart_platform::HangfaCommand::Device_version:                          //COMMAD_TYPE ----  21　读取设备版本号
		{ 
			ROS_INFO("Recv device version.");
			uint8_t device_type = static_cast<uint8_t>(buf[2]);
			
			hangfa_uart_platform::DeviceVersion device_version;
			device_version.device_type = static_cast<uint8_t>(buf[1]);
			
			boost::shared_array<char> hardware_version_buffer(new char[10]);
			sprintf(hardware_version_buffer.get(),"%d.%d",static_cast<uint8_t>(buf[6]),static_cast<uint8_t>(buf[5]));
			device_version.hardware_version = std::string(hardware_version_buffer.get());
			
			boost::shared_array<char> software_version_buffer(new char[20]);
			sprintf(software_version_buffer.get(),"%d.%d.%d.%d",static_cast<uint8_t>(buf[7]),static_cast<uint8_t>(buf[8]),static_cast<uint8_t>(buf[9]),static_cast<uint8_t>(buf[10]));
			device_version.software_version = std::string(software_version_buffer.get());

			m_pubDeviceVersion.publish(device_version);
			break;
		}
		case hangfa_uart_platform::HangfaCommand::Device_serialNumber:                     //COMMAD_TYPE ----  22　读取设备序列号
		{
			ROS_INFO("Recv device serial number.");
			hangfa_uart_platform::DeviceSerialNumber serial_number;
		
			serial_number.device_index = buf[5] | ( (buf[6]>>2 &0x01)<<2 | ( (buf[6]>>1 &0x01)<<1 | buf[6]&0x01 ) ) << 8 ;
			serial_number.day = (buf[6]>>3 & 0x01) | (buf[6]>>4 & 0x01) << 1 | (buf[6]>>5 & 0x01) << 2 | (buf[6] >> 6 & 0x01) << 3 | (buf[6] >> 7 & 0x01) << 4;
			serial_number.month = buf[7]&0x01 | (buf[7]>>1 &0x01)<< 1 | (buf[7]>>2 &0x01) << 2 | (buf[7]>>3 &0x01) << 3;
			serial_number.year = (buf[7]>>4&0x01 | (buf[7]>>5&0x01)<< 1 | (buf[7]>>6 &0x01) << 2 | (buf[7]>>7 &0x01) << 3) | buf[8] << 8;
			
			m_pubDeviceSerialNumber.publish(serial_number);
			break;
		}
		case hangfa_uart_platform::HangfaCommand::Save_config:                             //COMMAD_TYPE ----  23　保存设置
		{
			ROS_INFO("Recv save parameter config.");
			hangfa_uart_platform::SaveConfigResult saveConfigResult;
			saveConfigResult.device_type = static_cast<uint8_t>(buf[1]);
			saveConfigResult.save_result = static_cast<uint8_t>(buf[5]);
			m_pubSaveConfig.publish(saveConfigResult);
			break;
		}
		case hangfa_uart_platform::HangfaCommand::Restore_factory_setting:                 //COMMAD_TYPE ----  24　恢复出厂设置
		{
			ROS_INFO("Recv restore factory setting.");
			break;
		}
		case hangfa_uart_platform::HangfaCommand::Device_restart:                          //COMMAD_TYPE ----  25　设备重启
		{
			ROS_INFO("Recv device restart.");
			break;
		}
		case hangfa_uart_platform::HangfaCommand::GetDeviceConfig:                         //COMMAD_TYPE ----  27　读取设备所有的参数
		{
			ROS_INFO("Recv get device parameter config.");
			unsigned char device_address = buf[2];
			switch(device_address)
			{
				case hangfa_uart_platform::HangfaCommand::DeviceType_RobotHost:
				{
					//主机
					break;
				}
				case hangfa_uart_platform::HangfaCommand::DeviceType_PowerManage:
				{
					//电源管理模块
					hangfa_uart_platform::PowerConfig power_config;
					power_config.uart_baud = static_cast<uint8_t>(buf[5]);
					power_config.can_baud = static_cast<uint8_t>(buf[6]);
					power_config.rs485_baud = static_cast<uint8_t>(buf[7]);
					power_config.device_address = static_cast<uint8_t>(buf[8]);
					power_config.device_type =static_cast<uint8_t>(buf[9]);

					power_config.voltage_max = bytes2int32(buf+10);
					power_config.voltage_min = bytes2int32(buf+14);
					power_config.voltage_level_1 = bytes2int32(buf+18);
					power_config.voltage_level_2= bytes2int32(buf+22);
					power_config.voltage_level_3= bytes2int32(buf+26);
					power_config.voltage_level_4= bytes2int32(buf+30);
					power_config.voltage_level_5= bytes2int32(buf+34);
					power_config.voltage_level_6= bytes2int32(buf+38);
					power_config.voltage_level_7= bytes2int32(buf+42);
					power_config.voltage_level_8= bytes2int32(buf+46);
					power_config.voltage_level_9= bytes2int32(buf+50);
					power_config.voltage_level_10= bytes2int32(buf+54);
					power_config.full_power_voltage= bytes2int32(buf+58);
					power_config.full_power_current= bytes2int32(buf+62);
					power_config.battery_current_correcting= bytes2int32(buf+66);
					power_config.charging_current_correcting= bytes2int32(buf+70);
					power_config.i2t_limi_current= bytes2int32(buf+74);
					power_config.peek_limi_current= bytes2int32(buf+78);
					power_config.battery_current_coefficient= bytes2int32(buf+82);
					power_config.charing_current_coefficient= bytes2int32(buf+86);
					
 					m_pubPowerConfig.publish(power_config);
					break;
				}
				case hangfa_uart_platform::HangfaCommand::DeviceType_MotionControl:
				{
					//电机驱动模块
					hangfa_uart_platform::HandfaPlatformConfig platform_config;
					platform_config.uart_baudrate = bytes2int32(buf+5);
					platform_config.can_baudrate  = bytes2int32(buf+9);
					platform_config.device_address = bytes2int32(buf+13);
					platform_config.min_interval  = bytes2int32(buf+17);
					platform_config.max_run_time = bytes2int32(buf+21);

					platform_config.origin_orientation  = bytes2float32(buf+25);
					platform_config.enable_fan   = bytes2int32(buf+29);
					platform_config.cpu_max_temperature = bytes2int32(buf+33);
					platform_config.fan_start_temperature = bytes2int32(buf+37);
					platform_config.platform_type = bytes2int32(buf+41);
					platform_config.motive_wheel_perimeter = bytes2float32(buf+45);
					platform_config.distance_motive_wheel_center = bytes2float32(buf+49);
					platform_config.distance_motive_wheel_x_center = bytes2float32(buf+53);
					platform_config.distance_motive_wheel_y_center = bytes2float32(buf+57);
					for(int i=0;i<4;i++)
					{
						platform_config.wheel_rotation_adjust_coefficient[i] = bytes2float32(buf+61 + i*4);
						platform_config.wheel_linear_adjust_coefficient[i] = bytes2float32(buf+77 + i*4);
						platform_config.wheel_linear_y_adjust_coefficient[i] = bytes2float32(buf+93 + i*4);
					}
					platform_config.motor_driver_type = bytes2int32(buf+109);
					platform_config.undefine_1 = bytes2int32(buf+113);
					platform_config.motor_deceleration_ratio = bytes2float32(buf+117);
					platform_config.encoder_lines_number = bytes2int32(buf+121);
					platform_config.motor_driver_max_speed = bytes2int32(buf+125);
					platform_config.undefine_2 = bytes2int32(buf+129);
					m_pubPlatformConfig.publish(platform_config);
					break;
				}
				case hangfa_uart_platform::HangfaCommand::DeviceType_UltrasonicInfrared:
				{
					//超声红外模块
					break;
				}
				case hangfa_uart_platform::HangfaCommand::DeviceType_Ultrasonic:
				{
					//超声模块	
					hangfa_uart_platform::UltrasonicConfig ultrasonicConfig;
					ultrasonicConfig.uart_baud = bytes2int32(buf+5);
					ultrasonicConfig.can_baud = bytes2int32(buf+9);
					ultrasonicConfig.device_address = bytes2int32(buf+13);
					ultrasonicConfig.send_time_interval = bytes2int32(buf+17);
					ultrasonicConfig.send_mode = bytes2int32(buf+21);
					for(int i=0;i<16;i++)
						ultrasonicConfig.send_config_group[i] = static_cast<unsigned int>(bytes2int32(buf+25 + i*4));

					ultrasonicConfig.inquire_config = bytes2int32(buf + 89);
					ultrasonicConfig.beep_enable = bytes2int32(buf + 93);
					ultrasonicConfig.alarm_distance = bytes2int32(buf + 97);
					ultrasonicConfig.uart_distance_send_enable = bytes2int32(buf + 101);
					ultrasonicConfig.uart_alarm_send_enable = bytes2int32(buf + 105);
					ultrasonicConfig.can_distance_send_enable =  bytes2int32(buf + 109);
					ultrasonicConfig.can_alarm_send_enable = bytes2int32(buf + 113);

					m_pubUltrasonicConfig.publish(ultrasonicConfig);
					break;
				}
				case hangfa_uart_platform::HangfaCommand::DeviceType_Acceleration:
				{
					//加速度
					break;
				}

				case hangfa_uart_platform::HangfaCommand::DeviceType_Gyroscope:
				{
					//陀螺仪
					break;
				}
				case hangfa_uart_platform::HangfaCommand::DeviceType_Compass:
				{
					//电子罗盘
					break;
				}
				case hangfa_uart_platform::HangfaCommand::DeviceType_GPS:
				{
					//GPS
					break;
				}
				case hangfa_uart_platform::HangfaCommand::DeviceType_Inclinometer:
				{
					//倾角
					break;
				}
				case hangfa_uart_platform::HangfaCommand::DeviceType_Lidar:
				{
					//激光雷达
					break;
				}
				case hangfa_uart_platform::HangfaCommand::DeviceType_EncoderPosition:
				{
					//码盘定位
					hangfa_uart_platform::EncoderConfig encoder_config;
					encoder_config.uart_baud = bytes2int32(buf+5);
					encoder_config.can_baud = bytes2int32(buf+9);
					encoder_config.device_address = bytes2int32(buf+13);
					encoder_config.encoder_line = bytes2int32(buf+17);

					encoder_config.encoder_wheel_perimeter = bytes2float32(buf+21);
					encoder_config.distance_encoder1_center = bytes2float32(buf+25);
					encoder_config.distance_encoder2_center = bytes2float32(buf+29);
					encoder_config.distance_encoder3_center = bytes2float32(buf+33);
					encoder_config.k_coefficient_distance_encoder1_center = bytes2float32(buf+37);
					encoder_config.k_coefficient_distance_encoder2_center = bytes2float32(buf+41);
					encoder_config.k_coefficient_distance_encoder3_center = bytes2float32(buf+45);
					encoder_config.b_coefficient_distance_encoder1_center = bytes2float32(buf+49);
					encoder_config.b_coefficient_distance_encoder2_center = bytes2float32(buf+53);
					encoder_config.b_coefficient_distance_encoder3_center = bytes2float32(buf+57);
					encoder_config.k_coefficient_wheel1_install = bytes2float32(buf+61);
					encoder_config.k_coefficient_wheel2_install = bytes2float32(buf+65);
					encoder_config.k_coefficient_wheel3_install = bytes2float32(buf+69);
					encoder_config.b_coefficient_wheel1_install = bytes2float32(buf+73);
					encoder_config.b_coefficient_wheel2_install = bytes2float32(buf+77);
					encoder_config.b_coefficient_wheel3_install = bytes2float32(buf+81);
					encoder_config.enable_locating = bytes2int32(buf+85);
					encoder_config.sample_time_interval= bytes2int32(buf+89);
					encoder_config.send_time_interval= bytes2int32(buf+93);
					encoder_config.enable_uart_send_position= bytes2int32(buf+97);
					encoder_config.enable_can_send_position= bytes2int32(buf+101);
					encoder_config.enable_uart_send_pulse= bytes2int32(buf+105);
					encoder_config.enable_can_send_pulse= bytes2int32(buf+109);
					encoder_config.distance_encoder_platform_center =  bytes2float32(buf+113);
					
					m_pubEncoderConfig.publish(encoder_config);
					break;
				}
				case hangfa_uart_platform::HangfaCommand::DeviceType_AreaPosition:
				{
					//局域定位
					break;
				}
				case hangfa_uart_platform::HangfaCommand::DeviceType_LaserInfrared:
				{
					//激光红外
					break;
				}
				case hangfa_uart_platform::HangfaCommand::DeviceType_FindMarkingLine:
				{
					//巡线模块
					break;
				}
				case hangfa_uart_platform::HangfaCommand::DeviceType_Broadcast:
				{
					//广播设备
					break;
				}
				default:
					break;


			}
			for(int i=0;i<len;i++)
				ROS_INFO("%x",buf[i]);
			//ROS_INFO("Recv get device parameter config.");
			break;
		}
		case hangfa_uart_platform::HangfaCommand::SetDeviceConfig:                            //COMMAD_TYPE ----  28　设置设备参数
		{
			ROS_INFO("Recv set device parameter config.");
			break;
		}
		case hangfa_uart_platform::HangfaCommand::GetSingleParam:                            //COMMAD_TYPE ----  29　获取单个参数
		{
			ROS_INFO("Recv get device single parameter.");
			break;
		}
		case hangfa_uart_platform::HangfaCommand::SetSingleParam:                             //COMMAD_TYPE ----  30　设置单个参数
		{
			ROS_INFO("Recv set device single parameter.");
			break;
		}
		case hangfa_uart_platform::HangfaCommand::SetSonarAlarmState:                         //COMMAD_TYPE ----  31
		{
			ROS_INFO("Recv set Sonar Alarm State.");
			break;
		}
		case hangfa_uart_platform::HangfaCommand::Respond:                                    //COMMAD_TYPE ----  255
		{
			hangfa_uart_platform::ErrorMessage errorMessage;
			errorMessage.erron = buf[5];
			switch(errorMessage.erron)
			{
				case hangfa_uart_platform::ErrorMessage::Correct:
				{
					errorMessage.error_info = "Correct.";
					break;
				}
				case hangfa_uart_platform::ErrorMessage::StartFlagError:
				{
					errorMessage.error_info = "Error info: Start flag error";
					break;
				}
				case hangfa_uart_platform::ErrorMessage::EndFlagError:
				{
					errorMessage.error_info = "Error info: End flag error";
					break;
				}
				case hangfa_uart_platform::ErrorMessage::PkgLengthSmall:
				{
					errorMessage.error_info = "Error info: Package length smaller than minimum value.";
					break;
				}
				case hangfa_uart_platform::ErrorMessage::DeviceAddressError:
				{
					errorMessage.error_info = "Error info: Device address error.";
					break;
				}
				case hangfa_uart_platform::ErrorMessage::DeviceIdError:
				{
					errorMessage.error_info = "Error info: Device id error.";
					break;
				}
				case hangfa_uart_platform::ErrorMessage::CrcCheckError:
				{
					errorMessage.error_info = "Error info: CRC check error.";
					break;
				}
				case hangfa_uart_platform::ErrorMessage::CmdExecuteError:
				{
					errorMessage.error_info = "Error info: Command execution error.";
					break;
				}
				default:
					ROS_ERROR("Unknown error message");
					break;
				
			}
			m_pubErrorMessage.publish(errorMessage);
			break;
		}
		case 40:
		{
			unsigned char device_address = buf[2];
			switch(device_address)
			{
				case hangfa_uart_platform::HangfaCommand::DeviceType_PowerManage: //电源模块
				{
					ROS_INFO("Power status.");
					hangfa_uart_platform::PowerManageStatus powerManageStatus;
					uint8_t powerPcbStatus = static_cast<uint8_t>(buf[5]);

					powerManageStatus.header.stamp = ros::Time::now();
					powerManageStatus.header.frame_id = "power manage module";
					//判断每个bit的数值	
					powerManageStatus.load_switch = powerPcbStatus & 0x01;
					powerManageStatus.manual_charging_connecting = powerPcbStatus >> 1 & 0x01;
					powerManageStatus.auto_charging_connecting = powerPcbStatus >> 1 & 0x01;
					powerManageStatus.battery_undervoltage = powerPcbStatus >> 1 & 0x01;
					powerManageStatus.battery_overvoltage = powerPcbStatus >> 1 & 0x01;
					powerManageStatus.battery_overflow = powerPcbStatus >> 1 & 0x01;
					powerManageStatus.charging_state = powerPcbStatus >> 1 & 0x01;
					powerManageStatus.undefine = powerPcbStatus >> 1 & 0x01;
					
					m_pubPowerPcbStatus.publish(powerManageStatus);
				}
				default:
					break;
			}
			break;
		}
		case hangfa_uart_platform::HangfaCommand::ReadSampleValueOnce:                             //COMMAD_TYPE ----  41
		{
			unsigned char device_address = buf[2];
			switch(device_address)
			{
				case hangfa_uart_platform::HangfaCommand::DeviceType_Ultrasonic:  //超声模块
				{
					sensor_msgs::Range sonar_range_left,sonar_range_right;
					sonar_range_left.header.stamp = ros::Time::now();
					sonar_range_left.header.frame_id = "sonar_frame";
					sonar_range_left.min_range = 0.3;
					sonar_range_left.max_range = 4.5;
					sonar_range_left.range = bytes2int16(buf+7)*1.0/1000.0;
					sonar_range_right.header.stamp = ros::Time::now();
					sonar_range_right.header.frame_id = "sonar_frame";
					sonar_range_right.min_range = 0.3;
					sonar_range_right.max_range = 4.5;
					sonar_range_right.range = bytes2int16(buf+9)*1.0/1000.0;
					
					switch(static_cast<uint8_t>(buf[5]))
					{
						case 1:
						{
							m_pubSonar[0].publish(sonar_range_left);
							m_pubSonar[0+8].publish(sonar_range_right);
							break;
						}
						case 2:
						{
							m_pubSonar[1].publish(sonar_range_left);
							m_pubSonar[1+8].publish(sonar_range_right);
							break;
						}
						case 4:
						{
							m_pubSonar[2].publish(sonar_range_left);
							m_pubSonar[2+8].publish(sonar_range_right);
							break;
						}
						case 8:
						{
							m_pubSonar[3].publish(sonar_range_left);
							m_pubSonar[3+8].publish(sonar_range_right);
							break;
						}
						case 16:
						{
							m_pubSonar[4].publish(sonar_range_left);
							m_pubSonar[4+8].publish(sonar_range_right);
							break;
						}
						case 32:
						{
							m_pubSonar[5].publish(sonar_range_left);
							m_pubSonar[5+8].publish(sonar_range_right);
							break;
						}
						case 64:
						{
							m_pubSonar[6].publish(sonar_range_left);
							m_pubSonar[6+8].publish(sonar_range_right);
							break;
						}
						case 128:
						{
							m_pubSonar[7].publish(sonar_range_left);
							m_pubSonar[7+8].publish(sonar_range_right);
							break;
						}
						default:
							break;
					}  
					break;
				}
				case hangfa_uart_platform::HangfaCommand::DeviceType_PowerManage: //电源模块
				{
					ROS_INFO("Power management.");
					break;
				}
				case hangfa_uart_platform::HangfaCommand::DeviceType_EncoderPosition:        //编码器
				{
					ROS_INFO("EncoderPosition locating result clean"); //定位结果清零
					break;
				}
				case hangfa_uart_platform::HangfaCommand::DeviceType_LaserInfrared:        //激光红外
				{
					ROS_INFO("laserInfrared: %f",bytes2float32(buf+6));
					hangfa_uart_platform::AutomaticCharging autoCharging;
					autoCharging.status = static_cast<uint8_t>(buf[5]);
					autoCharging.laser_intensity  = bytes2float32(buf+6);
					m_pubAutoCharging.publish(autoCharging);
					break;
				}
				default:
					break;
			}
			break;
		}
		case 42:
		{
			unsigned char device_address = buf[2];   
			switch(device_address)
			{
				case hangfa_uart_platform::HangfaCommand::DeviceType_PowerManage: //电源模块 0x10
				{
					hangfa_uart_platform::PowerInfo power_info;
					power_info.voltage = bytes2int32(buf+5);
					power_info.current = bytes2int32(buf+9);
					power_info.charging_current = bytes2int32(buf+11);

					m_pubPowerInfo.publish(power_info);
					break;
				}
				case 0x20: //运动控制板
				{
					ROS_INFO("Read motion control board.");
					for(int i=0;i<len;i++)
						ROS_INFO("%x",buf[i]);

					hangfa_uart_platform::MotorDriverStatus motorDriverStatus;
					motorDriverStatus.action_state = static_cast<uint8_t>(buf[5]);
					motorDriverStatus.system_error = static_cast<uint8_t>(buf[6]);
					motorDriverStatus.motor_error = static_cast<uint8_t>(buf[7]);
					motorDriverStatus.actionbuffer_rest_action = static_cast<uint8_t>(buf[8]);
					motorDriverStatus.actionbuffer_current_address = static_cast<uint8_t>(buf[9]);
					motorDriverStatus.commandbuffer_rest_command = static_cast<uint8_t>(buf[10]);
					motorDriverStatus.commandbuffer_current_address = static_cast<uint8_t>(buf[11]);
					
					m_pubMotorStatus.publish(motorDriverStatus);
					break;
				}
				case hangfa_uart_platform::HangfaCommand::DeviceType_Ultrasonic: //超声波 0x32
				{
					/*ROS_INFO("read alarm state");
					for(int i=0;i<len;i++)
						ROS_INFO("%x",buf[i]);*/
					break;
				}
				case hangfa_uart_platform::HangfaCommand::DeviceType_EncoderPosition:        //编码器
				{
					ROS_INFO("Begin locating computating.");                    //开始定位计算
					break;
				}
				default:
					break;
			}
			break;
		}
		case 43:
		{
			unsigned char device_address = buf[2];   
			switch(device_address)
			{
				case 0x20: //运动控制板
				{
					ROS_INFO("Recovery all motor driver.");
					break;
				}
				case hangfa_uart_platform::HangfaCommand::DeviceType_Ultrasonic://超声波模块
				{
					ROS_INFO("Ultrasonic cpu mean temputure: %d degree.",static_cast<uint8_t>(buf[5]));
					break;
				}
				case hangfa_uart_platform::HangfaCommand::DeviceType_EncoderPosition:        //编码器
				{
					ROS_INFO("Stop locating computating.");                    
					break;
				}
				default:
					break;
			}
			break;
		}
		case 44:
		{	
			unsigned char device_address = buf[2];   			
			switch(device_address)
			{	
				case 0x20:
				{
					ROS_INFO("Clean up buffer success.");
					break;
				}
				case hangfa_uart_platform::HangfaCommand::DeviceType_EncoderPosition:        //编码器
				{
					ROS_INFO("Read all encoder number.");                     //读取所有编码器脉冲数
					break;
				}
				default:
					break;
			}
			break;
		}
		case 45:
		{
			unsigned char device_address = buf[2];
			switch(device_address)
			{
				case 0x20:
				{
					ROS_INFO("Clean up error flag success.");
					break;
				}
				case hangfa_uart_platform::HangfaCommand::DeviceType_Ultrasonic:   //停止测量
				{
					if(0 == static_cast<uint8_t>(buf[5]))
					{
						ROS_INFO("Stop ultrasonic mesasure success.");
					}
					else
						ROS_ERROR("Stop ultrasonic mesasure failed.");
					break;
				}
				case hangfa_uart_platform::HangfaCommand::DeviceType_EncoderPosition:        //编码器
				{
					ROS_INFO("Read odom locating information.");                             //读取定位数据
					for(int i=0;i<len;i++)
						ROS_INFO("%x",buf[i]);
					hangfa_uart_platform::HangfaOdom hangfaOdom;
					hangfaOdom.header.stamp = ros::Time::now();
					hangfaOdom.x_position = bytes2float32(buf+5)/1000.0;
					hangfaOdom.y_position = bytes2float32(buf+9)/1000.0;
					hangfaOdom.theta = bytes2float32(buf+13);
					m_pubHangfaOdom.publish(hangfaOdom);
					break;
				}
				default:
					break;
			}
			break;
		}
		case 46:
		{
			unsigned char device_address = buf[2];
			switch(device_address)
			{
				case hangfa_uart_platform::HangfaCommand::DeviceType_Ultrasonic:   //超声波
				{
					if(1 == static_cast<uint8_t>(buf[5])) //清除设备内部报警状态值 
					{
						ROS_INFO("Clear ultrasonic alarm success.");
					}
					else
						ROS_ERROR("Clear ultrasonic alarm failed.");
					break;
				}
				case hangfa_uart_platform::HangfaCommand::DeviceType_EncoderPosition:        //编码器
				{
					//设置基准位姿
					if(0 == static_cast<uint8_t>(buf[5]))
						ROS_INFO("Pose setting success.");
					else
						ROS_ERROR("Pose setting failed.");
					break;
				}
				default:
					break;
			}
			break;
		}
		case hangfa_uart_platform::HangfaCommand::SpeedWithRotationLinearMotion:              //COMMAD_TYPE ----  55
		{
			ROS_INFO("Recv SpeedWithRotationLinearMotion");
			break;
		}
		case hangfa_uart_platform::HangfaCommand::JoystickControlMotion:                     //COMMAD_TYPE ----  66
		{
			ROS_INFO("Recv joystic motion.");
			break;
		}
		default:
		{
			ROS_INFO("Recv other command!");
			break;
		}
	}
}



bool HangfaPlatfromUartDriver::cksCheck(const char* buffer,const int length)
{
	uint16_t crc = bytes2int16(buffer+length-3);
	if(crc == getCks((const uint8_t*)buffer,length-3))
	{	
		return true;
	}
	else
	{	
		/*for(int i=0;i<length;i++)
			ROS_INFO("%x",buffer[i]);
		ROS_WARN("Wrong crc check.");*/
		return false;
	}
}



void HangfaPlatfromUartDriver::uartRec(const void *msg, unsigned int msglen, void *user_data)
{
	if(0 == m_bigbuf.size())
	{
		for(int i=0;i<msglen;i++)
		{
			if(((unsigned char*)msg)[i] == SYNC_FLAG_START)
			{
				m_bigbuf.push_back(((unsigned char*)msg)[i]);
				m_recvIndex += 1;
			}
			else
			{
				if(m_recvIndex > 0)
				{
					//已经接收到数据头，开始将数据添加到容器中
					m_bigbuf.push_back(((unsigned char*)msg)[i]);
					m_recvIndex += 1;

					if(5 == m_recvIndex)
					{
						m_packageLength = ((unsigned char*)msg)[i];
					}
					
					if(m_packageLength+8 == m_recvIndex)
					{
						//读到一帧完整的数据
						onMessageProcess(m_bigbuf.data(),m_bigbuf.size());

						//标识符还原
						m_recvIndex = 0;
						m_packageLength = 0;
						m_bigbuf.clear();
					}
				}
				else
				{
					//没有接收到数据头
					ROS_INFO("Drop this byte %x.",((unsigned char*)msg)[i]);
				}
			}
		}
	}
	else
	{
		for(int i=0;i<msglen;i++)
		{
			m_bigbuf.push_back(((unsigned char*)msg)[i]);
			m_recvIndex += 1;

			if(5 == m_recvIndex)
			{
				m_packageLength = ((unsigned char*)msg)[i];
			}
			
			if(m_packageLength+8 == m_recvIndex)
			{
				//读到一帧完整的数据
				onMessageProcess(m_bigbuf.data(),m_bigbuf.size());

				//标识符还原
				m_recvIndex = 0;
				m_packageLength = 0;
				m_bigbuf.clear();
			}
		}
	}
}





//更新当前的速度
void HangfaPlatfromUartDriver::onTwistCb(const geometry_msgs::Twist::ConstPtr &msg)
{
	//ROS_INFO("update cmdvel");
	m_current_cmdvel = boost::shared_ptr<const geometry_msgs::Twist>(msg);
}



//将当前的速度以一定的频率发送到运动控制器
void HangfaPlatfromUartDriver::onCmdvelTimerEvent(const ros::TimerEvent &e)
{
	if(m_isAutoCharging)
		return;

	switch(m_controllerType)
	{
		case 0:                                 //实时速度控制(0x37)
		{
			//ROS_INFO("Real time control mode.");
			boost::shared_array<char> cmdvelBuffer(new char[14]);

			cmdvelBuffer[0] = SYNC_FLAG_START;
			cmdvelBuffer[1] = MOTION_DEVICE_TYPE;
			cmdvelBuffer[2] = MOTION_DEVICE_ADDRESS;

			cmdvelBuffer[3] = 0x37; //实时速度控制	
			cmdvelBuffer[4] = 0x06; //长度
	
			//运动方向
			char* orientation = hangfa_platform::int162bytes(90);
			memcpy(cmdvelBuffer.get()+5,orientation,2);

			//线速度
			char* linearSpeed = hangfa_platform::int162bytes(static_cast<int>(m_current_cmdvel->linear.x*10000));
			memcpy(cmdvelBuffer.get()+7,linearSpeed,2);

			//角速度
			char* angularSpeed = hangfa_platform::int162bytes(static_cast<int>(m_current_cmdvel->angular.z*100));
			memcpy(cmdvelBuffer.get()+9,angularSpeed,2);

			//CRC检校
			uint16_t crcValue = getCks((uint8_t*)(cmdvelBuffer.get()),11);
			char* crc = hangfa_platform::int162bytes(crcValue);
			memcpy(cmdvelBuffer.get()+11,crc,2);

			cmdvelBuffer[13] = SYNC_FLAG_END;

			boost::mutex::scoped_lock lock(m_ttsStatusMutex);
			uart_send(m_uartHd,cmdvelBuffer.get(),14);
#ifdef HANGFA_UART_DEBUG	
			for(int i=0;i<14;i++)
				ROS_INFO("%2X",cmdvelBuffer[i]);
#endif//HANGFA_UART_DEBUG
			break;
		}
		case 1:                               //手柄控制运动模式(0x42)                   
		{
			//ROS_INFO("Joystick control mode.");
			boost::shared_array<char> cmdvelBuffer(new char[14]);

			cmdvelBuffer[0] = SYNC_FLAG_START;
			cmdvelBuffer[1] = MOTION_DEVICE_TYPE;
			cmdvelBuffer[2] = MOTION_DEVICE_ADDRESS;

			cmdvelBuffer[3] = 0x42; //实时速度控制	
			cmdvelBuffer[4] = 0x06; //长度
	
			//左右X速度(将转向速度从-1.0~1.0映射到-500~500)
			int x_speed = static_cast<int>(m_current_cmdvel->angular.z*500);
			if(x_speed > 500)
				x_speed = 500;
			else if(x_speed < -500)
				x_speed = -500;
			char* xSpeed = hangfa_platform::int162bytes(-x_speed);
			memcpy(cmdvelBuffer.get()+5,xSpeed,2);

			//前后Y速度(将前进速度从-1.0~1.0映射到-500~500)
			int y_speed = static_cast<int>(m_current_cmdvel->linear.x*500);
			if(y_speed > 500)
				y_speed = 500;
			else if(y_speed < -500)
				y_speed = -500;
			char* ySpeed = hangfa_platform::int162bytes(y_speed);
			memcpy(cmdvelBuffer.get()+7,ySpeed,2);

			//转向Z速度(设置为0)
			char* zSpeed = hangfa_platform::int162bytes(0);
			memcpy(cmdvelBuffer.get()+9,zSpeed,2);

			//CRC检校
			uint16_t crcValue = getCks((uint8_t*)(cmdvelBuffer.get()),11);
			char* crc = hangfa_platform::int162bytes(crcValue);
			memcpy(cmdvelBuffer.get()+11,crc,2);

			cmdvelBuffer[13] = SYNC_FLAG_END;

			boost::mutex::scoped_lock lock(m_ttsStatusMutex);
			uart_send(m_uartHd,cmdvelBuffer.get(),14);
#ifdef HANGFA_UART_DEBUG	
			//for(int i=0;i<14;i++)
			//	ROS_INFO("%x",cmdvelBuffer[i]);
#endif//HANGFA_UART_DEBUG
			break;
		}
		case 2:
		{
			//ROS_INFO("Simple joystick control mode.");
			boost::shared_array<char> cmdvelBuffer(new char[14]);

			cmdvelBuffer[0] = SYNC_FLAG_START;
			cmdvelBuffer[1] = 0x40/*MOTION_DEVICE_TYPE*/;
			cmdvelBuffer[2] = MOTION_DEVICE_ADDRESS;

			cmdvelBuffer[3] = 0x2A; //设置平台速度
			cmdvelBuffer[4] = 0x06; //长度
	
			//左右X速度
			int x_speed = static_cast<int>(m_current_cmdvel->angular.z*10000);
			char* xSpeed = hangfa_platform::int162bytes(x_speed );
			memcpy(cmdvelBuffer.get()+5,xSpeed,2);

			//前后Y速度
			int y_speed = static_cast<int>(m_current_cmdvel->linear.x*10000);
			char* ySpeed = hangfa_platform::int162bytes(y_speed);
			memcpy(cmdvelBuffer.get()+7,ySpeed,2);

			//转向Z速度(设置为0)
			char* zSpeed = hangfa_platform::int162bytes(0);
			memcpy(cmdvelBuffer.get()+9,zSpeed,2);

			//CRC检校
			uint16_t crcValue = getCks((uint8_t*)(cmdvelBuffer.get()),11);
			char* crc = hangfa_platform::int162bytes(crcValue);
			memcpy(cmdvelBuffer.get()+11,crc,2);

			cmdvelBuffer[13] = SYNC_FLAG_END;

			boost::mutex::scoped_lock lock(m_ttsStatusMutex);
			uart_send(m_uartHd,cmdvelBuffer.get(),14);
			break;
		}
		case 3:
		{
			//PathFinder控制运动模式
			boost::shared_array<char> cmdvelBuffer(new char[31]);

			cmdvelBuffer[0] = SYNC_FLAG_START;
			cmdvelBuffer[1] = 0x0F/*MOTION_DEVICE_TYPE*/;
			cmdvelBuffer[2] = 0x01;

			cmdvelBuffer[3] = 0x2F; //主动发送模式
			cmdvelBuffer[4] = 0x17; //长度
	
			/****        大摇杆三轴(6-bytes)     *****/
			//左右x速度
			int x_speed = static_cast<int>(m_current_cmdvel->angular.z*500);
			char* xSpeed = hangfa_platform::int162bytes(x_speed);
			memcpy(cmdvelBuffer.get()+5,xSpeed,2);

			//前后Y速度
			int y_speed = static_cast<int>(m_current_cmdvel->linear.x*500);
			char* ySpeed = hangfa_platform::int162bytes(y_speed);
			memcpy(cmdvelBuffer.get()+7,ySpeed,2);

			//转向Z速度(设置为0)
			char* zSpeed = hangfa_platform::int162bytes(0);
			memcpy(cmdvelBuffer.get()+9,zSpeed,2);

			/****        １号（左）小摇杆(4-bytes)     *****/
			//X-axis
			memcpy(cmdvelBuffer.get()+11,hangfa_platform::int162bytes(0),2);

			//y-axis
			memcpy(cmdvelBuffer.get()+13,hangfa_platform::int162bytes(0),2);


			/****        2号（右）小摇杆(4-bytes)     *****/
			//X-axis
			memcpy(cmdvelBuffer.get()+15,hangfa_platform::int162bytes(0),2);

			//y-axis
			memcpy(cmdvelBuffer.get()+17,hangfa_platform::int162bytes(0),2);

			/*电位器Ａ(2-bytes)(0~999)*/
			memcpy(cmdvelBuffer.get()+19,hangfa_platform::int162bytes(0),2);

			/*电位器Ｂ(2-bytes)(0~999)*/
			memcpy(cmdvelBuffer.get()+21,hangfa_platform::int162bytes(0),2);


			/*旋转开关A B(1-Byte)　　
			 *旋转开关功能有以下几种:BANA=1,选择遥控器控制模式,BANA=2 选择自动充电桩模式,BANA=3 选择巡线功能*/
			cmdvelBuffer[23] = 0x11;


			/*编码器Ａ(1-byte)*/
			cmdvelBuffer[24] = 0;

			/*编码器Ｂ(1-byte)*/
			cmdvelBuffer[25] = 0;

			/*钮子开关(1-byte)*/
			cmdvelBuffer[26] = 16 +  m_maxSpeed;

			/*按键开关(1-byte)*/
			cmdvelBuffer[27] = 0;

			//CRC检校
			uint16_t crcValue = getCks((uint8_t*)(cmdvelBuffer.get()),28);
			char* crc = hangfa_platform::int162bytes(crcValue);
			memcpy(cmdvelBuffer.get()+28,crc,2);

			cmdvelBuffer[30] = SYNC_FLAG_END;

			boost::mutex::scoped_lock lock(m_ttsStatusMutex);
			uart_send(m_uartHd,cmdvelBuffer.get(),31);
			break;
		}
		default:                              
		{
			break;
		}
	}
	
}





//命令控制
void HangfaPlatfromUartDriver::onHangfaCommandCb(const hangfa_uart_platform::HangfaCommand::ConstPtr &msg)
{
	ROS_INFO("Send command to main board.");
	boost::shared_array<char> commandBuffer(new char[8]);

	//Header
	commandBuffer[0] = SYNC_FLAG_START;
 	commandBuffer[1] = msg->device_type/*MOTION_DEVICE_TYPE*/;
	commandBuffer[2] = msg->device_address/*MOTION_DEVICE_ADDRESS*/;

	//Command
	commandBuffer[3] = msg->command_type;
	
	//Length
	commandBuffer[4] = 0x00;

	//CRC检校
	uint16_t crcValue = getCks((uint8_t*)(commandBuffer.get()),5);
	char* crc = hangfa_platform::int162bytes(crcValue);
	memcpy(commandBuffer.get()+5,crc,2);

	commandBuffer[7] = SYNC_FLAG_END;

	boost::mutex::scoped_lock lock(m_ttsStatusMutex);
	uart_send(m_uartHd,commandBuffer.get(),8);

#ifdef HANGFA_UART_DEBUG	
	for(int i=0;i<8;i++)
		ROS_INFO("%x",commandBuffer[i]);
#endif//HANGFA_UART_DEBUG
}





/*设置单个参数设置*/
void HangfaPlatfromUartDriver::onSingleParamSettingCb(const hangfa_uart_platform::SingleParamSetting::ConstPtr &msg)
{
	boost::shared_array<char> paramBuffer(new char[13]);

	//Header
	paramBuffer[0] = SYNC_FLAG_START;
 	paramBuffer[1] = MOTION_DEVICE_TYPE;
	paramBuffer[2] = MOTION_DEVICE_ADDRESS;

	//Command
	paramBuffer[3] = 0x1E;

	//Length
	paramBuffer[4] = 0x05;

	//Data
	paramBuffer[5] = msg->index;
	char* paramData = hangfa_platform::int322bytes(msg->data);
	memcpy(paramBuffer.get()+6,paramData,4);

	//CRC check
	uint16_t crcValue = getCks((uint8_t*)(paramBuffer.get()),10);
	char* crc = hangfa_platform::int162bytes(crcValue);
	memcpy(paramBuffer.get()+10,crc,2);

	paramBuffer[12] = SYNC_FLAG_END;

	boost::mutex::scoped_lock lock(m_ttsStatusMutex);
	uart_send(m_uartHd,paramBuffer.get(),13);
}


void HangfaPlatfromUartDriver::onAutoChargingCb(const std_msgs::Bool::ConstPtr& msg)
{
	boost::shared_array<char> autoChargeBuffer(new char[8]);
	autoChargeBuffer[0] = SYNC_FLAG_START;
	if(msg->data)//开始自动充电
	{
		autoChargeBuffer[1] = 0x01;//device type
		autoChargeBuffer[2] = 0x01;//device address
		autoChargeBuffer[3] = 0x2C;//开始自动充电
		autoChargeBuffer[4] = 0;//package length

		//CRC check
		autoChargeBuffer[5] = 0xC5;
		autoChargeBuffer[6] = 0x6E;

		autoChargeBuffer[7] = SYNC_FLAG_END;

		boost::mutex::scoped_lock lock(m_ttsStatusMutex);
		uart_send(m_uartHd,autoChargeBuffer.get(),13);
	}
	else
	{
		autoChargeBuffer[1] = 0x01;//device type
		autoChargeBuffer[2] = 0x01;//device address
		autoChargeBuffer[3] = 0x2D;//停止自动充电
		autoChargeBuffer[4] = 0;//package length

		//CRC check
		autoChargeBuffer[5] = 0xC5;
		autoChargeBuffer[6] = 0x6E;

		autoChargeBuffer[7] = SYNC_FLAG_END;

		boost::mutex::scoped_lock lock(m_ttsStatusMutex);
		uart_send(m_uartHd,autoChargeBuffer.get(),13);
	}
	m_isAutoCharging = msg->data;
}

//电源板功率控制
void  HangfaPlatfromUartDriver::onPowerManageCb(const hangfa_uart_platform::PowerManage::ConstPtr& msg)
{
	boost::shared_array<char> powerManageBuffer(new char[9]);
	powerManageBuffer[0] = SYNC_FLAG_START;
	
	powerManageBuffer[1] = 0x10;//device type
	powerManageBuffer[2] = 0x10;//device address

	powerManageBuffer[3] = 0x29;//电源板功率控制
	powerManageBuffer[4] = 1;   //package length
	
	//Param
	powerManageBuffer[5] = msg->load_switch | (msg->dcdc_mode <<1);

	//CRC check
	uint16_t crcValue = getCks((uint8_t*)(powerManageBuffer.get()),6);
	char* crc = hangfa_platform::int162bytes(crcValue);
	memcpy(powerManageBuffer.get()+6,crc,2);

	powerManageBuffer[8] = SYNC_FLAG_END;

	boost::mutex::scoped_lock lock(m_ttsStatusMutex);
	uart_send(m_uartHd,powerManageBuffer.get(),9);

}


void HangfaPlatfromUartDriver::onSonarAlarmCb(const hangfa_uart_platform::SetSonarAlarm::ConstPtr& msg)
{
	boost::shared_array<char> sonarAlarmBuffer(new char[10]);
	sonarAlarmBuffer[0] = SYNC_FLAG_START;
	
	sonarAlarmBuffer[1] = 0x20;//device type
	sonarAlarmBuffer[2] = 0x20;//device address

	sonarAlarmBuffer[3] = 0x1F;//sonar alarm
	sonarAlarmBuffer[4] = 2;   //package length
	
	//Param
	sonarAlarmBuffer[5] = msg->sonar_1_8;
	sonarAlarmBuffer[6] = msg->sonar_9_16;

	//CRC check
	uint16_t crcValue = getCks((uint8_t*)(sonarAlarmBuffer.get()),7);
	char* crc = hangfa_platform::int162bytes(crcValue);
	memcpy(sonarAlarmBuffer.get()+7,crc,2);

	sonarAlarmBuffer[9] = SYNC_FLAG_END;

	boost::mutex::scoped_lock lock(m_ttsStatusMutex);
	uart_send(m_uartHd,sonarAlarmBuffer.get(),10);
}

