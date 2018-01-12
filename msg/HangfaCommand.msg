#device type
uint8 DeviceType_RobotHost = 1
uint8 DeviceType_PowerManage = 16
uint8 DeviceType_MotionControl = 32
uint8 DeviceType_UltrasonicInfrared = 49
uint8 DeviceType_Ultrasonic = 50
uint8 DeviceType_Acceleration = 51
uint8 DeviceType_Gyroscope = 52
uint8 DeviceType_Compass= 53
uint8 DeviceType_GPS = 54
uint8 DeviceType_Inclinometer = 55
uint8 DeviceType_Lidar = 56
uint8 DeviceType_EncoderPosition = 57
uint8 DeviceType_AreaPosition = 58
uint8 DeviceType_LaserInfrared = 59
uint8 DeviceType_FindMarkingLine = 60
uint8 DeviceType_Broadcast = 255


#command type
uint8 Device_type = 20
uint8 Device_version = 21
uint8 Device_serialNumber = 22
uint8 Save_config = 23
uint8 Restore_factory_setting = 24
uint8 Device_restart = 25
uint8 GetDeviceConfig = 27
uint8 SetDeviceConfig = 28
uint8 GetSingleParam = 29
uint8 SetSingleParam = 30
uint8 SetSonarAlarmState = 31
uint8 Respond = 255

#specific command 
uint8 ReadSampleValueOnce = 41
uint8 GetDeviceStatus  = 42
uint8 RestoreMotorDriver = 43
uint8 ClearBuffer = 44
uint8 ClearErrorFlag = 45
uint8 BeginAction = 46
uint8 PauseAction = 47
uint8 ContinueAction = 48
uint8 StopAction = 49
uint8 JumpToNextCommand = 50
uint8 StopAndSetOrientation = 51
uint8 CompleteAndSetOrientation = 52
uint8 HoldPosition = 53
uint8 RealSpeed = 54
uint8 SpeedWithRotationLinearMotion = 55
uint8 SpeedWithRotationClockwiseMotion = 56
uint8 SpeedWithRotationCounterclockwise = 57
uint8 CustomSpeedMotion = 58
uint8 SpeedCycleMotion = 59
uint8 RelativePositionWithRotationLinearMotion = 60


uint8 JoystickControlMotion = 66

uint8 device_type
uint8 device_address
uint8 command_type

