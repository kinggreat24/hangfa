#! /usr/bin/env python

import rospy
import pdb
from hangfa_uart_platform.msg import SingleParamSetting

if __name__ == '__main__':
	rospy.init_node('talker', anonymous=True)
	pub = rospy.Publisher('/singleParamSetting', SingleParamSetting, queue_size=1)

	singleParamSetting = SingleParamSetting()
    	singleParamSetting.index = 30
	pdb.set_trace()
	singleParamSetting.datas[0] = b'\x10'
	singleParamSetting.datas[1] = b'\x27'
	singleParamSetting.datas[2] = b'\x00'
	singleParamSetting.datas[3] = b'\x00'

	pub.publish(singleParamSetting)
