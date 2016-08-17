#!/usr/bin/env python
import rospy
import sys
import pudb

from time 					import sleep
from std_msgs.msg  			import String
from ros_secure_com.msg 	import PickleSend, pubLists, MACmlist, topicTypeList
from cPickle				import loads, dumps
from binascii 				import hexlify
from os						import urandom

Publishers 		=	{}
MESSAGE_TO_S 	= None
Strt_Colc		= False

def pickle_msg_callback(msg):
	if not Strt_Colc:
		Strt_Colc = True
		print("Getting data")


def Host_update_callback(pubLists):
	global MESSAGE_TO_S
	if MESSAGE_TO_S == None:
		MESSAGE_TO_S 	=	urandom(1400)
		i 				= String()
		i.data 			= MESSAGE_TO_S
		msg 			= dumps(i)

	global Publishers
	for maclist in pubLists.MACmlists:
		if not maclist.MAC in Publishers:
			pub = rospy.Publisher('msg_proxy/ext/' + maclist.MAC, PickleSend, queue_size=100)
			Publishers[maclist.MAC] = pub
			#publish

			ps 					= PickleSend()
			ps.MAC 				= maclist.MAC
			ps.pickled_message	= msg

			while True:
				pub.publish(ps)
				sleep(10)


if __name__ == '__main__':
	if len(sys.argv) > 1:
		global MESSAGE_TO_S
		MESSAGE_TO_S	= " ".join(sys.argv[1:])
	#pudb.set_trace() #For Debugging

	rospy.init_node('send_my_string', anonymous=True)
	rospy.Subscriber("msg_proxy/Host_lists", pubLists, Host_update_callback)
	rospy.Subscriber("msg_proxy/Pickle_msg_out", PickleSend, pickle_msg_callback)

	rospy.spin()