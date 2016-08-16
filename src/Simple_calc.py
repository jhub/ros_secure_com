#!/usr/bin/env python
import rospy
import sys
import pudb

from time 					import sleep
from std_msgs.msg  			import String
from ros_secure_com.msg 	import PickleSend, pubLists, MACmlist, topicTypeList
from cPickle				import loads, dumps
from binascii 				import hexlify

Publishers 		=	{}
MESSAGE_TO_S 	= None

def pickle_msg_callback(msg):
	print("got: " + str(loads(msg.pickled_message).data) + "  from mac: " + hexlify(msg.MAC))


def Host_update_callback(pubLists):
	global MESSAGE_TO_S
	if MESSAGE_TO_S == None:
		MESSAGE_TO_S 	=	"Default msg"

	global Publishers
	for maclist in pubLists.MACmlists:
		if not maclist.MAC in Publishers:
			pub = rospy.Publisher('msg_proxy/ext/' + maclist.MAC, PickleSend, queue_size=100)
			Publishers[maclist.MAC] = pub
			#publish
			i = String()
			i.data = MESSAGE_TO_S
			try:
				pub.publish(i)
			except TypeError:
				print("Evaded incorrect publish")
			ps = PickleSend()
			ps.MAC 				= maclist.MAC
			ps.pickled_message	= dumps(i)

			sleep(10)

			pub.publish(ps)
			print("Published!")

			sleep(10)

			#pub.unregister()
			#del pub
			#print("unregistered!")


if __name__ == '__main__':
	if len(sys.argv) > 1:
		global MESSAGE_TO_S
		MESSAGE_TO_S	= " ".join(sys.argv[1:])
	#pudb.set_trace() #For Debugging

	rospy.init_node('send_my_string', anonymous=True)
	rospy.Subscriber("msg_proxy/Host_lists", pubLists, Host_update_callback)
	rospy.Subscriber("msg_proxy/Pickle_msg_out", PickleSend, pickle_msg_callback)

	rospy.spin()