#!/usr/bin/env python
import rospy
import sys
import fcntl, socket, struct
import hashlib
#import pudb

from ros_secure_com.msg 	import DBupdate, PickleSend
from cPickle				import loads, dumps
from numpy 					import array
from time 					import sleep

FILE_PATH0 		= 'db_PC_eth0'
FILE_PATH1 		= 'db_ZED1_eth0'
FILE_PATH2 		= 'db_ZED3_eth2'

m0 = '\x20\x6A\x8A\x29\x35\x18' #PC[Ethernet]
m1 = '\x00\x0A\x35\x00\x01\x22' #ZED1[Ethernet]
m2 = '\x00\x0A\x35\x00\x01\x24'	#ZED3[Ethernet]
#...

k0 = '\x0b\xf5\xbd\xdam\\\x9f-\x96x*\xab\xdfe\xbc\x82\x89x\xae$Q\xaa\xf8g\xb12:\xec\x80o\xc0@\x83\xd4\xa7\x03g\x96\x00\xb5U\x8a!\x07\xda\xdd\xe0M*\xa3\xfa[\x056V\x9e\xf8b\xad\xefO\x1f\xf0\x1f\xab3\xdc\x11<\xc0\xef3\x94\xff\xc4\xa9\xfeIg\xc7q\xc3\xbdW\x00\xa5I\xd9\xdd\xcb\xd1\xadZ\xdb\xda\xbe\x89\xe2\xc7s25\xffs\xa4\xc9$/\xc6\xa5\xf3\xdc\xab\x9b2\xea~\xc7j\xb7r\x1bE\xc3\xa00\x07\xa2'
k1 = '\x83\x00\xb6T~<\xb4J\xe9\xbf\xb9\xb8a\xc9A*\xf7\xb4\x96\xb6\xda\x98\xa3\xd4\xa0\xf2\xb03\xba\xfa\xe5\xc1\xaewI-3\xb8=\x87\xf2\x06\'\x87\xd8\xb9\xfc\x17\x1f\xe11LU7,\x0feH\x99\x1f8\x9c\xec\xca\xfa\x1a\xd5\xe7o{\x9d\xab\x98o\xb5\xd0{\xc8\xf2f\x03\xads\xfdi\xdf7\xaf\xc5\xa7\x9f\xcb$!\x15"m\x91\xabYK\xfezQ\xb0q\xbc\xa0\xd5\x12\x97\xeb\xa2\xce]\x18`\xacR\xee\xe6@\xd2\xdb\xe4\xaa\x04\xed'
k2 = '\x1cY\xfd$#\xd0\xe1qvU\x0b\tZ\xcfN\xcd\x8a\x8fx\xa2\\I\xb6\x83\tE\x00Rc\xb5\x8e\x18]V\xb2\xf2\x85\xfb\x8fthdH\x1d)\xeb\xff\x86\xbc\x1c\xe3G\xdf\x9e\x1a\t\x1e\xbb\x07\xa8s\xb7\xe6(w\xc8\x89\xee\xf1\xe8\xaf\x98\x83,`/u\xb3\x8f\xec\xa8\xdfN\xf4\x178\xff\xa4p\x86\xbd\xa39\xf1\x83}(\x07\x14\xd8M\xbc!6\xfbG\xdd\x1b\xddP\x8c\xaa\x10\x9aT\x8ds\xe1\xa5\x88/e\xdb\x8e\xae\x00\xf1'
#...

db_pub 		= None
db_file_pub	= None

def publish_db_upd():
	upd_db 			= DBupdate()

	upd_db.macList 	= dumps([m0,m1,m2])
	upd_db.hwKeys	= dumps([k0,k1,k2])

	while not rospy.is_shutdown() and not db_pub.get_num_connections() > 0:
		sleep(.1)
	if db_pub.get_num_connections() > 0:
		db_pub.publish(upd_db)
		print('published update msg')


def publish_db_write():	
	file_write0 	= PickleSend()
	file_write1 	= PickleSend()
	file_write2 	= PickleSend()

	file_write0.MAC	= m0
	file_write1.MAC	= m1
	file_write2.MAC	= m2

	file_write0.pickled_message	= dumps(FILE_PATH0)
	file_write1.pickled_message	= dumps(FILE_PATH1)
	file_write2.pickled_message	= dumps(FILE_PATH2)

	while  not rospy.is_shutdown() and not db_file_pub.get_num_connections() > 0:
		sleep(.1)

	if db_pub.get_num_connections() > 0:
		db_file_pub.publish(file_write0)
		db_file_pub.publish(file_write1)
		db_file_pub.publish(file_write2)

		print('published write to files')


if __name__ == '__main__':
	rospy.init_node('upd_CB_example', anonymous=True)
	db_pub 		= rospy.Publisher('db_update', DBupdate, queue_size=100)
	db_file_pub = rospy.Publisher('db_request', PickleSend, queue_size=100)

	publish_db_upd()
	publish_db_write()
