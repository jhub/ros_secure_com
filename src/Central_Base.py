#!/usr/bin/env python
import rospy
import sys
import fcntl, socket, struct
import hashlib
import pudb
import hmac
import robot_black_magic

from numpy 					import array
from os 					import urandom, path
from Crypto.Cipher 			import AES
from cPickle				import loads, dumps
from binascii				import unhexlify

from ros_secure_com.msg 	import PickleSend, DBupdate

Central_Base_object 	= None
key 					= '\x93\x1d\xb3x\r\xeb\xa3\xbf\xd5\xec+\xe6\x00Iz\x12\x965\x1f\x86\n\xc9\xc7\xe9*\xe9\xfe\x00\x16KdD\x85\\s\xf7\xaf\xbd\xe9\x05`!\xaa<v\xe1w\xee\'\xad\xef\x0cf\x15}\x95r\x91\xf1\x08*\xaat\x00-\xbeE\xa9]\x14\x9d\x90\xff\r\xd7W.\x8f\xd2\xd9O\x1e\x17 \n\xa5m\xb6\xa1\xcbs9\x8c|\xc0\xe7{"(r\xd1\xc8a\xb8\xb2\xdeT\x85"\xce\x82\xd9\xb9\xe4d\x05]\xf9K\x86c\xd4U8\xe8\xe1\xdb\xb4'

FILE_EXISTS_ERROR 		= "File already exists!!"
DIR_ERROR				= "Directory not found!!"


MAC_INDEX 				= 0
HW_INDEX				= 1

PICKLE_PROTOCOL			= 2

CR_KEY_SIZE				= 32

HSE 					= robot_black_magic

def get_self_MAC(hwr_intrf):
	s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	info = fcntl.ioctl(s.fileno(), 0x8927,  struct.pack('256s', hwr_intrf[:15]))
	return info[18:24]

central_MAC 			= unhexlify(get_self_MAC('wlan0')[0:6].encode('hex'))
MAC_hardwr_keys 		= {}#{central_MAC:key} if we want to communicate with the central base


def update_MAC_hardwr_keys(MAC_array_list, hardware_key_list):
	dict_out = {}
	for mk in range(len(MAC_array_list)):
		dict_out[MAC_array_list[mk]]	= hardware_key_list[mk]
	return dict_out


def get_db(mac_address):
	global MAC_hardwr_keys
	self_hw_key			= MAC_hardwr_keys[mac_address]
	db = {}
	pudb.set_trace()  #For Debugging
	for mac,hw_key in MAC_hardwr_keys.iteritems():
		if(mac_address == mac): 							#skip one's own MAC
			continue

		ch 				= urandom(CR_KEY_SIZE)
		resp 			= HSE.get_response(hw_key, ch)

		CR_entry_enc	= HSE.enc_CR([ch,resp], self_hw_key)

		mac_enc			= HSE.compute_hmac(mac, self_hw_key)

		db[mac_enc]		= CR_entry_enc

	return db


def db_update_callback(db_update):
	mac_list 	= loads(db_update.macList)
	hw_keys 	= loads(db_update.hwKeys)
	try:
		if len(mac_list) == len(hw_keys):
			global MAC_hardwr_keys
			#pudb.set_trace()  #For Debugging
			tmp_dict = update_MAC_hardwr_keys(mac_list, hw_keys)
			for key, value in tmp_dict.iteritems():
				MAC_hardwr_keys[key] = value
			print("Success with update!")
	except Exception, e:
		print("Error with input data :(")


def db_request_callback(db_request):
	#pudb.set_trace()  #For Debugging
	try:
		mac 				= db_request.MAC
		file_path 			= loads(db_request.pickled_message)
		pickled_message 	= dumps(get_db(mac),2)
		if path.exists(file_path):
			print(FILE_EXISTS_ERROR)
		elif  path.dirname(file_path) != '' and not path.lexists(path.dirname(file_path)):
			print(DIR_ERROR)
		else:
			f = open(file_path, 'w')
			f.write(pickled_message)
			f.close
			print("Success with writing db to: " + file_path)
	except Exception, e:
		print("Error with writing data :(")


if __name__ == '__main__':

	rospy.init_node('Central_Base', anonymous=True)

	rospy.Subscriber("db_update", DBupdate, db_update_callback)
	rospy.Subscriber("db_request", PickleSend, db_request_callback)

	rospy.spin()