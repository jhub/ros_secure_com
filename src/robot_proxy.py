#!/usr/bin/env python
import rospy
import sys
import fcntl, socket, struct
import pudb
import robot_black_magic
import topic_handler

from threading 				import Thread
from time 					import time, sleep
from cPickle				import loads, dumps
from struct					import unpack,pack

from ros_secure_com.msg 	import PickleSend, pubLists

#######################################################CONSTANTS########################################################

NOT_FOUND_MSSG 			= "The file given was not found"
INCORRECT_FORMAT_MSSG	= "The file given does not contain the correct database"

CHALLENGE_INDEX 		= 0
RESPONSE_INDEX			= 0
DATA_INDEX				= 0

BROADCAST_FREQUENCY		= 3 #in seconds
BROADCAST_MAC			= '\xFF\xFF\xFF\xFF\xFF\xFF'

SAFE_TRNSFR_RATE		= 64 * 1000 #64 kB = 512 kb, suggested transfer speed
TIMESTAMP_ALLOWANCE 	= 1e-5

ETH_HEADER_LENGTH 		= 46
PROXY_HEADER_LENGTH		= 26
SIGNATURE_LENGTH		= 16
MAC_LENGTH				= 6

ETH_HEADER_STRUCTURE 	='!6s6s2s32s'		#MAC(6),MAC(6),PROTOCOL(2),CHALLENGE(32)
PROXY_HEADER_STRUCTURE 	='!16s6s4s'			#SIGNATURE, SOURCE_MAC, TIMESTAMP
INTERFACE				= 'eth0'
ROBOT_PROXY_PROTOCOL	= '\xF0\x0F'
GTG_PROTOCOL			= 0x0003

HELLO_MESSAGE 			= 'A'
SUB_LIST 				= 'R'

HSE 					= robot_black_magic
T_HDLR    				= topic_handler
topic_skt 				= None
db_enc 					= None

###################################################PAYLOAD HANDLEING####################################################


def process_payload(src_mac,payload):
	pudb.set_trace() #For Debugging
	print("Hi!! Message received :D")
	#TODO publish message on correct topic (hopefully one indicative of the src mac)

def Pickle_send_callback(Pickle_msg):
	global topic_skt
	message			= loads(Pickle_msg.pickled_message)
	dest_mac		= Pickle_msg.MAC
	dest_mac_enc	= HSE.compute_hmac(dest_mac)
	#Check that the robot is addressable (eg. has CR pair)
	if dest_mac_enc in db_enc and is_conn_open(dest_mac):
		prep_send_packet(topic_skt, dest_mac, message)


####################################################PACKET HANDLEING####################################################


def churn(mac_enc, new_CR):
	dest_mac_enc 			= HSE.compute_hmac(mac_enc)
	db_enc[dest_mac_enc] 	= HSE.enc_CR(new_CR)			#automatically creates entry if not already present!


def within_accuracy(timestamp_packet, timestamp_rcv, data_length):
	#Do other calculations to determine clock skew allowance
	skew_proj = data_length/SAFE_TRNSFR_RATE #clock skew, propagation delay, and proc. delay can also affect allowance
	return  abs(timestamp_packet + skew_proj - timestamp_rcv) < TIMESTAMP_ALLOWANCE


def MAC_msg_listener(arg):
	try:
		skt 			= socket.socket( socket.AF_PACKET , socket.SOCK_RAW , socket.ntohs(GTG_PROTOCOL)) #GTG protocol
		skt.bind((INTERFACE, GTG_PROTOCOL))
	except socket.error , msg:
		print 'Socket could not be created. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
		sys.exit()
	while True:
		packet 			= skt.recvfrom(65565)[0]
		timestamp_rcv 	= time()

		if packet > ETH_HEADER_LENGTH:
			process_packet(skt,packet,timestamp_rcv)


def process_packet(skt,packet,timestamp_rcv):
	eth_header 							= packet[:ETH_HEADER_LENGTH]
	dest_mac,src_mac,eth_protocol,ch 	= unpack(ETH_HEADER_STRUCTURE , eth_header)
	if eth_protocol == ROBOT_PROXY_PROTOCOL:
		if dest_mac == HSE.MAC:
			#pudb.set_trace() #For Debugging
			process_directed_packet(skt,src_mac,ch,packet[ETH_HEADER_LENGTH:])
		if dest_mac == BROADCAST_MAC and src_mac != HSE.MAC:
			process_broadcast(skt,src_mac,ch)


def process_directed_packet(skt,src_mac,ch,proxy_msg):
	try:
		sg_msg 						= HSE.dec_msg(ch, proxy_msg)
	except ValueError:
		#Message not intended for this machine
		return
	proxy_header 					= sg_msg[:PROXY_HEADER_LENGTH]
	sg,MAC_sender,timestamp 		= unpack(PROXY_HEADER_STRUCTURE, proxy_header)
	if src_mac == MAC_sender and HSE.verify_msg(ch, sg, sg_msg[SIGNATURE_LENGTH:]): #\
	#and within_accuracy(unpack('f',timestamp), timestamp_rcv, len(packet)):
		pickle_array = loads(sg_msg[PROXY_HEADER_LENGTH:])
		process_verified_payload(skt,src_mac,pickle_array[0],pickle_array[1])


def process_verified_payload(skt,src_mac,new_CR,pickled_payload):
	churn(src_mac, new_CR)													#Done BEFORE response to accomodate new agents
	payload = loads(pickled_payload)
	if payload[0] == HELLO_MESSAGE or payload[0] == SUB_LIST:
		if payload[0] == HELLO_MESSAGE:
			msg_to_send = SUB_LIST + get_self_pickled_publisher_list()
			prep_send_packet(skt,src_mac, msg_to_send)
		update_connection(src_mac, payload[1:])
	else:
		process_payload(src_mac,payload)


def process_broadcast(skt,src_mac,pubs_hash):
	#pudb.set_trace() #For Debugging
	enc_mac 	= HSE.compute_hmac(src_mac)
	if not is_conn_open(src_mac) and enc_mac in db_enc:					#ignore unknown macs (They will auth later)
		msg_to_send = HELLO_MESSAGE + get_self_pickled_publisher_list()
		prep_send_packet(skt,src_mac, msg_to_send)
	elif is_conn_open(src_mac):
		renew_connection(skt,src_mac, pubs_hash)


def send_packet(skt, dest_mac, enc_sg_msg):
	src_MAC 	= HSE.MAC
	skt.send(dest_mac + src_MAC + ROBOT_PROXY_PROTOCOL + enc_sg_msg)


def construct_message(payload, challenge, response):
	pickle_array	= [HSE.new_CR(),dumps(payload,2)]
	msg 			= HSE.MAC + pack('f',time()) + dumps(pickle_array,2)
	sg 				= HSE.compute_hmac(msg, response)
	msg_sg 			= sg + msg
	return challenge + HSE.enc_msg(response, msg_sg)


def prep_send_packet(skt, dest_mac, payload):
	CR 				= get_host_CR(dest_mac)
	enc_sg_msg 		= construct_message(payload,CR[0], CR[1])
	send_packet(skt, dest_mac, enc_sg_msg)


def get_host_CR(mac):
	dest_mac_enc	= HSE.compute_hmac(mac)
	return HSE.dec_CR(db_enc[dest_mac_enc])


def MAC_broadcaster(arg):
	try:
		skt 	= socket.socket( socket.AF_PACKET , socket.SOCK_RAW , socket.ntohs(GTG_PROTOCOL)) #GTG protocol
		skt.bind((INTERFACE, 0))
	except socket.error , msg:
		print 'Socket could not be created. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
		sys.exit()
	#pudb.set_trace() #For Debugging
	while True:
		T_HDLR.clear_ttls()
		publisher_hash 			= get_self_publisher_hash()

		pubLists_msg 			= pubLists()
		T_HDLR.get_all_ext_pubs(pubLists_msg)
		
		tl_pub.publish(pubLists_msg)
		send_packet(skt, BROADCAST_MAC,publisher_hash)

		sleep(BROADCAST_FREQUENCY)


####################################################TOPIC HANDLEING#####################################################


def is_conn_open(src_mac):
	T_HDLR.is_conn_open(src_mac)


def update_connection(src_mac, pickled_pubs_list):
	T_HDLR.update_connection(src_mac, pickled_pubs_list)


def renew_connection(skt,src_mac, pubs_hash):
	if not T_HDLR.has_same_pubs_hash(src_mac, pubs_hash):
		prep_send_packet(skt,src_mac, SUB_LIST)


def get_self_pickled_publisher_list():
	return T_HDLR.get_self_pickled_publisher_list()


def get_self_publisher_hash():
	return T_HDLR.get_local_pubs_hash()


#####################################################INITIALIZATION#####################################################


def load_table(file_name):
	with open(file_name, 'rb') as handle:
		return loads(handle.read())


def usage():
	return "%s: [db_file_name]"%sys.argv[0]


if __name__ == '__main__':
	#pudb.set_trace() #For Debugging
	if len(sys.argv) == 2:
		try:
			db_enc = load_table(sys.argv[1])
			if type(db_enc) is not dict:
				print(NOT_CORRECT_FORMAT_MSSG)
				sys.exit(1)
		except IOError:
			print(NOT_FOUND_MSSG)
			sys.exit(1)
	else:
		print usage()
		sys.exit(1)

	rospy.init_node('robot_proxy', anonymous=True)

	try:
		topic_skt 	= socket.socket( socket.AF_PACKET , socket.SOCK_RAW , socket.ntohs(GTG_PROTOCOL)) #GTG protocol
		topic_skt.bind((INTERFACE, GTG_PROTOCOL))
	except socket.error , msg:
		print 'Socket could not be created. Error Code : ' + str(msg[0]) + ' Message ' + msg[1] + " Are you su?"
		sys.exit()

	listener_thread 		= Thread(target = MAC_msg_listener, args = (10, ))
	listener_thread.setDaemon(True)
	listener_thread.start()

	broadcast_thread 		= Thread(target = MAC_broadcaster, args = (10, ))
	broadcast_thread.setDaemon(True)
	broadcast_thread.start()

	rospy.Subscriber("msg_proxy/Pickle_msg_out", PickleSend, Pickle_send_callback)

	tl_pub = rospy.Publisher('msg_proxy/Host_lists', pubLists, queue_size=100)

	rospy.spin()