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
PROXY_HEADER_LENGTH		= 79
SIGNATURE_LENGTH		= 16
MAC_LENGTH				= 6

HELLO_MESSAGE 			= 128 	#1000 0000
SUB_LIST 				= 64	#0100 0000

HSE 					= robot_black_magic
T_HDLR    				= topic_handler
topic_skt 				= None
db_enc 					= None

ETH_HEADER_STRUCTURE 	='!6s6s2s32s'		#MAC(6),MAC(6),PROTOCOL(2),CHALLENGE(32)
PROXY_HEADER_STRUCTURE 	='!1s6sd32s32s'		#HELLO:SUBLIST:::::::,SOURCE_MAC, TIMESTAMP (8bytes), new_C, new_R
INTERFACE				= HSE.INTERFACE
ROBOT_PROXY_PROTOCOL	= '\xF0\x0F'
GTG_PROTOCOL			= 0x0003

###################################################PAYLOAD HANDLEING####################################################


def process_payload(src_mac,payload):
	ps 					= PickleSend()
	ps.MAC 				= src_mac
	ps.pickled_message	= dumps(payload)
	msg_pub.publish(ps)

def Pickle_send_callback(Pickle_msg):
	global topic_skt
	global db_enc
	message			= loads(Pickle_msg.pickled_message)
	dest_mac		= Pickle_msg.MAC
	dest_mac_enc	= HSE.compute_hmac(dest_mac)
	#Check that the robot is addressable (eg. has CR pair)
	if dest_mac_enc in db_enc and is_conn_open(dest_mac):
		prep_send_packet(topic_skt, dest_mac, message)

#attach to our topic handler
T_HDLR.setHandle(Pickle_send_callback)

####################################################PACKET HANDLEING####################################################


def churn(mac_enc, new_CR):
	dest_mac_enc 			= HSE.compute_hmac(mac_enc)
	db_enc[dest_mac_enc] 	= HSE.enc_CR(new_CR)			#automatically creates entry if not already present!

def within_accuracy(timestamp_packet, timestamp_rcv, data_length):
	#Do other calculations to determine clock skew allowance. Must synchronize times for this to work.
	skew_proj = data_length/SAFE_TRNSFR_RATE #clock skew, propagation delay, and proc. delay can also affect allowance
	return  abs(timestamp_packet + skew_proj - timestamp_rcv) < TIMESTAMP_ALLOWANCE


def MAC_msg_listener(connection_skt):

	while True:
		packet 			= connection_skt.recvfrom(65565)[0]
		timestamp_rcv 	= time()

		if len(packet) >= ETH_HEADER_LENGTH:
			process_packet(connection_skt,packet,timestamp_rcv)


def process_packet(skt,packet,timestamp_rcv):
	eth_header 							= packet[:ETH_HEADER_LENGTH]
	dest_mac,src_mac,eth_protocol,ch 	= unpack(ETH_HEADER_STRUCTURE , eth_header)
	if eth_protocol == ROBOT_PROXY_PROTOCOL and src_mac != HSE.MAC:
		if dest_mac == HSE.MAC:
			#pudb.set_trace() #For Debugging
			process_directed_packet(skt,src_mac,ch,packet[ETH_HEADER_LENGTH:])
		if dest_mac == BROADCAST_MAC:
			process_broadcast(skt,src_mac,ch)


def process_directed_packet(skt,src_mac,ch,proxy_msg):
	try:
		msg 							= HSE.dec_msg(ch, proxy_msg)
	except ValueError:
		#Message not intended for this machine
		return
	proxy_header 						= msg[:PROXY_HEADER_LENGTH]
	flgs,MAC_sender,ts, new_C, new_R	= unpack(PROXY_HEADER_STRUCTURE, proxy_header)
	if src_mac == MAC_sender: # and HSE.verify_msg(ch, sg, sg_msg[SIGNATURE_LENGTH:]): #\ No need anymore
	#and within_accuracy(unpack('f',ts), timestamp_rcv, len(packet)):
		churn(src_mac, [new_C,new_R])			#Done BEFORE response to accomodate new agents
		process_verified_payload(skt,src_mac,msg[PROXY_HEADER_LENGTH:], ord(flgs))


def process_verified_payload(skt,src_mac,pickled_payload, flags_int):									
	payload = loads(pickled_payload)
	try:
		if flags_int & HELLO_MESSAGE > 0 or flags_int & SUB_LIST > 0:
			if flags_int & HELLO_MESSAGE > 0:
				msg_to_send = get_self_pickled_publisher_list()
				prep_send_packet(skt,src_mac, msg_to_send, SUB_LIST)
			update_connection(src_mac, payload[1:])
		else:
			process_payload(src_mac,payload)
	except TypeError:
		process_payload(src_mac,payload)


def process_broadcast(skt,src_mac,pubs_hash):
	#pudb.set_trace() #For Debugging
	enc_mac 	= HSE.compute_hmac(src_mac)
	if not is_conn_open(src_mac) and enc_mac in db_enc:					#ignore unknown macs (They will auth later)
		msg_to_send =  get_self_pickled_publisher_list()
		prep_send_packet(skt,src_mac, msg_to_send, HELLO_MESSAGE)
	elif is_conn_open(src_mac):
		renew_connection(skt,src_mac, pubs_hash)


def send_packet(skt, dest_mac, enc_msg):
	src_MAC 	= HSE.MAC
	skt.send(dest_mac + src_MAC + ROBOT_PROXY_PROTOCOL + enc_msg)


def prep_send_packet(skt, dest_mac, payload, flags = 0):
	CR 				= get_host_CR(dest_mac)
	enc_msg 		= construct_message(payload,CR[0], CR[1], flags)
	send_packet(skt, dest_mac, enc_msg)


def construct_message(payload, challenge, response, flags):
	pickle_array	= dumps(payload,2)
	eth_header 		= chr(flags) + HSE.MAC + pack('d',time()) + HSE.new_CR()
	msg 			= eth_header + pickle_array
	return challenge + HSE.enc_msg(response, msg)



def get_host_CR(mac):
	dest_mac_enc	= HSE.compute_hmac(mac)
	return HSE.dec_CR(db_enc[dest_mac_enc])


def MAC_broadcaster(broadcast_skt, tl_pub):

	#pudb.set_trace() #For Debugging
	BROADCAST_FREQUENCY		= rospy.Rate(1) #in Hz
	while True:
		T_HDLR.clear_ttls()
		publisher_hash 			= get_self_publisher_hash()

		pubLists_msg 			= pubLists()
		T_HDLR.get_all_ext_pubs(pubLists_msg)
		
		tl_pub.publish(pubLists_msg)
		send_packet(broadcast_skt, BROADCAST_MAC,publisher_hash)

		BROADCAST_FREQUENCY.sleep()


####################################################TOPIC HANDLEING#####################################################


def is_conn_open(src_mac):
	T_HDLR.is_conn_open(src_mac)


def update_connection(src_mac, pickled_pubs_list):
	T_HDLR.update_connection(src_mac, pickled_pubs_list)


def renew_connection(skt,src_mac, pubs_hash):
	if not T_HDLR.has_same_pubs_hash(src_mac, pubs_hash):
		prep_send_packet(skt,src_mac, None, SUB_LIST)


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
		topic_skt 				= socket.socket( socket.AF_PACKET , socket.SOCK_RAW , socket.ntohs(GTG_PROTOCOL)) #GTG protocol
		topic_skt.bind((INTERFACE, GTG_PROTOCOL))

		broadcast_snd_skt 		= socket.socket( socket.AF_PACKET , socket.SOCK_RAW , socket.ntohs(GTG_PROTOCOL)) #GTG protocol
		broadcast_snd_skt.bind((INTERFACE, 0))

		connection_skt 			= socket.socket( socket.AF_PACKET , socket.SOCK_RAW , socket.ntohs(GTG_PROTOCOL)) #GTG protocol
		connection_skt.bind((INTERFACE, GTG_PROTOCOL))

	except socket.error , msg:
		print 'Socket could not be created. Error Code : ' + str(msg[0]) + ' Message ' + msg[1] + " Are you su?"
		sys.exit()

	tl_pub 	= rospy.Publisher('msg_proxy/Host_lists', pubLists, queue_size=100)
	msg_pub	= rospy.Publisher('msg_proxy/Pickle_msg_out', PickleSend, queue_size=100)

	listener_thread 		= Thread(target = MAC_msg_listener, args = (connection_skt, ))
	listener_thread.setDaemon(True)
	listener_thread.start()

	broadcast_thread 		= Thread(target = MAC_broadcaster, args = (broadcast_snd_skt,tl_pub, ))
	broadcast_thread.setDaemon(True)
	broadcast_thread.start()

	rospy.spin()