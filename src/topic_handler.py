#!/usr/bin/env python
import rospy
import sys
import threading
import rosgraph.masterapi
import hashlib
import pudb
import robot_proxy

from cPickle				import loads, dumps
from time 					import time
from pybloomfilter 			import BloomFilter
from binascii 				import unhexlify

from ros_secure_com.msg 	import PickleSend, pubLists, MACmlist, topicTypeList

TOPIC_HND_LIST			= {}				#Used to keep a list of all external subscriptions {MAC:Topic_obj}
EXTERNAL_TOPICS 		= '/msg_proxy/ext'
SELF_PUBLISHER_LIST 	= []				#Used to keep a list of all our subscriptions [[[topic,type],subs_obj],...]
PUBLISHER_BF 			= BloomFilter(10000000, 0.01)
TTLVALUE 				= 18.0				#TTL for publisher lists; chosen to give at least 3 broadcasts to renew
i_lock 					= threading.Lock()
lm	 					= rosgraph.masterapi.Master('/topic_handler')

PUB_ERROR_COUNT 		= 0
MAX_PUB_ERROR_COUNT 	= 3

class topic_handler_obj(object):

	def __init__(self, pickled_pubs_list):
		self.PUBLISHERS 	= loads(pickled_pubs_list) #TODO make into custum msg format: topicTypeList[]
		self.time_created	= time()


	def upd_pubs(self, pickled_pubs_list):
		self.PUBLISHERS 	= loads(pickled_pubs_list)
		self.time_created	= time()

	def get_hash(self):
		return unhexlify(hashlib.sha256(self.PUBLISHERS).hexdigest())


	def get_publishers(self):
		return self.PUBLISHERS


	def has_expired(self, TTL_value):
		return time() - self.time_created > TTL_value


def is_conn_open(src_mac):
	return src_mac in TOPIC_HND_LIST


def clear_ttls():
	i_lock.acquire()
	global TOPIC_HND_LIST
	global TTLVALUE
	TOPIC_HND_LIST = {key: value for key, value in TOPIC_HND_LIST.items() if not value.has_expired(TTLVALUE)}
	#print(TOPIC_HND_LIST)
	i_lock.release()


def update_connection(src_mac, pickled_pubs_list):
	i_lock.acquire()
	global TOPIC_HND_LIST
	if is_conn_open(src_mac):
		TOPIC_HND_LIST[src_mac].upd_pubs(pickled_pubs_list)
	else:
		print("Obtained new connection from: " + ":".join("{:02x}".format(ord(c)) for c in src_mac))
		TOPIC_HND_LIST[src_mac] = topic_handler_obj(pickled_pubs_list)
	i_lock.release()


def get_all_ext_pubs(pubLists_msg): #TODO: For internal nodes to know outer pub lists
	for mac,th_obj in TOPIC_HND_LIST.iteritems():
		#pudb.set_trace() #For Debugging
		mlist 			= MACmlist()
		mlist.MAC 		= "".join("{:02x}".format(ord(c)) for c in mac)
		mlist.ttLists	= th_obj.get_publishers()
		pubLists_msg.MACmlists.append(mlist)


def get_publisher_list(mac):
	return TOPIC_HND_LIST[mac].get_publishers() if is_conn_open(mac) else '\x00'


def has_same_pubs_hash(src_mac, pubs_hash):
	return pubs_hash == get_pubs_hash(src_mac)


def get_pubs_hash(mac):
	return TOPIC_HND_LIST[mac].get_hash() 		if is_conn_open(mac) else '\x00'


def get_pickled_publisher_list(mac):
	dumps(get_publisher_list(mac))


#######################################################LOCAL PUBS#######################################################


def get_local_pubs_hash():
	return unhexlify(hashlib.sha256(get_self_pickled_publisher_list()).hexdigest())


def get_self_pickled_publisher_list():
	return dumps(resolve_all_local_pubs())


def resolve_all_local_pubs():
	global PUB_ERROR_COUNT
	try:
		#pudb.set_trace() #For Debugging
		publist 		= lm.getPublishedTopics(EXTERNAL_TOPICS) 	#Get list of topics that can be subscribed to
		update_ros_subscribers(publist)							#Update all our subscribers for the previous publishers
		PUB_ERROR_COUNT = 0
	except Exception:
		PUB_ERROR_COUNT += 1
		if PUB_ERROR_COUNT > MAX_PUB_ERROR_COUNT:
			print("Error in getting published topics, using the previous ros publishers")
	return get_MACmlists()


def get_MACmlists(): #TODO: For external nodes to know self pub lists
	global SELF_PUBLISHER_LIST
	topicTypeListArray = []
	for subs in SELF_PUBLISHER_LIST:
		top_typ 		= topicTypeList()
		top_typ.topic 	= subs[0][0]
		top_typ.type 	= subs[0][1] #PickleSend by default
		topicTypeListArray.append(top_typ)
	return topicTypeListArray


######################################################SUBSCRIBERS#######################################################


def update_ros_subscribers(publist):
	global PUBLISHER_BF
	print publist
	if not does_bf_contain_all(publist):
		print("Adding new node!")
		PUBLISHER_BF.clear_all()
		unsubscribe_all()
		subscribe_to(publist)


def subscribe_to(publist):
	global SELF_PUBLISHER_LIST #[[[topic,type],subs_obj],...]
	global PUBLISHER_BF
	
	for tt_pair in publist:
		#Forces topic to receive PickleSend to support future compatability
		if tt_pair[1] == 'ros_secure_com/PickleSend':
			subs_obj = rospy.Subscriber(tt_pair[0], PickleSend, robot_proxy.Pickle_send_callback)
			SELF_PUBLISHER_LIST.append([tt_pair,subs_obj]) #this c/b should be from parent
			PUBLISHER_BF.add(tt_pair[0] + tt_pair[1])

			
def unsubscribe_all(): #TODO: very wasteful, it would be nice to leave same subs alone
	global SELF_PUBLISHER_LIST
	for subs in SELF_PUBLISHER_LIST:
		subs[1].unregister()
		del subs


def does_bf_contain_all(publist):							#Bloom filter used to tell if we need to update
	for tt_pair in publist:
		if not (tt_pair[0] + tt_pair[1]) in PUBLISHER_BF:
			return False
	return True