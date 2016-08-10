#!/usr/bin/env python
import rospy
import sys
import fcntl, socket, struct
import hashlib
import hmac
import RandomIO

from os 			import urandom
from Crypto.Cipher 	import AES
from binascii 		import unhexlify

SECRET_KEY 				= '\x0b\xf5\xbd\xdam\\\x9f-\x96x*\xab\xdfe\xbc\x82\x89x\xae$Q\xaa\xf8g\xb12:\xec\x80o\xc0@\x83\xd4\xa7\x03g\x96\x00\xb5U\x8a!\x07\xda\xdd\xe0M*\xa3\xfa[\x056V\x9e\xf8b\xad\xefO\x1f\xf0\x1f\xab3\xdc\x11<\xc0\xef3\x94\xff\xc4\xa9\xfeIg\xc7q\xc3\xbdW\x00\xa5I\xd9\xdd\xcb\xd1\xadZ\xdb\xda\xbe\x89\xe2\xc7s25\xffs\xa4\xc9$/\xc6\xa5\xf3\xdc\xab\x9b2\xea~\xc7j\xb7r\x1bE\xc3\xa00\x07\xa2'
#SECRET_KEY 			= '\x83\x00\xb6T~<\xb4J\xe9\xbf\xb9\xb8a\xc9A*\xf7\xb4\x96\xb6\xda\x98\xa3\xd4\xa0\xf2\xb03\xba\xfa\xe5\xc1\xaewI-3\xb8=\x87\xf2\x06\'\x87\xd8\xb9\xfc\x17\x1f\xe11LU7,\x0feH\x99\x1f8\x9c\xec\xca\xfa\x1a\xd5\xe7o{\x9d\xab\x98o\xb5\xd0{\xc8\xf2f\x03\xads\xfdi\xdf7\xaf\xc5\xa7\x9f\xcb$!\x15"m\x91\xabYK\xfezQ\xb0q\xbc\xa0\xd5\x12\x97\xeb\xa2\xce]\x18`\xacR\xee\xe6@\xd2\xdb\xe4\xaa\x04\xed'
#SECRET_KEY 			= '\x1cY\xfd$#\xd0\xe1qvU\x0b\tZ\xcfN\xcd\x8a\x8fx\xa2\\I\xb6\x83\tE\x00Rc\xb5\x8e\x18]V\xb2\xf2\x85\xfb\x8fthdH\x1d)\xeb\xff\x86\xbc\x1c\xe3G\xdf\x9e\x1a\t\x1e\xbb\x07\xa8s\xb7\xe6(w\xc8\x89\xee\xf1\xe8\xaf\x98\x83,`/u\xb3\x8f\xec\xa8\xdfN\xf4\x178\xff\xa4p\x86\xbd\xa39\xf1\x83}(\x07\x14\xd8M\xbc!6\xfbG\xdd\x1b\xddP\x8c\xaa\x10\x9aT\x8ds\xe1\xa5\x88/e\xdb\x8e\xae\x00\xf1'

INTERFACE				= 'eth0'
#INTERFACE				= 'eth2' #Used for ZED3 

def get_self_MAC(hwr_intrf):
	s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	info = fcntl.ioctl(s.fileno(), 0x8927,  struct.pack('256s', hwr_intrf[:15]))
	return info[18:24]

MAC 					= unhexlify(get_self_MAC(INTERFACE)[0:6].encode('hex'))
print ("MAC: " + (":".join("{:02x}".format(ord(c)) for c in MAC)))
print ("INTERFACE: " + INTERFACE)

def dec_msg(challenge, msg):
	key = get_response(SECRET_KEY,challenge)
	return BM_decrypt(msg, key)


def enc_msg(response, msg):
	key = response
	return BM_encrypt(msg, key)


def verify_msg(challenge, signature_rcvd, msg):
	signature = hmac.new(get_response(SECRET_KEY,challenge),msg).digest()
	return signature == signature_rcvd


def self_signature(challenge, msg):
	return compute_hmac(get_response(SECRET_KEY,challenge),msg)


def compute_hmac(data, key = SECRET_KEY):
	return hmac.new(key,data).digest()


def new_CR():
	C 			= urandom(32)
	R 			= get_response(SECRET_KEY, C)
	return  C + R


def enc_CR(dec_CR, key = SECRET_KEY):

	if len(key) > 32:
		key = key[0:32]

	CR 			= []
	for i in range(len(dec_CR)):
		CR.append(BM_encrypt(dec_CR[i], key))
	return CR


def dec_CR(enc_CR, key = SECRET_KEY):

	if len(key) > 32:
		key = key[0:32]

	CR 			= []
	for i in range(len(enc_CR)):
		CR.append(BM_decrypt(enc_CR[i], key))
	return CR


def BM_encrypt(msg, key):
	iv 		= urandom(16)

	AES_cipher = AES.new(key, AES.MODE_CFB, iv)
	return iv + AES_cipher.encrypt(msg)


def BM_decrypt(msg, key):
	iv 		= msg[0:16]

	AES_cipher = AES.new(key, AES.MODE_CFB, iv)
	return AES_cipher.decrypt(msg[16:])


def get_response(key, challenge):
	return RandomIO.RandomIO(key+challenge).read(len(challenge))


# def long_to_bytes(long_num, len_bytes):
# 	chars 		= []
# 	tot_bits 	= len_bytes * 8
# 	bits 	= bin(long_num)[2:]
# 	while len(bits) < tot_bits:
# 		bits = '0' + bits
# 	for b in range(len(bits) / 8):
# 		byte = bits[b*8:(b+1)*8]
# 		chars.append(chr(int(''.join([str(bit) for bit in byte]), 2)))
# 	return ''.join(chars)