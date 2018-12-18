#!/usr/bin/python
import socket
import sys
import time
import os
import logging
import logging.handlers
import sys, traceback
import unicodedata
from datetime import datetime, timedelta
from ConfigParser import SafeConfigParser

###########################
# PERSONAL CONFIG FILE READ
###########################

parser = SafeConfigParser()
parser.read('RS485Logger.ini')

# Read config params
LOG_FILENAME = parser.get('config', 'log_filename')

#SERVER_IP = parser.get('config', 'server_ip')
LISTEN_PORT = parser.getint('config', 'listen_port')

# Get custom names of each RX channel
CHANNELS_LIST = parser.get('config', 'channels')
channel_names = [ chunk.strip() for chunk in CHANNELS_LIST.split(",") ]

REF_DATA_HEX = parser.get('config', 'reference_data_hex')
ExpectedData=[]
if (REF_DATA_HEX!=""):
	ExpectedData = [int(e.strip(),16) for e in parser.get('config', 'reference_data_hex').split(',')]

doNotCheckMessage = False
if (len(ExpectedData)==0):
	doNotCheckMessage = True

#################
#  LOGGING SETUP
#################
#logging.basicConfig()

LOG_LEVEL = logging.INFO  # Could be e.g. "DEBUG" or "WARNING"

# Configure logging to log to a file, making a new file at midnight and keeping the last 3 day's data
# Give the logger a unique name (good practice)
logger = logging.getLogger(__name__)
# Set the log level to LOG_LEVEL
logger.setLevel(LOG_LEVEL)
# Make a handler that writes to a file, making a new file at midnight and keeping 3 backups
#handler = logging.handlers.TimedRotatingFileHandler(LOG_FILENAME, when="midnight", backupCount=3)

# Handler writing to a file, rotating the file every 50MB
handler = logging.handlers.RotatingFileHandler(LOG_FILENAME, maxBytes=25000000, backupCount=999)
# Format each log message like this
#formatter = logging.Formatter('%(asctime)s %(message)s')
formatter = logging.Formatter("%(asctime)s %(message)s", "%d/%m %H:%M:%S")

# Attach the formatter to the handler
handler.setFormatter(formatter)
# Attach the handler to the logger
logger.addHandler(handler)

# Make a class we can use to capture stdout and sterr in the log
class MyLogger(object):
	def __init__(self, logger, level):
		"""Needs a logger and a logger level."""
		self.logger = logger
		self.level = level

	def write(self, message):
		# Only log if there is a message (not just a new line)
		if message.rstrip() != "":
			self.logger.log(self.level, message.rstrip())

# Replace stdout with logging to file at INFO level
sys.stdout = MyLogger(logger, logging.INFO)
# Replace stderr with logging to file at ERROR level
sys.stderr = MyLogger(logger, logging.ERROR)

logger.info('Starting RS485 logger')

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#server_address = (SERVER_IP, LISTEN_PORT)
# Passing '' as the IP address will listen on all local IP addresses, so no need to care which broadcast address to use.
server_address = ('', LISTEN_PORT)
logger.info('starting up on %s port %s' % server_address, )
sock.bind(server_address)

try:
	oldPktIndex = -1
	while True:
		try:
			data, address = sock.recvfrom(1500)

	    		# Format:
	    		#  16 bytes for timestamp
	    		#  4 bytes for ethIndex
	    		#  2 bytes for channel index
	    		#  4 bytes for frameLength
	    		#  N bytes for data

			# Check received packet index: it should increment by one at each packet received,
			# else we have lost some packets...
			#print data[0].encode('hex')
			pktIndex = ord(data[0])
			if (oldPktIndex <> -1):
				if (pktIndex <> ((oldPktIndex+1) % 256)):
					logger.info("Packet LOST (old=%d, new=%d)" % (oldPktIndex, pktIndex))
				else:
					logger.info("Pkt Index=%d" % (pktIndex))

			
			oldPktIndex = pktIndex
			
			# Strip the packet index and proceed	
			data = data[1:]

	    		# A valid data packet has at least (one 26 bytes header + 1 byte of data) of payload
			while (len(data) > 26):
				# Read frame header
				timestamp = data[0:16]
				msgIndex = data[16:20]
				channelIndex = int(data[20:22])
				frameLength = int(data[22:26])
				# Read payload data for this frame
				payload_as_charlist = ''.join(data[26:(26+frameLength)])
				payload= [ord(elem) for elem in payload_as_charlist]
				cmp_list = payload

				if (doNotCheckMessage):
					logger.info( '%s:%s:%s:%s:%s' % (timestamp, msgIndex, channel_names[channelIndex], frameLength, ','.join(x.encode('hex') for x in payload_as_charlist)))
				elif cmp(payload, ExpectedData) == 0:
					logger.info( '%s:%s:%s:%s:OK' % (timestamp,msgIndex,channel_names[channelIndex], frameLength))
				else:
					logger.info( '%s:%s:%s:%s:!!!!!!ERROR!!!!!!:%s' % (timestamp, msgIndex, channel_names[channelIndex],  frameLength, ','.join(x.encode('hex') for x in payload_as_charlist)))

				# Strip the data packet from the frame we just processed, and carry on.
				data = data[26+frameLength:]

		except KeyboardInterrupt:
			raise
		except:
			logger.info("*****Exception in main loop******")
			exc_type, exc_value, exc_traceback = sys.exc_info()
			traceback.print_exception(exc_type, exc_value, exc_traceback,limit=2, file=sys.stdout)	
			del exc_traceback
			logger.info('LOOPING after exception in 10 seconds')
			time.sleep(10)
			pass

except KeyboardInterrupt:
	logger.info("manually interrupted")
except NameError as n:
	print("[ERROR] NameError %s" % n)
except:
	exc_type, exc_value, exc_traceback = sys.exc_info()
	traceback.print_exception(exc_type, exc_value, exc_traceback,limit=2, file=sys.stdout)	
	del exc_traceback
