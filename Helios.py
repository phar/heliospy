

import usb.core
import usb.util
import struct
import time
import queue
from threading import Thread

HELIOS_VID	= 0x1209
HELIOS_PID	= 0xE500
EP_BULK_OUT	= 0x02
EP_BULK_IN	= 0x81
EP_INT_OUT	= 0x06
EP_INT_IN	= 0x83

INTERFACE_INT =  0
INTERFACE_BULK = 1
INTERFACE_ISO =  2

HELIOS_MAX_POINTS	= 0x1000
HELIOS_MAX_RATE		= 0xFFFF
HELIOS_MIN_RATE		= 7

HELIOS_SUCCESS		= 1

# Functions return negative values if something went wrong
# Attempted to perform an action before calling OpenDevices()
HELIOS_ERROR_NOT_INITIALIZED	=-1
# Attempted to perform an action with an invalid device number
HELIOS_ERROR_INVALID_DEVNUM		= -2
# WriteFrame() called with null pointer to points
HELIOS_ERROR_NULL_POINTS		= -3
# WriteFrame() called with a frame containing too many points
HELIOS_ERROR_TOO_MANY_POINTS	= -4
# WriteFrame() called with pps higher than maximum allowed
HELIOS_ERROR_PPS_TOO_HIGH		= -5
# WriteFrame() called with pps lower than minimum allowed
HELIOS_ERROR_PPS_TOO_LOW		= -6

# Errors from the HeliosDacDevice class begin at -1000
# Attempted to perform an operation on a closed DAC device
HELIOS_ERROR_DEVICE_CLOSED			= -1000
# Attempted to send a new frame with HELIOS_FLAGS_DONT_BLOCK before previous DoFrame() completed
HELIOS_ERROR_DEVICE_FRAME_READY		= -1001
#/ Operation failed because SendControl() failed (if operation failed because of libusb_interrupt_transfer failure, the error code will be a libusb error instead)
HELIOS_ERROR_DEVICE_SEND_CONTROL	= -1002
# Received an unexpected result from a call to SendControl()
HELIOS_ERROR_DEVICE_RESULT			= -1003
# Attempted to call SendControl() with a null buffer pointer
HELIOS_ERROR_DEVICE_NULL_BUFFER		= -1004
# Attempted to call SendControl() with a control signal that is too long
HELIOS_ERROR_DEVICE_SIGNAL_TOO_LONG	= -1005

HELIOS_ERROR_LIBUSB_BASE		= -5000
	
HELIOS_FLAGS_DEFAULT			= 0
HELIOS_FLAGS_START_IMMEDIATELY	= (1 << 0)
HELIOS_FLAGS_SINGLE_MODE		= (1 << 1)
HELIOS_FLAGS_DONT_BLOCK			= (1 << 2)


HELIOS_CMD_STOP					=0x0001
HELIOS_CMD_SHUTTER				=0x0002
HELIOS_CMD_GET_STATUS			=0x0003
HELIOS_GET_FWVERSION			=0x0004
HELIOS_CMD_GET_NAME				=0x0005
HELIOS_CMD_SET_NAME				=0x0006
HELIOS_SET_SDK_VERSION			=0x0007
HELIOS_CMD_ERASE_FIRMWARE		=0x00de

HELIOS_SDK_VERSION	=	6

class HeliosPoint():
	def __init__(self,x,y,c = 0xff0000,i= 255):
		self.x = x
		self.y = y
		self.c = 0x010203
		self.i = i


class HeliosDAC():
	def __init__(self,queuethread=True, debug=0):
		self.debug=debug
		self.closed = 1
		self.frameReady = 0
		self.framebuffer = ""
		self.threadqueue = queue.Queue(maxsize=20)
		self.nextframebuffer = ""
		self.dev = usb.core.find(idVendor=HELIOS_VID, idProduct=HELIOS_PID)
		self.cfg = self.dev.get_active_configuration()
		self.intf = self.cfg[(0,1,2)]
		self.dev.reset()
		
		self.dev.set_interface_altsetting(interface = 0, alternate_setting = 1)
		
		if self.dev.is_kernel_driver_active(0) is True:
			self.dev.detach_kernel_driver(0)
		# claim the device
		usb.util.claim_interface(self.dev, 0)
			
		if self.dev is None:
			raise ValueError('Device not found')
		else:
			if self.debug:
				print(self.dev)
		
		try:
			transferResult = self.intf[0].read(32,1)
		except:
			if self.debug:
				print("no lingering data")

		if self.debug:
			print(self.GetName())
			print(self.getHWVersion())
		self.setSDKVersion()
		self.closed = False
		
		if queuethread:
			self.runQueueThread()

	def runQueueThread(self):
		worker = Thread(target=self.doframe_thread_loop)
		worker.setDaemon(True)
		worker.start()
			
	def doframe_thread_loop(self):
		while self.closed == 0:
			if self.closed:
				return;
			self.nextframebuffer = self.threadqueue.get(block=True)
			self.DoFrame();

	def getHWVersion(self):
		self.intf[1].write(struct.pack("<H",HELIOS_GET_FWVERSION))
		transferResult = self.intf[0].read(32)
		if transferResult[0] == 0x84:
			return struct.unpack("<L",transferResult[1:])[0]
		else:
			return None
		
	def setSDKVersion(self, version = HELIOS_SDK_VERSION):
		self.intf[1].write(struct.pack("<H",(version << 8) | HELIOS_SET_SDK_VERSION))
		return
		
	def setShutter(self, shutter=False):
		self.SendControl(struct.pack("<H",(shutter << 8) | HELIOS_CMD_SHUTTER))
		return
		
	def setName(self, name):
		self.SendControl(struct.pack("<H", HELIOS_CMD_SET_NAME) + name[:30] + b"\x00")
		return

	def newFrame(self,pps, pntobjlist, flags = HELIOS_FLAGS_DEFAULT):
		if self.closed:
			return HELIOS_ERROR_DEVICE_CLOSED;

		if ( len(pntobjlist) > HELIOS_MAX_POINTS):
			return HELIOS_ERROR_TOO_MANY_POINTS

		if (pps > HELIOS_MAX_RATE):
			return HELIOS_ERROR_PPS_TOO_HIGH

		if (pps < HELIOS_MIN_RATE):
			return HELIOS_ERROR_PPS_TOO_LOW
		
		#this is a bug workaround, the mcu won't correctly receive transfers with these sizes
		ppsActual = pps;
		numOfPointsActual = len(pntobjlist)
		if (((len(pntobjlist)-45) % 64) == 0):
			numOfPointsActual-=1
			ppsActual = (pps * numOfPointsActual / len(pntobjlist) + 0.5)

		pntobjlist = pntobjlist[:numOfPointsActual]
		self.nextframebuffer = b""
		for pnt in pntobjlist:
			a = (pnt.x >> 4) & 0xff
			b = ((pnt.x & 0x0F) << 4) | (pnt.y >> 8)
			c = pnt.y & 0xFF
			r = (pnt.c & 0xff0000) >> 16
			g = (pnt.c & 0xff00) >> 8
			b = (pnt.c & 0xff)
			self.nextframebuffer += struct.pack("BBBBBBB", a,b,c,r,g,b,pnt.i)
		self.nextframebuffer += struct.pack("BBBBB",  (ppsActual & 0xFF),(ppsActual >> 8) ,(len(pntobjlist) & 0xFF),(len(pntobjlist) >> 8),flags)
		self.threadqueue.put(self.nextframebuffer)

	def DoFrame(self):
		if (self.closed):
			return HELIOS_ERROR_DEVICE_CLOSED;

		self.intf[3].write(self.nextframebuffer)
		return self.getStatus()

	def GetName(self):
		self.SendControl(struct.pack("<H",HELIOS_CMD_GET_NAME))
		x = self.intf[0].read(32)[:16]
		if x[0] == 0x85:
			return "".join([chr(t) for t in x[1:]])
		else:
			return None
				
	def SendControl(self, buffer):
		if (buffer == None):
			return HELIOS_ERROR_DEVICE_NULL_BUFFER;
		if (len(buffer) > 32):
			return HELIOS_ERROR_DEVICE_SIGNAL_TOO_LONG;
		self.intf[1].write(buffer)

	def stop(self):
		self.SendControl(struct.pack("<H",0x0001), 2)
		time.sleep(.1)
		return
	
	def getStatus(self):
		self.SendControl(struct.pack("<H",0x0003))
		ret = self.intf[0].read(32)
		if self.debug:
			print(ret)
		return ret


if __name__ == "__main__":
	a = HeliosDAC()
	while(1):
		a.newFrame(20000,[HeliosPoint(300,300),
						HeliosPoint(300,305),
						HeliosPoint(305,305),
						HeliosPoint(305,300),
						HeliosPoint(300,300),
						HeliosPoint(300,305),
						HeliosPoint(5,5),
						HeliosPoint(5,0)])

