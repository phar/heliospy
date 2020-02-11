

import usb.core
import usb.util
import struct
import time


HELIOS_VID	=0x1209
HELIOS_PID	=0xE500
EP_BULK_OUT	=0x02
EP_BULK_IN	=0x81
EP_INT_OUT	=0x06
EP_INT_IN	=0x83

INTERFACE_INT = 0
INTERFACE_BULK = 1
INTERFACE_ISO = 2

HELIOS_MAX_POINTS	=0x1000
HELIOS_MAX_RATE		=0xFFFF
HELIOS_MIN_RATE		=7

HELIOS_SUCCESS		=1

# Functions return negative values if something went wrong
# Attempted to perform an action before calling OpenDevices()
HELIOS_ERROR_NOT_INITIALIZED	=-1
# Attempted to perform an action with an invalid device number
HELIOS_ERROR_INVALID_DEVNUM		=-2
# WriteFrame() called with null pointer to points
HELIOS_ERROR_NULL_POINTS		=-3
# WriteFrame() called with a frame containing too many points
HELIOS_ERROR_TOO_MANY_POINTS	=-4
# WriteFrame() called with pps higher than maximum allowed
HELIOS_ERROR_PPS_TOO_HIGH		=-5
# WriteFrame() called with pps lower than minimum allowed
HELIOS_ERROR_PPS_TOO_LOW		=-6

# Errors from the HeliosDacDevice class begin at -1000
# Attempted to perform an operation on a closed DAC device
HELIOS_ERROR_DEVICE_CLOSED			=-1000
# Attempted to send a new frame with HELIOS_FLAGS_DONT_BLOCK before previous DoFrame() completed
HELIOS_ERROR_DEVICE_FRAME_READY		=-1001
#/ Operation failed because SendControl() failed (if operation failed because of libusb_interrupt_transfer failure, the error code will be a libusb error instead)
HELIOS_ERROR_DEVICE_SEND_CONTROL	=-1002
# Received an unexpected result from a call to SendControl()
HELIOS_ERROR_DEVICE_RESULT			=-1003
# Attempted to call SendControl() with a null buffer pointer
HELIOS_ERROR_DEVICE_NULL_BUFFER		=-1004
# Attempted to call SendControl() with a control signal that is too long
HELIOS_ERROR_DEVICE_SIGNAL_TOO_LONG	=-1005

HELIOS_ERROR_LIBUSB_BASE		=-5000
	
HELIOS_FLAGS_DEFAULT			=0
HELIOS_FLAGS_START_IMMEDIATELY	=(1 << 0)
HELIOS_FLAGS_SINGLE_MODE		=(1 << 1)
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
		self.c = c
		self.i = i


class HeliosDAC():
	def __init__(self):
		self.closed = 1
		self.frameReady = 0
		self.framebuffer = ""
		self.nextframebuffer = ""
		#1209:e500
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
#			print(self.dev)
			pass
		
		print(self.GetName())
		print(self.getHWVersion())
		self.setSDKVersion()
#		self.newFrame()
		self.closed = 0
		self.newFrame(3,[HeliosPoint(-5,5),HeliosPoint(5,5),HeliosPoint(5,-5),HeliosPoint(-5,-5)],HELIOS_FLAGS_START_IMMEDIATELY)
		self.doframe_thread_loop()
			
	def doframe_thread_loop(self):
		while self.closed == 0:
			while ((not self.frameReady) and (not self.closed)):
				time.sleep(.100)

			if self.closed:
				return;

			self.DoFrame();

			self.frameReady = false;

	def getHWVersion(self):
		self.intf[1].write(struct.pack("<H",HELIOS_GET_FWVERSION))
#		transferResult = self.intf[1].write()
		transferResult = self.intf[0].read(32)
		if transferResult[0] == 0x84:
			return struct.unpack("<L",transferResult[1:])[0]
		else:
			return None
		
	def setSDKVersion(self, version = HELIOS_SET_SDK_VERSION):
		self.intf[1].write(struct.pack("<H",(version << 8) | HELIOS_SET_SDK_VERSION))
		transferResult = self.intf[1].read(32)
		print(transferResult)
		return
		
	def newFrame(self,pps, pntobjlist, flags = HELIOS_FLAGS_DEFAULT):
		#this is a bug workaround, the mcu won't correctly receive transfers with these sizes
		if self.closed:
			return HELIOS_ERROR_DEVICE_CLOSED;

		if self.frameReady:
			return HELIOS_ERROR_DEVICE_FRAME_READY;

		ppsActual = pps;
		numOfPointsActual = len(pntobjlist)
		if (((len(pntobjlist)-45) % 64) == 0):
			numOfPointsActual-=1
			#adjust pps to keep the same frame duration even with one less point
			ppsActual = (pps * numOfPointsActual / len(pntobjlist) + 0.5)

		pntobjlst = pntobjlist[:numOfPointsActual]
		self.nextframebuffer = b""
		for pnt in pntobjlist:
			a = (pnt.x >> 4) & 0xff
			b = ((pnt.x & 0x0F) << 4) | (pnt.y >> 8)
			c = pnt.y & 0xFF
			r = (pnt.c & 0xff0000) >> 16
			g = (pnt.c & 0xff00) >> 8
			b = (pnt.c & 0xff)
			print(a,b,c,r,g,b,pnt.i)
			self.nextframebuffer += struct.pack("BBBBBBB", a,b,c,r,g,b,pnt.i)
			
		self.nextframebuffer += struct.pack("BBBBB",  (ppsActual & 0xFF),(ppsActual >> 8) ,(len(pntobjlist) & 0xFF),(len(pntobjlist) >> 8),flags)
		if ((flags & HELIOS_FLAGS_DONT_BLOCK) != 0):
			self.frameReady = true;
			return HELIOS_SUCCESS
		else:
			return self.DoFrame()
		
	def DoFrame(self):
		if (self.closed):
			return HELIOS_ERROR_DEVICE_CLOSED;

		self.intf[3].write(self.nextframebuffer)
		ctrlBuffer5 = self.intf[3].read(32)#[:8 + (len(self.nextframebuffer) >> 5)]

	def GetName(self):
		x = self.SendControl(struct.pack("<H",HELIOS_CMD_GET_NAME))
		if x[0] == "\x85":
			return x[1:]
		else:
			return None
				
	def SendControl(self, buffer):
		if (buffer == None):
			return HELIOS_ERROR_DEVICE_NULL_BUFFER;

		if (len(buffer) > 32):
			return HELIOS_ERROR_DEVICE_SIGNAL_TOO_LONG;

		self.intf[1].write(buffer)
		transferResult = self.intf[0].read(32)[:16]
		transferResult  = "".join([chr(x) for x in transferResult])
		return transferResult;

	def stop(self):
		self.SendControl(struct.pack("<H",0x0001), 2)
		time.sleep(100)
		return
	
	def getStatus(self):
		ret = self.SendControl(struct.pack("<H",0x0003))
		if ret[0] == "\x85":
			if ret[1:7] == "Helios":
				self.intf[1].write(ctrlBuffer4,2)
				ctrlBuffer5 = self.intf[0].read(32)[:16]
				ctrlBuffer5  = "".join([chr(x) for x in ctrlBuffer5])
				return ctrlBuffer5
		else:
			return None




a = HeliosDAC()

# find our device

