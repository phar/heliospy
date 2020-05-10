

import usb.core
import usb.util
import struct
import time
import queue
from hershey import *
from threading import Thread
import matplotlib.pyplot as plt
import numpy as np


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
	def __init__(self,x,y,c = 0xff0000,i= 255,blank=False):
		self.x = x
		self.y = y
		self.c = 0x010203
		self.i = i
		self.blank = blank
		
	def __str__(self):
		return "HeleiosPoint(%d, %d,0x%0x,%d,%d)" % (self.x, self.y, self.c,self.i, self.blank)

class HeliosDAC():
	def __init__(self,queuethread=True, debug=0):
		self.debug=debug
		self.closed = 1
		self.frameReady = 0
		self.framebuffer = ""
		self.threadqueue = queue.Queue(maxsize=20)
		self.nextframebuffer = ""
		self.adcbits = 12
		self.dev = usb.core.find(idVendor=HELIOS_VID, idProduct=HELIOS_PID)
		self.cfg = self.dev.get_active_configuration()
		self.intf = self.cfg[(0,1,2)]
		self.dev.reset()
		self.palette =    [(   0,   0,   0 ),	# Black/blanked (fixed)
		 ( 255, 255, 255 ),	# White (fixed)
		 ( 255,   0,   0 ),  # Red (fixed)
		 ( 255, 255,   0 ),  # Yellow (fixed)
		 (   0, 255,   0 ),  # Green (fixed)
		 (   0, 255, 255 ),  # Cyan (fixed)
		 (   0,   0, 255 ),  # Blue (fixed)
		 ( 255,   0, 255 ),  # Magenta (fixed)
		 ( 255, 128, 128 ),  # Light red
		 ( 255, 140, 128 ),
		 ( 255, 151, 128 ),
		 ( 255, 163, 128 ),
		 ( 255, 174, 128 ),
		 ( 255, 186, 128 ),
		 ( 255, 197, 128 ),
		 ( 255, 209, 128 ),
		 ( 255, 220, 128 ),
		 ( 255, 232, 128 ),
		 ( 255, 243, 128 ),
		 ( 255, 255, 128 ),	# Light yellow
		 ( 243, 255, 128 ),
		 ( 232, 255, 128 ),
		 ( 220, 255, 128 ),
		 ( 209, 255, 128 ),
		 ( 197, 255, 128 ),
		 ( 186, 255, 128 ),
		 ( 174, 255, 128 ),
		 ( 163, 255, 128 ),
		 ( 151, 255, 128 ),
		 ( 140, 255, 128 ),
		 ( 128, 255, 128 ),	# Light green
		 ( 128, 255, 140 ),
		 ( 128, 255, 151 ),
		 ( 128, 255, 163 ),
		 ( 128, 255, 174 ),
		 ( 128, 255, 186 ),
		 ( 128, 255, 197 ),
		 ( 128, 255, 209 ),
		 ( 128, 255, 220 ),
		 ( 128, 255, 232 ),
		 ( 128, 255, 243 ),
		 ( 128, 255, 255 ),	# Light cyan
		 ( 128, 243, 255 ),
		 ( 128, 232, 255 ),
		 ( 128, 220, 255 ),
		 ( 128, 209, 255 ),
		 ( 128, 197, 255 ),
		 ( 128, 186, 255 ),
		 ( 128, 174, 255 ),
		 ( 128, 163, 255 ),
		 ( 128, 151, 255 ),
		 ( 128, 140, 255 ),
		 ( 128, 128, 255 ),	# Light blue
		 ( 140, 128, 255 ),
		 ( 151, 128, 255 ),
		 ( 163, 128, 255 ),
		 ( 174, 128, 255 ),
		 ( 186, 128, 255 ),
		 ( 197, 128, 255 ),
		 ( 209, 128, 255 ),
		 ( 220, 128, 255 ),
		 ( 232, 128, 255 ),
		 ( 243, 128, 255 ),
		 ( 255, 128, 255 ), # Light magenta
		 ( 255, 128, 243 ),
		 ( 255, 128, 232 ),
		 ( 255, 128, 220 ),
		 ( 255, 128, 209 ),
		 ( 255, 128, 197 ),
		 ( 255, 128, 186 ),
		 ( 255, 128, 174 ),
		 ( 255, 128, 163 ),
		 ( 255, 128, 151 ),
		 ( 255, 128, 140 ),
		 ( 255,   0,   0 ),	# Red (cycleable)
		 ( 255,  23,   0 ),
		 ( 255,  46,   0 ),
		 ( 255,  70,   0 ),
		 ( 255,  93,   0 ),
		 ( 255, 116,   0 ),
		 ( 255, 139,   0 ),
		 ( 255, 162,   0 ),
		 ( 255, 185,   0 ),
		 ( 255, 209,   0 ),
		 ( 255, 232,   0 ),
		 ( 255, 255,   0 ),	#Yellow (cycleable)
		 ( 232, 255,   0 ),
		 ( 209, 255,   0 ),
		 ( 185, 255,   0 ),
		 ( 162, 255,   0 ),
		 ( 139, 255,   0 ),
		 ( 116, 255,   0 ),
		 (  93, 255,   0 ),
		 (  70, 255,   0 ),
		 (  46, 255,   0 ),
		 (  23, 255,   0 ),
		 (   0, 255,   0 ),	# Green (cycleable)
		 (   0, 255,  23 ),
		 (   0, 255,  46 ),
		 (   0, 255,  70 ),
		 (   0, 255,  93 ),
		 (   0, 255, 116 ),
		 (   0, 255, 139 ),
		 (   0, 255, 162 ),
		 (   0, 255, 185 ),
		 (   0, 255, 209 ),
		 (   0, 255, 232 ),
		 (   0, 255, 255 ),	# Cyan (cycleable)
		 (   0, 232, 255 ),
		 (   0, 209, 255 ),
		 (   0, 185, 255 ),
		 (   0, 162, 255 ),
		 (   0, 139, 255 ),
		 (   0, 116, 255 ),
		 (   0,  93, 255 ),
		 (   0,  70, 255 ),
		 (   0,  46, 255 ),
		 (   0,  23, 255 ),
		 (   0,   0, 255 ),	# Blue (cycleable)
		 (  23,   0, 255 ),
		 (  46,   0, 255 ),
		 (  70,   0, 255 ),
		 (  93,   0, 255 ),
		 ( 116,   0, 255 ),
		 ( 139,   0, 255 ),
		 ( 162,   0, 255 ),
		 ( 185,   0, 255 ),
		 ( 209,   0, 255 ),
		 ( 232,   0, 255 ),
		 ( 255,   0, 255 ),	# Magenta (cycleable)
		 ( 255,   0, 232 ),
		 ( 255,   0, 209 ),
		 ( 255,   0, 185 ),
		 ( 255,   0, 162 ),
		 ( 255,   0, 139 ),
		 ( 255,   0, 116 ),
		 ( 255,   0,  93 ),
		 ( 255,   0,  70 ),
		 ( 255,   0,  46 ),
		 ( 255,   0,  23 ),
		 ( 128,   0,   0 ),	# Dark red
		 ( 128,  12,   0 ),
		 ( 128,  23,   0 ),
		 ( 128,  35,   0 ),
		 ( 128,  47,   0 ),
		 ( 128,  58,   0 ),
		 ( 128,  70,   0 ),
		 ( 128,  81,   0 ),
		 ( 128,  93,   0 ),
		 ( 128, 105,   0 ),
		 ( 128, 116,   0 ),
		 ( 128, 128,   0 ),	# Dark yellow
		 ( 116, 128,   0 ),
		 ( 105, 128,   0 ),
		 (  93, 128,   0 ),
		 (  81, 128,   0 ),
		 (  70, 128,   0 ),
		 (  58, 128,   0 ),
		 (  47, 128,   0 ),
		 (  35, 128,   0 ),
		 (  23, 128,   0 ),
		 (  12, 128,   0 ),
		 (   0, 128,   0 ),	# Dark green
		 (   0, 128,  12 ),
		 (   0, 128,  23 ),
		 (   0, 128,  35 ),
		 (   0, 128,  47 ),
		 (   0, 128,  58 ),
		 (   0, 128,  70 ),
		 (   0, 128,  81 ),
		 (   0, 128,  93 ),
		 (   0, 128, 105 ),
		 (   0, 128, 116 ),
		 (   0, 128, 128 ),	# Dark cyan
		 (   0, 116, 128 ),
		 (   0, 105, 128 ),
		 (   0,  93, 128 ),
		 (   0,  81, 128 ),
		 (   0,  70, 128 ),
		 (   0,  58, 128 ),
		 (   0,  47, 128 ),
		 (   0,  35, 128 ),
		 (   0,  23, 128 ),
		 (   0,  12, 128 ),
		 (   0,   0, 128 ),	# Dark blue
		 (  12,   0, 128 ),
		 (  23,   0, 128 ),
		 (  35,   0, 128 ),
		 (  47,   0, 128 ),
		 (  58,   0, 128 ),
		 (  70,   0, 128 ),
		 (  81,   0, 128 ),
		 (  93,   0, 128 ),
		 ( 105,   0, 128 ),
		 ( 116,   0, 128 ),
		 ( 128,   0, 128 ),	# Dark magenta
		 ( 128,   0, 116 ),
		 ( 128,   0, 105 ),
		 ( 128,   0,  93 ),
		 ( 128,   0,  81 ),
		 ( 128,   0,  70 ),
		 ( 128,   0,  58 ),
		 ( 128,   0,  47 ),
		 ( 128,   0,  35 ),
		 ( 128,   0,  23 ),
		 ( 128,   0,  12 ),
		 ( 255, 192, 192 ),	# Very light red
		 ( 255,  64,  64 ),	# Light-medium red
		 ( 192,   0,   0 ),	# Medium-dark red
		 (  64,   0,   0 ),	# Very dark red
		 ( 255, 255, 192 ),	# Very light yellow
		 ( 255, 255,  64 ),	# Light-medium yellow
		 ( 192, 192,   0 ),	# Medium-dark yellow
		 (  64,  64,   0 ),	# Very dark yellow
		 ( 192, 255, 192 ),	# Very light green
		 (  64, 255,  64 ),	# Light-medium green
		 (   0, 192,   0 ),	# Medium-dark green
		 (   0,  64,   0 ),	# Very dark green
		 ( 192, 255, 255 ),	# Very light cyan
		 (  64, 255, 255 ),	# Light-medium cyan
		 (   0, 192, 192 ),	# Medium-dark cyan
		 (   0,  64,  64 ),	# Very dark cyan
		 ( 192, 192, 255 ),	# Very light blue
		 (  64,  64, 255 ),	# Light-medium blue
		 (   0,   0, 192 ),	# Medium-dark blue
		 (   0,   0,  64 ),	# Very dark blue
		 ( 255, 192, 255 ),	# Very light magenta
		 ( 255,  64, 255 ),	# Light-medium magenta
		 ( 192,   0, 192 ),	# Medium-dark magenta
		 (  64,   0,  64 ),	# Very dark magenta
		 ( 255,  96,  96 ),	# Medium skin tone
		 ( 255, 255, 255 ),	# White (cycleable)
		 ( 245, 245, 245 ),
		 ( 235, 235, 235 ),
		 ( 224, 224, 224 ),	# Very light gray (7/8 intensity)
		 ( 213, 213, 213 ),
		 ( 203, 203, 203 ),
		 ( 192, 192, 192 ),	# Light gray (3/4 intensity)
		 ( 181, 181, 181 ),
		 ( 171, 171, 171 ),
		 ( 160, 160, 160 ),	# Medium-light gray (5/8 int.)
		 ( 149, 149, 149 ),
		 ( 139, 139, 139 ),
		 ( 128, 128, 128 ),	# Medium gray (1/2 intensity)
		 ( 117, 117, 117 ),
		 ( 107, 107, 107 ),
		 (  96,  96,  96 ),	# Medium-dark gray (3/8 int.)
		 (  85,  85,  85 ),
		 (  75,  75,  75 ),
		 (  64,  64,  64 ),	# Dark gray (1/4 intensity)
		 (  53,  53,  53 ),
		 (  43,  43,  43 ),
		 (  32,  32,  32 ),	# Very dark gray (1/8 intensity)
		 (  21,  21,  21 ),
		 (  11,  11,  11 )]	# Black
		
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
			ppsActual = int((pps * numOfPointsActual / len(pntobjlist) + 0.5))

		pntobjlist = pntobjlist[:numOfPointsActual]
		nextframebuffer = b""
		for pnt in pntobjlist:
			a = (pnt.x >> 4) & 0xff
			b = ((pnt.x & 0x0F) << 4) | (pnt.y >> 8)
			c = pnt.y & 0xFF
			if pnt.blank == False:
				r = (pnt.c & 0xff0000) >> 16
				g = (pnt.c & 0xff00) >> 8
				b = (pnt.c & 0xff)
				i = pnt.i
			else:
				r = 0
				g = 0
				b = 0
				i = 0
			nextframebuffer += struct.pack("BBBBBBB", a,b,c,r,g,b,i)
		nextframebuffer += struct.pack("BBBBB",  (ppsActual & 0xFF),(ppsActual >> 8) ,(len(pntobjlist) & 0xFF),(len(pntobjlist) >> 8),flags)
		self.threadqueue.put(nextframebuffer)
		
	def DoFrame(self):
		if (self.closed):
			return HELIOS_ERROR_DEVICE_CLOSED;
		self.nextframebuffer = self.threadqueue.get(block=True)
		self.intf[3].write(self.nextframebuffer)
		t = time.time()
		while(self.getStatus()[1] == 0): #wait for the laser
			pass
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
		
	def generateText(self,text,xpos,ypos,cindex=0,scale=1.0):
		pointstream = []
		ctr = 0
		for c in text:
			lastx = xpos
			lasty = ypos
			blank = True
			for x,y in HERSHEY_FONT[ord(c)-32]:
				if (x == -1) and (y == -1):
#					pointstream.append(HeliosPoint(lastx,lasty,blank=blank))
					blank = True
				else:
					lastx = int((x + (ctr * HERSHEY_WIDTH)) * scale)
					lasty = int(y * scale)
					blank = False
					pointstream.append(HeliosPoint(lastx,lasty,self.palette[cindex],blank=blank))
			ctr += 1

		return pointstream

	def loadILDfile(self,filename, xscale=1.0, yscale=1.0):
		f = open(filename,"rb")
		headerstruct = ">4s3xB8s8sHHHBx"
		moreframes = True
		frames = []
		while moreframes:
			(magic, format, fname, cname, rcnt, num, total_frames, projectorid) = struct.unpack(headerstruct,f.read(struct.calcsize(headerstruct)))
			if magic == b"ILDA":
				pointlist = []
				palette = []
				x = y = z = red = green = blue = 0
				blank = 1
				lastpoint = 0
				if rcnt > 0:
					for  i in range(rcnt):
						if format in [0,1,4,5]:
							if format == 0:
								fmt = ">hhhBB"
								(x,y,z,status,cindex) = struct.unpack(fmt,f.read(struct.calcsize(fmt)))

							elif format == 1:
								fmt = ">hhBB"
								(x,y,status,cindex) = struct.unpack(fmt,f.read(struct.calcsize(fmt)))

							elif format == 4:
								(x,y,z,status,red,green,blue) = struct.unpack(fmt,f.read(struct.calcsize(fmt)))

							elif format == 5:
								fmt = ">hhhBBBB"
								(x,y,status,red,green,blue) = struct.unpack(fmt,f.read(struct.calcsize(fmt)))
						
							blank = (status & 0x40) > 0
							lastpoint = (status & 0x80) > 0
							lessadcbits = (16 - self.adcbits)
							x = int((x >> lessadcbits) * xscale)
							y = int((y >> lessadcbits) * yscale)
							pointlist.append(HeliosPoint(x,y,self.palette[cindex],blank=blank))
							
						elif format == 2:
							fmt = ">BBB"
							(r,g,b) = struct.unpack(fmt,f.read(struct.calcsize(fmt)))
							palette.append((r<<16) | (g<<8) | b)
							
					if format == 2:
						frames.append((("palette",fname,cname, num),palette))
					else:
						frames.append((("frame",fname,cname,num),pointlist))
					
				else:
					moreframes = 0
			else:
				moreframes = 0

		return frames
		
	def plot(self, pntlist):
		fig, ax = plt.subplots()  # Create a figure containing a single axes.
		xlst = []
		ylst = []
		for p in pntlist:
			if p.blank == False:
				xlst.append(p.x)
				ylst.append(p.y)
		ax.plot(xlst,ylst)
		plt.show()
		
		
		

if __name__ == "__main__":
	a = HeliosDAC()

	a.runQueueThread()

#	cal = a.generateText("hello World", 20,20,scale=10)
##	print(cal)
#	a.plot(cal)
#
#	while(1):
#		a.newFrame(2000,cal)
#		a.DoFrame()


	cal = a.loadILDfile("astroid.ild")
	while(1):
		for (t,n1,n2,c),f in cal:
			print("playing %s,%s, %d" % (n1,n2,c))
			a.newFrame(5000,f)
#			a.DoFrame()

#			a.plot(f)


#	while(1):
##		a.newFrame(1000,[HeliosPoint(16000,16000)])
#		a.newFrame(100,[HeliosPoint(16000-2500,16000),HeliosPoint(16000,16000),HeliosPoint(16000+2500,16000),HeliosPoint(16000,16000),HeliosPoint(16000,16000+2500),HeliosPoint(16000,16000),HeliosPoint(16000,16000-2500),HeliosPoint(16000,16000)])
#	a.DoFrame()


#	while(1):
#		a.newFrame(1000,[HeliosPoint(0,200),
#						HeliosPoint(200,200),
#						HeliosPoint(200,0),
#						HeliosPoint(0,0),
#						])
#		a.DoFrame()


