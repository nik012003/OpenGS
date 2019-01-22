#OpenGS : The Open Srouce Ground Station Project
#MSP v2 Protocol Handler
#Wiki: https://github.com/iNavFlight/inav/wiki/MSP-V2

#
from serial import Serial
import time
from threading import Event, Thread
#

class RepeatedTimer:
    #Repeat `function` every `interval` seconds made by shlapion (https://github.com/shlapion)
    def __init__(self, interval, function, *args, **kwargs):
        self.interval = interval
        self.function = function
        self.args = args
        self.kwargs = kwargs
        self.start = time.time()
        self.event = Event()
        self.thread = Thread(target=self._target)

    def startTimer(self):
        self.thread.start()

    def _target(self):
        while not self.event.wait(self._time):
            self.function(*self.args, **self.kwargs)

    @property
    def _time(self):
        return self.interval - ((time.time() - self.start) % self.interval)

    def stopTimer(self):
        self.event.set()
        self.thread.join()

class MSP(object):
    def __init__(self, controller: Serial):
        self._controller = controller
        
    def crc8_dvb_s2(self,crc, a):
        crc ^= a 
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0xD5) % 256
            else:
                crc = (crc << 1) % 256
        return crc
    
    def checksum(self,message): 
        crc = 0  #initialize crc
        #calculate crc8_dvb_s2
        for i in range(len(message)):
            crc  = self.crc8_dvb_s2(crc, message[i])
        #-------------
        return crc
    
    def construct_payload(self,code: int,payload = bytes()):
        preamble = b'$X<' #MSP v2 Preamble
        flag = b'\x00' #uint8, flag, usage to be defined (set to zero)
        function = code.to_bytes(2, byteorder='little') #uint16 (little endian). 0 - 255 
        payload_size = len(payload).to_bytes(2, byteorder='little') #unit16
        ck = self.checksum(flag + function + payload_size + payload).to_bytes(1, byteorder='little') #unit8 checksum
        return preamble + flag + function + payload_size + payload + ck
    
    def read_payload(self, func):
        if (self._controller.read(3) == b'$X>'): #find the preamble
            flag = self._controller.read(1) #read the unit8 flag
            function = self._controller.read(2) # read the unit16 function 
            if (int.from_bytes(function, byteorder='little') == func):
                payload_size = self._controller.read(2) #read the unit16 payload size
                payload = self._controller.read(int.from_bytes(payload_size, byteorder='little'))#read the payload using the payload_size to know how many bytes to read
                ck = self._controller.read(1)# read the checksum
                if (ck == self.checksum(flag + function + payload_size + payload).to_bytes(1, byteorder='little')): #compare the checksum
                    return payload
        self._controller.flushInput()
        raise RuntimeError("Controller has not responded.")
    def get_status(self):
        self._controller.flushInput()
        self._controller.write(self.construct_payload(150)) #write the command
        payload = self.read_payload(150) # get the payload
        values = dict()
        values['cpu_load'] = int.from_bytes(payload[0:2], byteorder='little', signed=False) 
        values['arming_flags'] = int.from_bytes(payload[2:4], byteorder='little', signed=False) 
        values['calibration_axis_flags'] = int.from_bytes(payload[4:5], byteorder='little', signed=False) 
        return values
        
    def get_raw_imu(self):
        self._controller.flushInput()
        self._controller.write(self.construct_payload(102)) #write the command
        payload = self.read_payload(102) # get the payload
        values = dict()
        values['accx'] = int.from_bytes(payload[0:2], byteorder='little', signed=True) 
        values['accy'] = int.from_bytes(payload[2:4], byteorder='little', signed=True) 
        values['accz'] = int.from_bytes(payload[4:6], byteorder='little', signed=True) 
        values['gyrx'] = int.from_bytes(payload[6:8], byteorder='little', signed=True) 
        values['gyry'] = int.from_bytes(payload[8:10], byteorder='little', signed=True) 
        values['gyrz'] = int.from_bytes(payload[10:12], byteorder='little', signed=True) 
        values['magx'] = int.from_bytes(payload[12:14], byteorder='little', signed=True) 
        values['magy'] = int.from_bytes(payload[14:16], byteorder='little', signed=True) 
        values['magz'] = int.from_bytes(payload[16:18], byteorder='little', signed=True) 
        return values
    
    def get_attitude(self):
        self._controller.flushInput()
        self._controller.write(self.construct_payload(108)) #write the command
        payload = self.read_payload(108) # get the payload
        values = dict()
        #deconstrunct the payload
        values['roll'] = int.from_bytes(payload[0:2], byteorder='little', signed=True) / 10 #make int from the first 2 bytes 
        values['pitch'] = int.from_bytes(payload[2:4], byteorder='little', signed=True) / 10
        values['yaw'] = int.from_bytes(payload[4:6], byteorder='little', signed=False)
        return values
            
    def get_raw_gps(self):
        self._controller.flushInput()
        self._controller.write(self.construct_payload(106))
        payload = self.read_payload(106)
        values = dict()
        values['fix_type'] = int.from_bytes(payload[0:1], byteorder='little', signed=False)
        values['sats'] = int.from_bytes(payload[1:2], byteorder='little', signed=False)
        values['lat'] = int.from_bytes(payload[2:6], byteorder='little', signed=True) / 10000000
        values['lon'] = int.from_bytes(payload[6:10], byteorder='little', signed=True) / 10000000
        values['alt'] = int.from_bytes(payload[10:12], byteorder='little', signed=True)
        values['groundSpeed'] = int.from_bytes(payload[12:14], byteorder='little', signed=False)
        values['groundCourse'] = int.from_bytes(payload[14:16], byteorder='little', signed=False)
        values['hdop'] = int.from_bytes(payload[16:18], byteorder='little', signed=False) / 100
        return values

    def get_rc(self):
        self._controller.flushInput()
        self._controller.write(self.construct_payload(105))
        payload = self.read_payload(105)
        values = dict()
        types = ( #channel names
        "roll", "pitch", "throttle", "yaw",
        "aux1", "aux2", "aux3", "aux4",
        "aux5", "aux6", "aux7", "aux8",
        "aux9", "aux10", "aux11", "aux12",
        "aux12", "aux13","aux14", "aux15"
        )
        for i in range(len(payload)//2): #each channel is 2 bytes, so we need half of the payload lenght
            x = i * 2
            values[types[i]] = int.from_bytes(payload[x:x+2], byteorder='little', signed=False)#separate all channels
        return values
    
    def get_wp(self, wp_number : int):
        self._controller.flushInput()
        self._controller.write(self.construct_payload(118,wp_number.to_bytes(1, byteorder='little')))
        payload = self.read_payload(118)
        values = dict()
        values['number'] = int.from_bytes(payload[0:1], byteorder='little', signed=False)
        values['action'] = int.from_bytes(payload[1:2], byteorder='little', signed=False)
        values['lat'] = int.from_bytes(payload[2:6], byteorder='little', signed=True) / 10000000
        values['lon'] = int.from_bytes(payload[6:10], byteorder='little', signed=True) / 10000000
        values['alt'] = int.from_bytes(payload[10:14], byteorder='little', signed=True)
        values['p1'] = int.from_bytes(payload[14:16], byteorder='little', signed=False)
        values['p2'] = int.from_bytes(payload[16:18], byteorder='little', signed=False)
        values['p3'] = int.from_bytes(payload[18:20], byteorder='little', signed=False)
        values['flags'] = int.from_bytes(payload[20:21], byteorder='little', signed=False)
        return values
    
    def get_analog(self):
        self._controller.flushInput()
        self._controller.write(self.construct_payload(110))
        payload = self.read_payload(110)
        values = dict()
        values['battery_voltage'] = int.from_bytes(payload[0:1], byteorder='little', signed=False) #vbat
        values['mah_drawn'] = int.from_bytes(payload[1:3], byteorder='little', signed=False) #mah drawn
        values['rssi'] = int.from_bytes(payload[3:5], byteorder='little', signed=False) #rssi
        values['amp'] = int.from_bytes(payload[5:7], byteorder='little', signed=True) #current Amp draw
        return values

    def set_wp(self, wp_number,action,lat,lon,alt,p1,p2,p3,flag : int):
        payload = bytes()
        payload += wp_number.to_bytes(1, byteorder='little')
        payload += action.to_bytes(1, byteorder='little')#action
        payload += int(lat * 10000000).to_bytes(4, byteorder='little')#lat 
        payload += int(lon * 10000000).to_bytes(4, byteorder='little')#lon
        payload += alt.to_bytes(4, byteorder='little')#to set altitude (cm)
        payload += p1.to_bytes(2, byteorder='little')#P1
        payload += p1.to_bytes(2, byteorder='little')#P2
        payload += p1.to_bytes(2, byteorder='little')#P3
        payload += flag.to_bytes(1, byteorder='little')#P3
        self._controller.flushInput()
        self._controller.write(self.construct_payload(209,payload)) # pass payload
        self.read_payload(209)
        
    def set_raw_rc(self, channels: list):
        payload = bytes()
        for channel in channels:
            payload += channel.to_bytes(2, byteorder='little')#add all channels together in one signle 'bytes' variable
        self._controller.flushInput()
        self._controller.write(self.construct_payload(200,payload)) # pass payload
        self.read_payload(200)

    def start_threaded_rc(self,refreshRate): #refreshrate in Hertz. Start sending data to MSP_RX every x seconds.
      global timer, secondsPeriod
      secondsPeriod = 1 / refreshRate #calculate Hz to seconds(period)
      timer = RepeatedTimer(secondsPeriod,self.set_raw_rc,[1500,1500,1000,1500,1000,1000,1000,1000]) #set the timer
      timer.startTimer() #stop the timer
        
    def set_threaded_rc(self,channels: list): #change the data beeing sent
      global timer, secondsPeriod
      timer.stopTimer() #stop the timer
      timer = RepeatedTimer(secondsPeriod,self.set_raw_rc,channels)#set the timer with the new channel values
      timer.startTimer()#restat the timer
    
    def stop_threaded_rc(self): #stops the timer
      global timer
      timer.stopTimer()