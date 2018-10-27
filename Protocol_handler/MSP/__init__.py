#OpenGS : The Open Srouce Ground Station Project
#MSP v2 Protocol Handler
#Wiki: https://github.com/iNavFlight/inav/wiki/MSP-V2

#
from serial import Serial
#
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
    
    def construct_payload(self,code: int):
        preamble = b'$X<' #MSP v2 Preamble
        flag = b'\x00' #uint8, flag, usage to be defined (set to zero)
        function = code.to_bytes(2, byteorder='little') #uint16 (little endian). 0 - 255 
        payload = bytes()
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
            
    def get_attitude(self):
        self._controller.write(self.construct_payload(108))
        payload = self.read_payload(108)
        if (payload):
            roll = int.from_bytes(payload[0:2], byteorder='little', signed=True) / 10
            pitch = int.from_bytes(payload[2:4], byteorder='little', signed=True) / 10
            yaw = int.from_bytes(payload[4:6], byteorder='little', signed=False)
            return roll,pitch,yaw
        else:
            raise RuntimeError("Controller has not responded.")