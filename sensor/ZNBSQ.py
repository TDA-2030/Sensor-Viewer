from serial import Serial, iterbytes
from serial.threaded import ReaderThread, Protocol,Packetizer
import time
from queue import Queue, Empty
from threading import Lock

def _xor(data:bytearray)->int:
    check = 0
    for d in data:
        check = check ^ d
    return check

class FramedPacket(Protocol):
    """
    Read binary packets. Packets are expected to have a start and stop marker.

    The class also keeps track of the transport.
    """

    def __init__(self):
        self.packet = bytearray()
        self.in_packet = False
        self.transport = None
        self.recv_queue:Queue = None

    def connection_made(self, transport):
        """Store transport"""
        self.transport = transport
        print("Connected, ready to receive data...")

    def connection_lost(self, exc):
        """Forget transport"""
        print("COM disconnected")
        self.transport = None
        self.in_packet = False
        del self.packet[:]
        super(FramedPacket, self).connection_lost(exc)

    def data_received(self, data):
        """Find data enclosed in START/STOP, call handle_packet"""
        # print(f"r raw:{data}")
        
        for byte in iterbytes(data):
            if not self.in_packet and byte == b'\xbb':
                self.packet.extend(byte)
                if self.packet[:3]==b'\xbb\xbb\xbb':
                    self.in_packet = True
                else:
                    continue
            
            elif self.in_packet:
                self.packet.extend(byte)
                if len(self.packet)==10:
                    chk = _xor(self.packet[:-1])
                    if self.packet[-1] == chk:
                        # print("========", self.packet)
                        self.in_packet = False
                        self.handle_packet(bytes(self.packet)) # make read-only copy
                        del self.packet[:]
                    
            else:
                self.handle_out_of_packet_data(byte)
            

    def handle_packet(self, packet):
        """Process packets - to be overridden by subclassing"""
        packet = bytes(packet)
        self.recv_queue.put(packet)

    def handle_out_of_packet_data(self, data):
        """Process data that is received outside of packets"""
        pass

class COM:

    def __init__(self, port, baud):
        self.port = port
        self.baud = int(baud)
        self.open_com = None
        self.recv_queue = Queue(10)
        
    def open(self):
        # Initiate serial port
        self.serial_port = Serial(self.port, self.baud)
        # Initiate ReaderThread
        self.reader = ReaderThread(self.serial_port, FramedPacket)
        # Start reader
        self.reader.start()
        self.recv_queue.queue.clear()
        self.reader.protocol.recv_queue = self.recv_queue

    def close(self):
        if hasattr(self, 'reader'):
            self.reader.close()

    def send_cmd(self, addr: int, cmd:bytearray, data: bytearray, wait_sec: float = 1.):
        data = b'\xAA\xAA\xAA' + addr.to_bytes(1, byteorder='big') + cmd + data
        
        data += _xor(data).to_bytes(1, byteorder='big')
        # print("send:", data, len(data))
        self.serial_port.write(data)
        if wait_sec > 0:
            try:
                data = self.reader.protocol.recv_queue.get(block=True, timeout=wait_sec)
                # print(f"recv:{data}")
                return data
            except Empty:
                print("command no ack")


class ForceSensor():

    def __init__(self, com:COM, addr:int):
        self.com = com
        self.addr = addr
        self.mean = 0
        self.com.open()
        self.lock = Lock()

    def set_buad(self, buad):
        buads = { 2400 : 1,  4800 : 2,  9600 : 3,  19200 : 4,  38400 : 5}
        res = self.com.send_cmd(self.addr, b'\xA2', buads[buad].to_bytes(2, 'big'))

    def get_raw_force(self)->int:
        self.lock.acquire()
        res = self.com.send_cmd(self.addr, b'\xB1', b'\x00\x00', wait_sec=0.2)
        self.lock.release()
        if not res:
            return 0
        f = int.from_bytes(res[5:7], 'big', signed=True)
        return f

    def tare(self)->None:
        self.mean = self.get_raw_force()

    def get_force(self)->int:
        return self.get_raw_force() - self.mean

    def zero(self)->None:
        self.mean = 0
    
    def start_sample(self):
        res = self.com.send_cmd(self.addr, b'\xB0', b'\x00\x00')

    def __del__(self):
        self.com.close()


if __name__ == "__main__":

    com = COM('COM3', 38400)
    com.open()
    s = ForceSensor(com, 1)
    # s.set_buad(38400)
    # s.start_sample()

    while True:
        f = s.get_force()
        print(f)
        time.sleep(0.02)
    com.close()
