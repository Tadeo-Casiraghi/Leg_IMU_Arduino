import serial
import time
import os


class ArduinoSerial:

    def __init__(self, port, baudrate, data_location = None):
        self.ser = serial.Serial(port,baudrate)
        time.sleep(1)
        self.values = []
        self.packetsize = 6*2*3+4
        if data_location == None:
            current_directory = os.path.dirname(os.path.abspath(__file__))
            # Go up one directory
            parent_directory = os.path.dirname(current_directory)
            # Go into the "data" directory
            self.data_directory = os.path.join(parent_directory, 'data')
    
    def wait_for_arduino(self):
        while True:
            msg = self.ser.readline()
            msg = msg.decode()
            if "Start" in msg:
                break

    def calibration(self,):
        print('Starting Calibration. Please move all joints slowly')
        self.toggle_imus()
        t0 = time.monotonic()
        while time.monotonic() - t0 < 10:
            self.read_data()
        self.toggle_imus()
        self.save_data('calibration')
        print('Done calibrating')
    
    def toggle_imus(self):
        self.ser.write(b"1")

    def close_serial(self):
        self.ser.close()

    def read_data(self):
        self.values.append(self.ser.read(self.packetsize))
    
    def save_data(self, filename = None, extension = 'data'):
        if filename == None:
            current_time_seconds = time.time()
            filename = time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime(current_time_seconds)) + f"_{extension}" +  ".bin"
        with open(os.path.join(self.data_directory, filename), "wb") as bin_file:
            for data in self.values:
                bin_file.write(data)
        self.values = []

