from classes.arduino_reader import ArduinoSerial
import signal
import time

terminate_program = False

# Signal handler for SIGTERM
def sigterm_handler(signal, frame):
    global terminate_program
    
    print("SIGTERM received, exiting gracefully...")
    # Set the global flag to terminate the program
    terminate_program = True


def main():
    global terminate_program

    arduino = ArduinoSerial('COM4',115200) 
    arduino.wait_for_arduino()

    input('Start calibration?')
    arduino.calibration()


    input('Start measurements?')

    arduino.toggle_imus()

    while not terminate_program:
        arduino.read_data()
        
    arduino.toggle_imus()
    arduino.save_data()

    arduino.close_serial()


if __name__ == '__main__':
    # Register the signal handler
    signal.signal(signal.SIGTERM, sigterm_handler)
    signal.signal(signal.SIGINT, sigterm_handler)

    main()

