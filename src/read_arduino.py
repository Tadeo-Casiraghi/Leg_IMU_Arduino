from classes.arduino_reader import ArduinoSerial
import signal
import argparse

terminate_program = False

# Signal handler for SIGTERM
def sigterm_handler(signal, frame):
    global terminate_program
    
    print("SIGTERM received, exiting gracefully...")
    # Set the global flag to terminate the program
    terminate_program = True


def main(calibration=False):
    global terminate_program

    arduino = ArduinoSerial('COM3',115200) 
    arduino.wait_for_arduino()

    if calibration:
        input('Start calibration?')
        print('Starting Calibration. Please move all joints slowly')
        arduino.toggle_imus()
        while not terminate_program:
            arduino.read_data()
        arduino.toggle_imus()
        arduino.save_data(calibration = True)
        print('Done calibrating')

    else:
        input('Start measurements?')

        arduino.toggle_imus()

        while not terminate_program:
            arduino.read_data()
            
        arduino.toggle_imus()
        
        arduino.save_data()

        arduino.close_serial()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Read configuration")
    parser.add_argument("-c", "--calibrate", action="store_true", help = 'Create new data folder and save calibration data')
    args = parser.parse_args()
    # Register the signal handler
    signal.signal(signal.SIGTERM, sigterm_handler)
    signal.signal(signal.SIGINT, sigterm_handler)
    main(args.calibrate)

