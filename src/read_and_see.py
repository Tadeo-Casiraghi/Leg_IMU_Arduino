import signal
import threading
from classes.arduino_reader import ArduinoSerial
import argparse
import cv2

terminate_program = False

# Signal handler for SIGTERM
def sigterm_handler(signal, frame):
    global terminate_program
    
    print("SIGTERM received, exiting gracefully...")
    # Set the global flag to terminate the program
    terminate_program = True


def record_video():
    global terminate_program

    # Set up video capture
    video_capture = cv2.VideoCapture(0)  # Change the parameter to the appropriate video source
    # Set resolution
    video_capture.set(3, 1280)
    video_capture.set(4, 720)

    # Define the codec and create a VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    output_file = 'output.avi'
    frame_width = int(video_capture.get(3))
    frame_height = int(video_capture.get(4))
    fps = int(video_capture.get(cv2.CAP_PROP_FPS))
    out = cv2.VideoWriter(output_file, fourcc, fps, (frame_width, frame_height))

    # Start recording
    while not terminate_program:
        # Capture frame-by-frame
        ret, frame = video_capture.read()

        if ret:
            # Write the frame into the file
            out.write(frame)

            # Display the resulting frame
            cv2.imshow('Video Recorder', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                terminate_program = True

    # Release the video capture and writer objects
    video_capture.release()
    out.release()

    # Close all windows
    cv2.destroyAllWindows()


def main(calibration = False):
    global terminate_program

    arduino = ArduinoSerial('COM4',115200) 
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

        video_thread = threading.Thread(target=record_video)
        video_thread.start()

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
