import cv2
import numpy as np
import serial
import time

# --- Configuration ---
# IMPORTANT: REPLACE 'COMX' with your Arduino's actual serial port!
ARDUINO_PORT = "COM3"  # e.g., 'COM3', '/dev/ttyACM0', '/dev/cu.usbmodemXXXX'
BAUD_RATE = 9600
TASK_COOLDOWN_TIME = (
    20  # Seconds: Estimated time for robot to complete one task (adjust as needed)
)
MAX_TASKS = 3  # Total number of tasks to perform (e.g., Task 1, Task 2, Task 3)

# --- Serial Communication Setup ---
arduino_serial = None
# Variable to hold the message to display on the OpenCV window
status_message_opencv = "[INIT] Connecting to Arduino..."

try:
    arduino_serial = serial.Serial(
        ARDUINO_PORT, BAUD_RATE, timeout=0.1
    )  # Shorter timeout for non-blocking read

    status_message_opencv = (
        f"[INIT] Giving Arduino 7 seconds to reset and initialize..."
    )
    print(status_message_opencv)
    time.sleep(7)  # Initial delay for Arduino reset and setup

    status_message_opencv = (
        f"[INIT] Successfully connected to Arduino on {arduino_serial.port}"
    )
    print(status_message_opencv)
    # Clear any potential buffered data from Arduino startup (good practice)
    arduino_serial.flushInput()
    arduino_serial.flushOutput()
except serial.SerialException as e:
    status_message_opencv = f"[ERROR] Could not open serial port {ARDUINO_PORT}: {e}"
    print(status_message_opencv)
    print(
        "    --> Make sure Arduino is connected, drivers are installed, correct COM port is specified,"
    )
    print("    --> AND Arduino IDE's Serial Monitor is CLOSED.")
    arduino_serial = None  # Ensure it's None if connection failed


def detect_brown_objects(frame):
    """
    Detects brown objects in a frame, draws bounding boxes.
    Returns the processed frame and a boolean indicating if *any* block was detected.
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_brown = np.array([10, 50, 50])
    upper_brown = np.array([30, 255, 200])
    mask = cv2.inRange(hsv, lower_brown, upper_brown)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    block_detected_in_frame = False
    for contour in contours:
        if cv2.contourArea(contour) > 500:  # Filter small noise (adjust as needed)
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            block_detected_in_frame = True
    return frame, block_detected_in_frame


# Initialize the camera
cap = cv2.VideoCapture(1)  # Try index 1 first (external webcam)

if not cap.isOpened():
    status_message_opencv = (
        "[ERROR] Could not open webcam at index 1. Trying index 0..."
    )
    print(status_message_opencv)
    cap = cv2.VideoCapture(0)  # Try index 0 (built-in webcam)
    if not cap.isOpened():
        status_message_opencv = (
            "[ERROR] Could not open webcam at index 0 either. Exiting."
        )
        print(status_message_opencv)
        if arduino_serial and arduino_serial.is_open:
            arduino_serial.close()
        exit()

status_message_opencv = (
    "[INFO] Camera initialized. Waiting 5 seconds for camera to warm up..."
)
print(status_message_opencv)
time.sleep(5)  # Delay for camera warm-up
status_message_opencv = "[INFO] Waiting complete. Starting object detection."
print(status_message_opencv)
status_message_opencv = (
    f"[STATUS] Searching for blocks to start {MAX_TASKS} tasks. Press 'q' to quit."
)
print(status_message_opencv)


# --- Task Management Variables ---
tasks_completed_count = 0
robot_busy = False  # Flag to indicate if robot is currently performing a task
task_start_time = 0  # To track when the current task started for cooldown
all_blocks_processed_message_shown = (
    False  # To ensure the final message is shown only once
)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            status_message_opencv = "[ERROR] Failed to capture image. Exiting loop."
            print(status_message_opencv)
            break

        processed_frame, detected_status = detect_brown_objects(frame)

        # --- Read from Arduino and update status message ---
        if arduino_serial and arduino_serial.in_waiting > 0:
            try:
                # Read all available lines to get the latest message
                while arduino_serial.in_waiting > 0:
                    arduino_response = (
                        arduino_serial.readline()
                        .decode("utf-8", errors="ignore")
                        .strip()
                    )
                    if arduino_response:  # Only update if not an empty line
                        status_message_opencv = f"[ARDUINO] {arduino_response}"
                        print(
                            f"[SERIAL_RAW] {arduino_response}"
                        )  # Also print to console for full log
            except Exception as e:
                status_message_opencv = (
                    f"[SERIAL ERROR] Error reading from Arduino: {e}"
                )
                print(status_message_opencv)

        # --- Logic for sequential tasks ---
        if not robot_busy:  # Only look for new blocks if the robot is NOT busy
            if detected_status:  # A block is currently detected
                if tasks_completed_count < MAX_TASKS:  # Still have tasks to perform
                    if not (
                        tasks_completed_count == MAX_TASKS - 1
                        and all_blocks_processed_message_shown
                    ):  # Don't trigger if already finished
                        tasks_completed_count += 1  # Increment task counter
                        command_to_send = str(
                            tasks_completed_count
                        )  # Prepare command ('1', '2', or '3')

                        status_message_opencv = f"[DETECTION] Block detected! Initiating Task {tasks_completed_count}..."
                        print(status_message_opencv)

                        if arduino_serial and arduino_serial.is_open:
                            try:
                                arduino_serial.write(
                                    command_to_send.encode()
                                )  # Send command to Arduino
                                status_message_opencv = f"[SERIAL] Command '{command_to_send}' sent. Robot busy."
                                print(status_message_opencv)
                                robot_busy = True  # Set robot to busy state
                                task_start_time = (
                                    time.time()
                                )  # Record the start time of this task
                            except serial.SerialException as write_e:
                                status_message_opencv = f"[SERIAL ERROR] Failed to write to serial port: {write_e}"
                                print(status_message_opencv)
                                robot_busy = False  # Reset if write error
                            except Exception as gen_e:
                                status_message_opencv = f"[SERIAL ERROR] An unexpected error occurred while writing: {gen_e}"
                                print(status_message_opencv)
                                robot_busy = False  # Reset if error
                        else:
                            status_message_opencv = "[SERIAL] Arduino serial port is NOT open. Cannot send command."
                            print(status_message_opencv)
                else:  # tasks_completed_count >= MAX_TASKS, but block is *still* detected
                    status_message_opencv = (
                        "[STATUS] All tasks done. Block detected. Clear area."
                    )
                    print(status_message_opencv)
            else:  # No block detected
                # Only update status if robot isn't busy and we are searching
                if not robot_busy and tasks_completed_count < MAX_TASKS:
                    status_message_opencv = "[STATUS] Searching for blocks..."
                elif (
                    tasks_completed_count >= MAX_TASKS
                    and not all_blocks_processed_message_shown
                ):
                    status_message_opencv = (
                        "\n[INFO] ALL BLOCKS PROCESSED! No more blocks detected."
                    )
                    print(status_message_opencv)
                    all_blocks_processed_message_shown = True
                    # Uncomment the line below if you want the program to fully exit
                    # after all tasks are done AND no block is detected.
                    # break

        # --- Cooldown timer for robot busy state ---
        if robot_busy:
            elapsed_time = time.time() - task_start_time
            if elapsed_time < TASK_COOLDOWN_TIME:
                # Update busy status with countdown
                status_message_opencv = f"[ROBOT] Task {tasks_completed_count} in progress... ({int(TASK_COOLDOWN_TIME - elapsed_time)}s remaining)"
            else:
                robot_busy = False
                status_message_opencv = f"[ROBOT] Cooldown for Task {tasks_completed_count} finished. Ready for next detection."
                print(status_message_opencv)

        # --- Display status message on OpenCV window ---
        cv2.putText(
            processed_frame,
            status_message_opencv,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )  # Yellow text

        # Show the frame
        cv2.imshow("Brown Object Detection", processed_frame)

        # Allow manual exit by pressing 'q'
        if cv2.waitKey(1) & 0xFF == ord("q"):
            print("[INFO] Manual exit initiated by user.")
            break

except KeyboardInterrupt:
    print("[INFO] Program interrupted by user (Ctrl+C).")
finally:
    # --- Cleanup Resources ---
    if cap.isOpened():
        cap.release()
        print("[INFO] Camera released.")
    cv2.destroyAllWindows()
    print("[INFO] OpenCV windows closed.")
    if arduino_serial and arduino_serial.is_open:
        arduino_serial.close()
        print("[SERIAL] Arduino serial port closed.")
    print("[INFO] Program finished.")
