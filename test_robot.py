from rhino import init_hound
from time import sleep, time



def main():
    hound = init_hound(connection_str="/dev/tty.usbmodem11201")
    print(f"Initialized robot: {hound.name}")  # DEBUG

    print("Starting hound maneuver...")

    drive_duration = 3  # seconds
    start_time = time()
        
    while time() - start_time < drive_duration:
        # Send a command: 25% throttle, 0% right steer
        hound.send_velocity_cmd(throttle=0.75, steering=0)
        sleep(0.1) # Send commands at 10 Hz



    # Stop for 1 sec
    # print("STOP\t\t\t[1s]")
    # hound.apply_brake()
    sleep(1.0)

    start_time = time()
        
    while time() - start_time < drive_duration:
        # Send a command: 25% throttle, 0% right steer
        hound.send_velocity_cmd(throttle=0.25, steering=-1.0)
        sleep(0.1) # Send commands at 10 Hz

    print("Hound maneuver complete")
    hound.close()


if __name__ == "__main__":
    main()
