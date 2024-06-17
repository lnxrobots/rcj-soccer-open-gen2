import subprocess
import time
import signal
import sys
import json

import serial

PACKET_SIZE = 152
UNDERCARRIAGE_SERIAL_PORT = "/dev/ttyAMA0"
UNDERCARRIAGE_BOUD_RATE = 230400

class Button:
    def __init__(self, when_released=lambda: None, when_held=lambda: None):
        self.state = False
        self.when_released = when_released
        self.when_held = when_held
        self.press_time = time.time()
        self.held_done = False

    def update(self, state):
        if not self.state and state:
            self.press_time = time.time()
            self.held_done = False
        if self.state and not state and time.time() - self.press_time < 1:
            self.when_released()
        if not self.held_done and self.state and time.time() - self.press_time >= 1:
            self.when_held()
            self.held_done = True
        self.state = state


def get_wifi_state() -> bool:
    output = subprocess.check_output(['ifconfig', 'wlan0']).decode('utf-8').strip()
    return ('UP' in output.split('\n')[0])

def get_ip_addr() -> list[int]:
    ip_addr = subprocess.check_output(['hostname', '-I']).decode('ascii').strip().split()
    if ip_addr:
        return list(map(int, ip_addr[0].split('.')))
    return [300, 300, 300, 300]

def get_ssid() -> str:
    return subprocess.check_output(["iwgetid", "wlan0"]).decode('utf-8').strip().split("\"")[-2]

# BUTTON FUNCTIONS
def change_goal():
    global current_goal

    if current_goal == "blue":
        current_goal = "yellow"
    elif current_goal == "yellow":
        current_goal = "none"
    else:
        current_goal = "blue"

    print(f"Changing goal to {current_goal}")

def launch_program():
    global robot_program, robot_program_running

    robot_program_running = True
    print("Launching program")

    serial_connection.close()
    robot_program = subprocess.Popen(["sudo", "-u", "pi", "python3", "../RoboCupOpen/robot.py", current_goal, str(wifi_status)], cwd = "../RoboCupOpen", stdout = subprocess.DEVNULL, stderr = subprocess.STDOUT)

def change_wifi_status():
    global wifi_status

    wifi_status = not wifi_status
    print(f"Changing wifi status: {wifi_status}")

    if (wifi_status):
        subprocess.call(["sudo", "ifconfig", "wlan0", "up"])
    else:
        subprocess.call(["sudo", "ifconfig", "wlan0", "down"])

def terminate_program():
    robot_program.send_signal(signal.SIGINT)

def do_kick():
    global kick
    kick = True

def read_data() -> None:
    global data_packet
    len_data = len(data_packet)

    size_to_read = PACKET_SIZE - len_data
    in_waiting = serial_connection.in_waiting

    if(size_to_read > in_waiting):
        size_to_read = in_waiting

    #LoggerModule.log_info("Size to read: {}".format(size_to_read))
    raw_data = serial_connection.read(size_to_read)
    #LoggerModule.log_info("Raw data: {}".format(raw_data.hex()))

    for byte in raw_data:
        if(len_data <= 3):
            if(len_data == 0 and byte != 0x7b): #b'{'
                #LoggerModule.log_info("Byte: {}".format(hex(byte)))
                continue
            if(len_data == 1 and byte != 0x23): #b'#'
                len_data = 0
                data_packet.clear()
                continue
            if(len_data == 2 and byte != 0x66): #b'f'
                len_data = 0
                data_packet.clear()
                continue
            if(len_data == 3 and byte != 0x91): #145
                len_data = 0
                data_packet.clear()
                continue

        # print(repr(byte), len(raw_data), size_to_read)
        data_packet.append(byte)
        len_data += 1

    #LoggerModule.log_info("Len data: {}".format(len_data))
    if(len_data >= PACKET_SIZE):
        b = int(data_packet[20])
        button1.update(bool(b % 10))
        button2.update(bool(b // 10 % 10))
        button3.update(bool(b // 100))
        data_packet.clear()

wifi_status = get_wifi_state()
current_goal = "blue"
ip_addr = get_ip_addr()
robot_program_running = False
exit_code = 0
shuting_down = False
kick = False

robot_program: subprocess.Popen
serial_connection = serial.Serial(UNDERCARRIAGE_SERIAL_PORT, UNDERCARRIAGE_BOUD_RATE, timeout = 1, write_timeout=1)
serial_connection.flush()
data_packet: bytearray = bytearray()
show_ip = True
ip_data_switch_time = time.time()
ip_fetch_time = time.time()

button1 = Button(change_goal, do_kick)
button2 = Button(launch_program, change_wifi_status)
button3 = Button()

# bind_buttons()

start = time.time()
while True:
    while not robot_program_running and serial_connection.in_waiting > 0:
        read_data()
    if robot_program_running:
        if (robot_program.poll() is not None):
            exit_code = robot_program.poll()
            time.sleep(1)
            print(f"Program stopped. Exit code: {exit_code}")
            robot_program_running = False
            serial_connection = serial.Serial(UNDERCARRIAGE_SERIAL_PORT, UNDERCARRIAGE_BOUD_RATE, timeout = 1, write_timeout=1)
    else:
        if time.time() - ip_fetch_time > 5:
            ip_addr = get_ip_addr()
            ip_fetch_time = time.time()
        goal_num = ['blue', 'yellow', 'none'].index(current_goal)
        ip = [goal_num, int(wifi_status), exit_code, 0]
        if time.time() - ip_data_switch_time > 1:
            should_kick = False
            if kick:
                should_kick = True
                kick = False
            show_ip = not show_ip
            ip_data_switch_time = time.time()
            if show_ip:
                ip = ip_addr
            message_json = json.dumps({
                "e": ",".join(map(str, 5*[0]+[ord('G'), goal_num, ord('W'), int(wifi_status)]+ip+[0, int(should_kick), 0])),
            }, separators=(',', ':'))
            message_json += (79-len(message_json))*" "+"\n"
            print(f'{time.time()-start:.5f} sending')
            try:
                serial_connection.write(message_json.encode())
            except serial.SerialTimeoutException:
                print('write timeout')
    time.sleep(0.05)
