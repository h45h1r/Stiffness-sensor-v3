<<<<<<< HEAD

import serial
import time
import timeit
from threading import Event
import numpy as np
import csv
import nidaqmx

import re

np.set_printoptions(formatter={'float_kind':'{:f}'.format})

COM_port = 'COM5'
BAUD_RATE = 115200

x_cord = -146
y_cord = -44
z_cord = -8.220

X_sensor = 'X' + str(x_cord)
Y_sensor = 'Y' + str(y_cord)
Z_sensor = 'Z' + str(z_cord)

s_machine = serial.Serial('COM5', 115200)

def read_calibration_file():

    cal_file = "FT26196.cal"
    cal_file_index=1
    with open(cal_file, 'r') as f:
        lines = f.read().splitlines()

    calibration_raw_data = [0]*6
    for i in range (6):
        calibration_raw_data[i] = lines[-9+i]

    calibration_matrix = [0]*6
    for i in range (6):
        calibration_matrix[i] = [float(s) for s in re.findall(r'[\d]*[.][\d]+', calibration_raw_data[i])]
        calibration_matrix[i] = [float(s) for s in re.findall(r'[-+]?(?:\d{1,3}(?:,\d{3})+|\d+)(?:\.\d+)?', calibration_raw_data[i])]

    print(calibration_matrix)
    calibration_matrix_trimmed = [ [0]*6 for i in range(6)]
    # global calibration_matrix_trimmed
    calibration_matrix_trimmed = np.delete(calibration_matrix, -1, 1)
    print(calibration_matrix_trimmed)

def sensor_bias():
    global bias_reading
    bias_reading = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    for i in range(10):
        get_DAQ_data()
        # print(np.array(data_from_DAQ))
        bias_reading = np.add(bias_reading, np.array(data_from_DAQ))
        # print(bias_reading)

    bias_reading = bias_reading / 10
    # print(bias_reading)

def initialize_ATI_mini40():
    read_calibration_file()
    sensor_bias()

def get_DAQ_data():

    with nidaqmx.Task() as task:
        V0= task.ai_channels.add_ai_voltage_chan("Dev1/ai0")
        # print(V0)
        V1= task.ai_channels.add_ai_voltage_chan("Dev1/ai1")
        # print(V1)
        V2= task.ai_channels.add_ai_voltage_chan("Dev1/ai2")
        # print(V2)
        V3= task.ai_channels.add_ai_voltage_chan("Dev1/ai3")
        # print(V3)
        V4= task.ai_channels.add_ai_voltage_chan("Dev1/ai4")
        # print(V4)
        V5= task.ai_channels.add_ai_voltage_chan("Dev1/ai5")
        # print(V5)
        # V6 = task.ai_channels.add_ai_voltage_chan("Dev1/ai6")
        # # print(V6)
        global data_from_DAQ
        data_from_DAQ = task.read()

def get_FT_data():
    # get_DAQ_data()
    raw_measurement = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    raw_measurement_biased = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    global calibrated_output
    calibrated_output = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    raw_measurement = data_from_DAQ
    raw_measurement_biased = raw_measurement - bias_reading
    # print(raw_measurement_biased)

    calibrated_output = np.transpose(np.matmul(calibration_matrix_trimmed, np.transpose(raw_measurement_biased)))
    # print(calibrated_output)
    return calibrated_output


def wait_for_movement_completion():
    Event().wait(1)
    idle_counter = 0

    while True:
        Event().wait(0.01)
        s_machine.reset_input_buffer()
        command_status = str.encode('?' + '\n')
        s_machine.write(command_status)
        print(command_status)
        input_response = s_machine.readline()
        print(input_response)
        decoded_response = input_response.strip().decode('utf-8')

        if decoded_response != 'ok':

            if decoded_response.find('Idle') > 0:
                # print('IDLE FOUND!!!!')
                idle_counter += 1

        if idle_counter > 5:
            break


def send_command(command):
    # l = command
    if command != '$X' or '$$':
        # print('Inside IF condition of send_command')
        # print('Sending: ' + command)
        s_machine.write((command + '\n').encode())  # Send g-code block to grbl
        response = s_machine.readline()  # Wait for grbl response with carriage return
        print(response.strip())
        wait_for_movement_completion()

    else:
        print('Inside ELSE condition of send_command')
        print('Sending: ' + command)
        s_machine.write((command + '\n').encode())  # Send g-code block to grbl
        wait_for_movement_completion()
        response = s_machine.readline()  # Wait for grbl response with carriage return
        print(response.strip())
        wait_for_movement_completion()

def initialize_cnc():
    s_machine.write("\r\n\r\n".encode())
    time.sleep(2)  # Wait for grbl to initialize
    s_machine.flushInput()  # Flush startup text in serial input
    command_list = ('$$', '$X', 'G90', '')
    for x in command_list:
        print('Initializing')
        send_command(x)
    send_command('G21') #input in mm
    feed_rate(1000, 1000, 100)


def feed_rate(X_feed, Y_feed, Z_feed):
    send_command('$110=' + str(X_feed))
    send_command('$111=' + str(Y_feed))
    send_command('$112=' + str(Z_feed))

def get_data_arduino ():
    print('x')

def mean_force_data():
    force_data = 0
    Fx_mean = 0
    Fy_mean = 0
    Fz_mean = 0
    for x in range(5):
        force_data = get_FT_data()
        Fx = force_data[0]
        Fy = force_data[1]
        Fz = force_data[2]
        Fx_mean = Fx_mean + Fx
        Fy_mean = Fy_mean + Fy
        Fz_mean = Fz_mean + Fz

    return Fx_mean/5, Fy_mean/5, Fz_mean/5


file_name = '0_PSI_3mm_1.csv'


initialize_ATI_mini40()
initialize_cnc()
wait_for_movement_completion()
#Machine Home
send_command('$H')
send_command(X_sensor)
send_command(Y_sensor)
send_command(Z_sensor)

steps = np.linspace(0,5,51)
step_size = 0.05
smaller_step_size = 0.05
incremented_step = 0
incremented_step = round(z_cord - step_size, 5)
a = 0


while True:

    time.sleep(0.2)
    sensor_zero = 0
    force_data = get_FT_data()
    Fx = force_data[0]
    Fy = force_data[1]
    Fz = force_data[2]
    print(Fz)
    Fx_mean, Fy_mean, Fz_mean = mean_force_data()
    print(Fz_mean)

    if Fz_mean > -0.03:
        print("No Force")
        send_command('Z' + str(incremented_step))
        wait_for_movement_completion()
        incremented_step = round(incremented_step - step_size, 5)
    else:
        print("Sensor Touching")
        incremented_step = round(incremented_step + 2*step_size, 5)
        send_command('Z' + str(incremented_step))
        wait_for_movement_completion()
        while True:
            force_data = get_FT_data()
            Fx = force_data[0]
            Fy = force_data[1]
            Fz = force_data[2]
            print(Fz)




    send_command('Z' + str(sensor_zero))
    wait_for_movement_completion()
    time.sleep(0.1)
    zero_steps = np.linspace(0, 4, 41)
    # print('inside if')
    initial_time = timeit.default_timer()
    for j in range(len(zero_steps)):
        # print('inside for loop')
        zero_incremented_step = round(sensor_zero - zero_steps[j], 5)
        send_command('Z' + str(zero_incremented_step))
        wait_for_movement_completion()
        force_data = get_FT_data()
        Fx = force_data[0]
        Fy = force_data[1]
        Fz = force_data[2]
        # time.sleep(0.2)
        print(zero_incremented_step)
        for k in range(30):
            data_buffer = list()
            # s_arduino.write("0".encode())
            # data = s_arduino.readline()  # Wait for arduino data
            # decoded_data = data.decode("utf-8").rstrip("\r\n")
            force_data = get_FT_data()
            Fx = force_data[0]
            Fy = force_data[1]
            Fz = force_data[2]
            list_decoded_data = list(map(lambda x: float(x), decoded_data.split(",")))
            stamp = timeit.default_timer() - initial_time
            data_buffer.append([round(float(stamp), 4)] + [round(zero_steps[j], 4)] + list_decoded_data + list([Fx]) + list([Fy]) + list([Fz]))
            [data_buffer] = data_buffer
            with open(file_name, "a", newline='') as f:
                writer = csv.writer(f)
                writer.writerow(data_buffer)
    send_command('Z0')



=======

import serial
import time
import timeit
from threading import Event
import numpy as np
import csv
import nidaqmx

import re

np.set_printoptions(formatter={'float_kind':'{:f}'.format})

# COM_port = 'COM10'
# BAUD_RATE = 115200

x_cord = -146
y_cord = -44
z_cord = -8.220

X_sensor = 'X' + str(x_cord)
Y_sensor = 'Y' + str(y_cord)
Z_sensor = 'Z' + str(z_cord)

s_machine = serial.Serial('/dev/tty.usbserial-11340', 115200)

def read_calibration_file():

    cal_file = "FT26196.cal"
    cal_file_index=1
    with open(cal_file, 'r') as f:
        lines = f.read().splitlines()

    calibration_raw_data = [0]*6
    for i in range (6):
        calibration_raw_data[i] = lines[-9+i]

    calibration_matrix = [0]*6
    for i in range (6):
        # calibration_matrix[i] = [float(s) for s in re.findall(r'[\d]*[.][\d]+', calibration_raw_data[i])]
        calibration_matrix[i] = [float(s) for s in re.findall(r'[-+]?(?:\d{1,3}(?:,\d{3})+|\d+)(?:\.\d+)?', calibration_raw_data[i])]

    # print(calibration_matrix)
    # calibration_matrix_trimmed = [ [0]*6 for i in range(6)]
    global calibration_matrix_trimmed
    calibration_matrix_trimmed = np.delete(calibration_matrix, -1, 1)
    # print(calibration_matrix_trimmed)

# def sensor_bias():
#     global bias_reading
#     bias_reading = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
#     for i in range(10):
#         get_DAQ_data()
#         # print(np.array(data_from_DAQ))
#         bias_reading = np.add(bias_reading, np.array(data_from_DAQ))
#         # print(bias_reading)

#     bias_reading = bias_reading / 10
#     # print(bias_reading)

def initialize_ATI_mini40():
    read_calibration_file()
    sensor_bias()

# def get_DAQ_data():

#     with nidaqmx.Task() as task:
#         V0= task.ai_channels.add_ai_voltage_chan("Dev1/ai0")
#         # print(V0)
#         V1= task.ai_channels.add_ai_voltage_chan("Dev1/ai1")
#         # print(V1)
#         V2= task.ai_channels.add_ai_voltage_chan("Dev1/ai2")
#         # print(V2)
#         V3= task.ai_channels.add_ai_voltage_chan("Dev1/ai3")
#         # print(V3)
#         V4= task.ai_channels.add_ai_voltage_chan("Dev1/ai4")
#         # print(V4)
#         V5= task.ai_channels.add_ai_voltage_chan("Dev1/ai5")
#         # print(V5)
#         # V6 = task.ai_channels.add_ai_voltage_chan("Dev1/ai6")
#         # # print(V6)
#         global data_from_DAQ
#         data_from_DAQ = task.read()

# def get_FT_data():
#     # get_DAQ_data()
#     raw_measurement = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
#     raw_measurement_biased = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
#     global calibrated_output
#     calibrated_output = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
#     raw_measurement = data_from_DAQ
#     raw_measurement_biased = raw_measurement - bias_reading
#     # print(raw_measurement_biased)

#     calibrated_output = np.transpose(np.matmul(calibration_matrix_trimmed, np.transpose(raw_measurement_biased)))
#     # print(calibrated_output)
#     return calibrated_output


def wait_for_movement_completion():
    Event().wait(1)
    idle_counter = 0

    while True:
        Event().wait(0.01)
        s_machine.reset_input_buffer()
        command_status = str.encode('?' + '\n')
        s_machine.write(command_status)
        # print(command_status)
        input_response = s_machine.readline()
        # print(input_response)
        decoded_response = input_response.strip().decode('utf-8')

        if decoded_response != 'ok':

            if decoded_response.find('Idle') > 0:
                # print('IDLE FOUND!!!!')
                idle_counter += 1

        if idle_counter > 5:
            break


def send_command(command):
    # l = command
    if command != '$X' or '$$':
        # print('Inside IF condition of send_command')
        # print('Sending: ' + command)
        s_machine.write((command + '\n').encode())  # Send g-code block to grbl
        response = s_machine.readline()  # Wait for grbl response with carriage return
        # print(response.strip())
        # wait_for_movement_completion()

    else:
        # print('Inside ELSE condition of send_command')
        # print('Sending: ' + command)
        s_machine.write((command + '\n').encode())  # Send g-code block to grbl
        # wait_for_movement_completion()
        response = s_machine.readline()  # Wait for grbl response with carriage return
        # print(response.strip())
        wait_for_movement_completion()

def initialize_cnc():
    s_machine.write("\r\n\r\n".encode())
    time.sleep(2)  # Wait for grbl to initialize
    s_machine.flushInput()  # Flush startup text in serial input
    command_list = ('$$', '$X', 'G90', '')
    for x in command_list:
        print('Initializing')
        send_command(x)
    send_command('G21') #input in mm
    feed_rate(1000, 1000, 100)


def feed_rate(X_feed, Y_feed, Z_feed):
    send_command('$110=' + str(X_feed))
    send_command('$111=' + str(Y_feed))
    send_command('$112=' + str(Z_feed))

def get_data_arduino ():
    print('x')

def mean_force_data():
    force_data = 0
    Fx_mean = 0
    Fy_mean = 0
    Fz_mean = 0
    for x in range(5):
        force_data = get_FT_data()
        Fx = force_data[0]
        Fy = force_data[1]
        Fz = force_data[2]
        Fx_mean = Fx_mean + Fx
        Fy_mean = Fy_mean + Fy
        Fz_mean = Fz_mean + Fz

    return Fx_mean/5, Fy_mean/5, Fz_mean/5


file_name = '0_PSI_3mm_1.csv'


# initialize_ATI_mini40()
initialize_cnc()
wait_for_movement_completion()
#Machine Home
send_command('$H')
send_command(X_sensor)
send_command(Y_sensor)
send_command(Z_sensor)

# steps = np.linspace(0,5,51)
step_size = 0.05
smaller_step_size = 0.05
incremented_step = 0
# incremented_step = round(z_cord - step_size, 5)
a = 0


while True:

    # time.sleep(0.2)
    # sensor_zero = 0
    # force_data = get_FT_data()
    # Fx = force_data[0]
    # Fy = force_data[1]
    # Fz = force_data[2]
    # print(Fz)
    Fx_mean, Fy_mean, Fz_mean = mean_force_data()
    print(Fz_mean)

    if Fz_mean > -0.03:
        print("No Force")
        send_command('Z' + str(incremented_step))
        wait_for_movement_completion()
        incremented_step = round(incremented_step - step_size, 5)
    else:
        print("Sensor Touching")
        incremented_step = round(incremented_step + 2*step_size, 5)
        send_command('Z' + str(incremented_step))
        wait_for_movement_completion()
        while True:
            force_data = get_FT_data()
            Fx = force_data[0]
            Fy = force_data[1]
            Fz = force_data[2]
            print(Fz)




    # send_command('Z' + str(sensor_zero))
    # wait_for_movement_completion()
    # time.sleep(0.1)
    # zero_steps = np.linspace(0, 4, 41)
    # # print('inside if')
    # initial_time = timeit.default_timer()
    # for j in range(len(zero_steps)):
    #     # print('inside for loop')
    #     zero_incremented_step = round(sensor_zero - zero_steps[j], 5)
    #     send_command('Z' + str(zero_incremented_step))
    #     wait_for_movement_completion()
    #     force_data = get_FT_data()
    #     Fx = force_data[0]
    #     Fy = force_data[1]
    #     Fz = force_data[2]
    #     # time.sleep(0.2)
    #     print(zero_incremented_step)
        # for k in range(30):
        #     data_buffer = list()
        #     # s_arduino.write("0".encode())
        #     # data = s_arduino.readline()  # Wait for arduino data
        #     # decoded_data = data.decode("utf-8").rstrip("\r\n")
        #     force_data = get_FT_data()
        #     Fx = force_data[0]
        #     Fy = force_data[1]
        #     Fz = force_data[2]
        #     list_decoded_data = list(map(lambda x: float(x), decoded_data.split(",")))
        #     stamp = timeit.default_timer() - initial_time
        #     data_buffer.append([round(float(stamp), 4)] + [round(zero_steps[j], 4)] + list_decoded_data + list([Fx]) + list([Fy]) + list([Fz]))
        #     [data_buffer] = data_buffer
        #     with open(file_name, "a", newline='') as f:
        #         writer = csv.writer(f)
        #         writer.writerow(data_buffer)
    # send_command('Z0')



>>>>>>> 93ee8941168e65bf3df490964862dcd668771fc6
