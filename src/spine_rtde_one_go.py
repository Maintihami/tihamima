import ctypes
import time
import matplotlib.pyplot as plt
import numpy as np
import os
import sys
import logging
"""the program collects data from the force torque sensor and change the trajectory of the robot when the force limits are exceeded"""
"""the program gives the user the possibility to select the forces to monitor and the thresholds in the tool frame"""
"""many forces can be monitored at the same time with different thresholds and logic operators"""
"""the program gives the user the possiblity to specify the speed vector in the base frame that dictate the trajectory of the robot"""
"""the program gives the user the possibility to change the trajectory of the robot when the force limits are exceeded"""
"""the frame is the tool frame"""
"""Does reinitialize the F/T lists after the force limits are exceeded"""
"""manage the negtive limits"""
"""the program collects all necessary inputs in one go (at the beginning) and then operates based on those inputs. It is concise and accurately reflects the batch input method."""
"""-----------------------------sensor initialization-----------------------------------"""
# Function to convert the force from the sensor frame to the tool frame
def rotation(force):
    Tz = np.array([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, -1]
    ])
    forces = np.dot(Tz, force.T).T
    return forces

# Relative path starting from the directory where your Python script is located
relative_path = r"../lib/AMTIUSBDevice - 64.dll"


# Construct the absolute path
dll_path = os.path.join(os.path.dirname(__file__), relative_path)

try:
    amti_dll = ctypes.CDLL(dll_path)
    print(f"Successfully loaded DLL: {dll_path}")
except OSError as e:
    print(f"Error loading DLL: {e}")
    exit(1)
'''Define the function prototypes as described in the DLL documentation. 
This tells Python how to call these functions and what type of data they
 return or expect as arguments.'''
# Initialize the DLL
amti_dll.fmDLLInit.restype = None

# Check if the device initialization is complete
amti_dll.fmDLLIsDeviceInitComplete.restype = ctypes.c_int

# Set data collection method to post messages
amti_dll.fmDLLPostDataReadyMessages.restype = None
amti_dll.fmDLLPostDataReadyMessages.argtypes = [ctypes.c_int]

# Start and stop data acquisition
amti_dll.fmBroadcastStart.restype = None
amti_dll.fmBroadcastStop.restype = None

# Get data (polling method)
amti_dll.fmDLLTransferFloatData.restype = ctypes.c_int
amti_dll.fmDLLTransferFloatData.argtypes = [ctypes.POINTER(ctypes.POINTER(ctypes.c_float))]

# Prototype pour définir les sensibilités DAC
amti_dll.fmSetDACSensitivityTable.restype = None
amti_dll.fmSetDACSensitivityTable.argtypes = [ctypes.POINTER(ctypes.c_float)]
# Prototype pour définir les offsets des canaux
amti_dll.fmSetChannelOffsetsTable.restype = None
amti_dll.fmSetChannelOffsetsTable.argtypes = [ctypes.POINTER(ctypes.c_float)]
# Initialiser la DLL
print("Initializing the DLL...")
amti_dll.fmDLLInit()

# Wait for initialization to complete
time.sleep(0.25)  # Sleep for 250 milliseconds
status = amti_dll.fmDLLIsDeviceInitComplete()
if status == 0:
    raise Exception("DLL has not completed initializing.")
elif status == 1:
    raise Exception("DLL initialized, but no signal conditioners are present.")
elif status == 2:
    print("DLL initialization complete and devices found.")


# define recommended settings
recommended_gains = (ctypes.c_long * 6)(8, 8, 8, 1, 1, 1)  # réglages de gains
recommended_excitations = (ctypes.c_long * 6)(2, 2, 2, 2, 2, 2)  #réglages d'excitations
recomended_dac_sensitivities = (ctypes.c_float * 6)(27.454, 27.930, 6.2936, 191.460, 191.4600, 146.300)
zero_offset = (ctypes.c_float * 6)(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)    #offsets
# Apply recommended settings
amti_dll.fmSetCurrentGains(recommended_gains)
amti_dll.fmSetCurrentExcitations(recommended_excitations)
amti_dll.fmSetDACSensitivityTable(recomended_dac_sensitivities)


# Create a dictionary to store the units based on the mode
units_dict = {
    0: ("N", "Nm", 'Configured data output mode to metric MSA 6 Compatible.'),  # Metric MSA 6 Compatible
    1: ("N", "Nm", 'Configured data output mode to metric fully conditioned.'),  # Metric Fully Conditioned
    2: ("lb", "lb-ft", 'Configured data output mode to english MSA 6 Compatible.'),  # English MSA 6 Compatible
    3: ("lb", "lb-ft", 'Configured data output mode to english Fully Conditioned.'),  # English Fully Conditioned
    4: ("bits", "bits", 'Configured data output mode to Bits MSA 6 Compatible.')  # Bits MSA 6 Compatible
}
# Function to get a valid mode from the user
def get_valid_mode():
    while True:
        try:
            mode = int(input("Enter the desired mode (0-4) [0&1: metric, 2&3: english, 4: Bits]: "))
            if mode in units_dict:
                return mode
            else:
                print("Invalid input. Please enter a number between 0 and 4.")
        except ValueError:
            print("Invalid input. Please enter an integer between 0 and 4.")

# Configure data output mode
mode = get_valid_mode()
amti_dll.fmBroadcastRunMode(mode)
force_units, moment_units, commentaire = units_dict[mode]
print(commentaire)

# Function to get a valid signal frequency from the user
def get_valid_frequency(max_recommended=500):
    while True:
        try:
            frequency = int(input(f"Enter the desired signal frequency in Hz (recommended up to {max_recommended} Hz): "))
            if frequency > 0:
                if frequency > max_recommended:
                    print(f"Note: {frequency} Hz exceeds the recommended maximum of {max_recommended} Hz.")
                return frequency
            else:
                print("Invalid input. Please enter a positive integer.")
        except ValueError:
            print("Invalid input. Please enter an integer.")

# Set the acquisition rate
signal_frequency = get_valid_frequency()
amti_dll.fmBroadcastAcquisitionRate(signal_frequency)       #The new acquisition rate will take affect with the next Start command
print(f"Acquisition rate set to {signal_frequency} Hz.")

# define the size of the packet
packet_size = 512       # 512 bytes means 128 float values with 4 bytes each
amti_dll.fmDLLSetUSBPacketSize(packet_size)
print(f"USB Packet size set to {packet_size} bytes")
# Define a pointer for data
data_pointer = ctypes.POINTER(ctypes.c_float)()
nbr_data_pooled = 128     # Number of float values to pool


# Listes pour stocker les données pour les forces et les moments
Fx_values, Fy_values, Fz_values = [], [], []
Mx_values, My_values, Mz_values = [], [], []
counter_values = []     #to store the counter received values, it does not include the none values
data_values = []        #to store the data received, it includes the none values

# Define the sample rate
# The  Nyquist–Shannon sampling theorem states that the sample rate must be at least twice the bandwidth of the signal to avoid aliasing
#sample_rate = 1/(signal_frequency)      #Median loop frequency: 391.66 Hz
sample_rate = 1/(2*signal_frequency)     #Median loop frequency: 791.08 Hz

# Define the relative path for the output folder
output_folder = '../output'

# Get the absolute path to the script's directory
script_dir = os.path.dirname(__file__)

# Construct the full path to the output folder
output_path = os.path.join(script_dir, output_folder)

# Create the output folder if it doesn't exist
os.makedirs(output_path, exist_ok=True)

# Specify the relative path for the new output file
output_file_path = os.path.join(output_path, "output.txt")
"""-----------------------------control parameters-----------------------------------"""

# Add the parent directory of src to the Python path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
# Add the rtde directory to the Python path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../rtde')))
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import socket
"""this code stops the robot when the force limits are exceeded
the forces are read from the internal force torque sensor of the robot and compared to the thresholds"""
# Parameters for the RTDE communication
ROBOT_HOST = '192.168.1.50'     # IP adress of the robot 
ROBOT_PORT = 30004

# Define the relative path to the configuration file
config_folder = '../config'
config_file = 'control_loop_configuration_spine.xml'

# Construct the full path to the configuration file
config_filename = os.path.join(script_dir, config_folder, config_file)
keep_running = True
logging.getLogger().setLevel(logging.INFO)

# Charge the rdte configuration
conf = rtde_config.ConfigFile(config_filename)
state_names, state_types = conf.get_recipe('state')
Vspeed_names, Vspeed_types = conf.get_recipe('Vspeed')
servoing_names, servoing_types = conf.get_recipe('servoing')
# Connexion to the robot
con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
con.connect()

# Get the controller version
con.get_controller_version()

# Send the recipes to the controller
con.send_output_setup(state_names, state_types)
Vspeed = con.send_input_setup(Vspeed_names, Vspeed_types)
servoing = con.send_input_setup(servoing_names, servoing_types)

Vspeed.input_double_register_0 = 0
Vspeed.input_double_register_1 = 0
Vspeed.input_double_register_2 = 0
Vspeed.input_double_register_3 = 0
Vspeed.input_double_register_4 = 0
Vspeed.input_double_register_5 = 0
servoing.input_int_register_0 = 0
# Fonctions utilitaires pour la conversion entre les listes et les points de consigne
def Vspeed_to_list(Vspeed):
    return [Vspeed.__dict__[f"input_double_register_{i}"] for i in range(6)]

def list_to_Vspeed(Vspeed, lst):
    for i in range(6):
        Vspeed.__dict__[f"input_double_register_{i}"] = lst[i]
    return Vspeed


# Function to check force limits based on user selection
def check_force_limits(selected_forces, thresholds, logic_op):
    def check_condition(force_data_lists):
        conditions = []
        exceeded_indices = []
        for i, force_list in enumerate(force_data_lists):
            threshold = thresholds[i]
            if threshold >= 0:  # Threshold is positive or zero
                exceeded = any(value >= threshold for value in force_list)
            else:  # Threshold is negative
                exceeded = any(value <= threshold for value in force_list)
                
            if exceeded:
                conditions.append(True)
                exceeded_indices.append(i)
            else:
                conditions.append(False)
        
        if logic_op == "or":
            return any(conditions), exceeded_indices
        elif logic_op == "and":
            return all(conditions), exceeded_indices
        else:
            raise ValueError("Invalid logic operator. Use 'or' or 'and'.")
    
    return check_condition

# Function to collect the initial inputs from the user
def collect_initial_inputs():
    Vspeed_list = []
    thresholds_list = []
    selected_forces_list = []
    logic_op_list = [] 

    print("Enter the initial data inputs.")
    
    while True:
        # Get the speed vector with validation
        while True:
            try:
                Vspeed_input = input("Enter the speed vector values in the base frame (m/s) (6 values separated by commas): ")
                Vspeed_input = [float(value.strip()) for value in Vspeed_input.split(',')]
                if len(Vspeed_input) == 6:
                    Vspeed_list.append(Vspeed_input)
                    break
                else:
                    print("Please enter exactly 6 values.")
            except ValueError:
                print("Invalid input. Please enter 6 numeric values separated by commas.")
        
        # Get force thresholds and selections
        force_indices = {'fx': 0, 'fy': 1, 'fz': 2, 'mx': 3, 'my': 4, 'mz': 5}
        selected_forces = []
        thresholds = []
        print("Available forces: Fx, Fy, Fz, Mx, My, Mz")
        forces = input("Enter the forces you want to monitor (separated by commas): ").lower().split(',')
        for force in forces:
            force = force.strip()
            if force in force_indices:
                selected_forces.append(force_indices[force])
                while True:
                    try:
                        threshold = float(input(f"Enter the threshold for {force} ({force_units}/{moment_units}): "))  # Adjust the unit display as needed
                        thresholds.append(threshold)
                        break
                    except ValueError:
                        print("Invalid input. Please enter a numeric value for the threshold.")
            else:
                print(f"Invalid force: {force}")

        selected_forces_list.append(selected_forces)
        thresholds_list.append(thresholds)

        # Get the logic operator with validation
        while True:
            logic_op = input("Enter the logic operator ('or' or 'and'): ").strip().lower()
            if logic_op in ['or', 'and']:
                logic_op_list.append(logic_op)
                break
            else:
                print("Invalid input. Please enter 'or' or 'and'.")

        # Check if more inputs are needed
        more_inputs = input("Do you want to add another set of inputs? (yes/no): ").strip().lower()
        if more_inputs != 'yes':
            break

    return Vspeed_list, selected_forces_list, thresholds_list, logic_op_list

# Collect all inputs at the start
Vspeed_list, selected_forces_list, thresholds_list, logic_op_list = collect_initial_inputs()
# Initialize the first set of inputs
current_index = 0
Vspeed_input = Vspeed_list[current_index]
selected_forces = selected_forces_list[current_index]
thresholds = thresholds_list[current_index]
logic_ops= logic_op_list[current_index]
check_limits = check_force_limits(selected_forces, thresholds, logic_ops)
Vspeed1 = list_to_Vspeed(Vspeed, Vspeed_input)
con.send(Vspeed1)

# Démarrer la synchronisation des données
if not con.send_start():
    sys.exit()


#Writing to the file
with open(output_file_path, 'w') as f:
    # Write the settings to the file
    f.write(f"packet size: {packet_size}bytes\n")
    f.write(f"number of packets: {nbr_data_pooled/8}\n")
    f.write(f"signal frequency: {signal_frequency}\n")
    f.write(f"sample frequency: {1/sample_rate}\n")
    f.write(f"force units: {force_units}, moment units: {moment_units}\n")

amti_dll.fmBroadcastResetSoftware()     #Apply the settings
time.sleep(0.5)  # Sleep for at least 250 milliseconds, Wait for the settings to take effect
# Zero the platform
amti_dll.fmBroadcastZero()
time.sleep(0.5)  # Wait for the zero command to take effect
print("Platform zeroed.")

# Start data acquisition
print("Starting data acquisition...")
amti_dll.fmBroadcastStart()
time.sleep(0.91)  # Wait for the start command to take effect
print("Data acquisition started.")
# Fonctions utilitaires pour la conversion entre les listes et les points de consigne
def Vspeed_to_list(Vspeed):
    return [Vspeed.__dict__[f"input_double_register_{i}"] for i in range(6)]

def list_to_Vspeed(Vspeed, lst):
    for i in range(6):
        Vspeed.__dict__[f"input_double_register_{i}"] = lst[i]
    return Vspeed

loop_frequencies = []
t0 = time.time()
Fx, Fy, Fz = [], [], []
Mx, My, Mz = [], [], []


"""-----------------------------main loop-----------------------------------"""
try:
    print("Collecting data. Press Ctrl+C to stop.")
    while True:
        try:
            
            loop_t0 = time.time()
            result = amti_dll.fmDLLTransferFloatData(ctypes.byref(data_pointer))
            if result == 1:
                data = ctypes.cast(data_pointer, ctypes.POINTER(ctypes.c_float *nbr_data_pooled)).contents

                data_values.append([data[i] for i in range(nbr_data_pooled) if i % 8 == 0])  # Store data, here using the first element for simplicity
                # Extract the individual components in the transducer frame
                counter = [data[i] for i in range(nbr_data_pooled) if i % 8 == 0]
                Fx = [data[i] for i in range(nbr_data_pooled) if i % 8 == 1]
                Fy = [data[i] for i in range(nbr_data_pooled) if i % 8 == 2]
                Fz = [data[i] for i in range(nbr_data_pooled) if i % 8 == 3]
                Mx = [data[i] for i in range(nbr_data_pooled) if i % 8 == 4]
                My = [data[i] for i in range(nbr_data_pooled) if i % 8 == 5]
                Mz = [data[i] for i in range(nbr_data_pooled) if i % 8 == 6]
                # Combine force components into an array of vectors
                forces = np.array(list(zip(Fx, Fy, Fz)))
                torques = np.array(list(zip(Mx, My, Mz)))
                # Apply rotation transformation to force vectors
                rotated_forces = rotation(forces)
                rotated_moments = rotation(torques)
                # Extract the individual components
                # the force and moment values are in the tool frame
                Fx = rotated_forces[:, 0].tolist()
                Fy = rotated_forces[:, 1].tolist()
                Fz = rotated_forces[:, 2].tolist()
                Mx = rotated_moments[:, 0].tolist()
                My = rotated_moments[:, 1].tolist()
                Mz = rotated_moments[:, 2].tolist()
                #write into the txt file
                with open(output_file_path, 'a') as f:
                    for i in range(len(counter)):
                        f.write(f"Counter: {counter[i]:<10}, ")
                        f.write(f"Fx: {round(Fx[i], 2):<10},")
                        f.write(f"Fy: {round(Fy[i], 2):<10}, ")
                        f.write(f"Fz: {round(Fz[i], 2):<10}, ")
                        f.write(f"Mx: {round(Mx[i], 4):<10},")
                        f.write(f"My: {round(My[i], 4):<10},")
                        f.write(f"Mz: {round(Mz[i], 4):<10},\n")

                # Add the data to the lists
                counter_values.extend(counter)
                Fx_values.extend(Fx)
                Fy_values.extend(Fy)
                Fz_values.extend(Fz)
                Mx_values.extend(Mx)
                My_values.extend(My)
                Mz_values.extend(Mz)


            else:
                # print("No new data available")
                data_values.append(None)  # Add a placeholder for no data

            time.sleep(sample_rate)  # Adjust the sleep time as necessary
            loop_t1= time.time()
            loop_duration = loop_t1 - loop_t0
            loop_frequency = 1 / loop_duration # Calculate the loop frequency
            loop_frequencies.append(loop_frequency)
            

            # """-------------------------------control loop--------------------------------"""
            state = con.receive()
            if state is None:
                break
            servoing.input_int_register_0  = 1
            con.send(servoing)
            if state.output_int_register_0 != 0:
                # print("Vspeed", Vspeed_to_list(Vspeed1))
                con.send(Vspeed1)
                

            # Check force limits
            force_data_lists = [Fx, Fy, Fz, Mx, My, Mz]
            limits_exceeded, exceeded_indices = check_limits([force_data_lists[i] for i in selected_forces])
            if limits_exceeded:
                print("Force limits exceeded.")
                servoing.input_int_register_0 = 0
                con.send(servoing)
                time.sleep(2)  # Wait for the robot to stop
                force_names = ['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']
                for index in exceeded_indices:
                    if thresholds[index] >= 0:
                        print(f"{force_names[selected_forces[index]]} has exceeded its threshold with a value of {max(force_data_lists[selected_forces[index]])} ({force_units}/{moment_units})")
                    else:
                        print(f"{force_names[selected_forces[index]]} has exceeded its threshold with a value of {min(force_data_lists[selected_forces[index]])} ({force_units}/{moment_units})")
                    continue
                # Move to the next set of inputs
                current_index += 1
                #initialize the force and moment values
                Fx, Fy, Fz = [], [], []
                Mx, My, Mz = [], [], []
                force_data_lists = [Fx, Fy, Fz, Mx, My, Mz]
                if current_index < len(Vspeed_list):
                    Vspeed_input = Vspeed_list[current_index]
                    selected_forces = selected_forces_list[current_index]
                    thresholds = thresholds_list[current_index]
                    logic_ops = logic_op_list[current_index]
                    check_limits = check_force_limits(selected_forces, thresholds, logic_ops)
                    Vspeed1 = list_to_Vspeed(Vspeed, Vspeed_input)
                    con.send(Vspeed1)
                    servoing.input_int_register_0 = 1
                    con.send(servoing)
                else:
                    print("All input sets have been used. Stopping.")
                    break

            # Additional processing...
        except Exception as e:
            print(f"An error occurred: {e}")
            continue
except KeyboardInterrupt:
    print("Data collection stopped by user.")
finally:
    con.send_pause()
    con.disconnect()

# Calculate the median loop frequency
def calculate_median(values):
    sorted_values = sorted(values)
    n = len(sorted_values)
    if n % 2 == 1:
        # If odd, return the middle element
        return sorted_values[n // 2]
    else:
        # If even, return the average of the two middle elements
        return (sorted_values[n // 2 - 1] + sorted_values[n // 2]) / 2.0

median_loop_frequency = calculate_median(loop_frequencies)
print(f"Median loop frequency: {median_loop_frequency:.2f} Hz")
#Median loop frequency: 791.08 Hz
t1 = time.time()
# Stop data acquisition
print("Stopping data acquisition...")
amti_dll.fmBroadcastStop()
print("Data acquisition stopped.")


# Create a figure and a set of subplots
fig, axs = plt.subplots(2, 1, figsize=(12, 12))

# Plot for Force Components
axs[0].plot(Fx_values, label='Fx', marker='o', markersize=1.5, linewidth=1)
axs[0].plot(Fy_values, label='Fy', marker='x', markersize=1.5, linewidth=1)
axs[0].plot(Fz_values, label='Fz', marker='^', markersize=1.5, linewidth=1)
axs[0].set_xlabel('Sample Number')
axs[0].set_ylabel(f'Force Value {force_units}')
axs[0].set_title('Force Components Fx, Fy, Fz')
axs[0].legend()
axs[0].grid(True)

# Plot for Moment Components
axs[1].plot(Mx_values, label='Mx', marker='o', markersize=1.5, linewidth=1)
axs[1].plot(My_values, label='My', marker='x', markersize=1.5, linewidth=1)
axs[1].plot(Mz_values, label='Mz', marker='^', markersize=1.5, linewidth=1)
axs[1].set_xlabel('Sample Number')
axs[1].set_ylabel(f'Moment Value {moment_units}')
axs[1].set_title('Moment Components Mx, My, Mz')
axs[1].legend()
axs[1].grid(True)

# Save the figure containing both plots
plt.tight_layout()  # Adjust the layout to make room for all subplots
output_FT_path = os.path.join(output_path, "force_moment_graphe.png")
plt.savefig(output_FT_path)
plt.show()





