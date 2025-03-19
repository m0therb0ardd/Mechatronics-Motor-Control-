import matplotlib.pyplot as plt 
from statistics import mean 
from genref import genRef
import serial

def read_plot_matrix():
    n_str = ser.read_until(b'\n');  # get the number of data points to receive
    n_int = int(n_str) # turn it into an int
    print('Data length = ' + str(n_int))
    ref = []
    data = []
    data_received = 0
    while data_received < n_int:
        dat_str = ser.read_until(b'\n');  # get the data as a string, ints seperated by spaces
        dat_int = list(map(float,dat_str.split())) # now the data is a list //THIS WAS CHANGED TO FLOAT BY PUSHKAR 
        ref.append(dat_int[0])
        data.append(dat_int[1])
        data_received = data_received + 1
    meanzip = zip(ref,data)
    meanlist = []
    for i,j in meanzip:
        meanlist.append(abs(i-j))
    score = mean(meanlist)
    t = range(len(ref)) # index array
    plt.plot(t,ref,'r*-',t,data,'b*-')
    plt.title('Score = ' + str(score))
    plt.ylabel('value')
    plt.xlabel('index')
    plt.show()


import serial
ser = serial.Serial('/dev/ttyUSB0',230400)
print('Opening port: ')
print(ser.name)

has_quit = False

import serial
ser = serial.Serial('/dev/ttyUSB0',230400)
print('Opening port: ')
print(ser.name)

has_quit = False


# menu loop
while not has_quit:
    print('PIC32 MOTOR DRIVER INTERFACE')
    # display the menu options; this list will grow
    print(' \tm: Load step trajectory  \tn: Load cubic trajectory (FEED FORWARD)  \to: Execute trajectory \tc: Read encoder(counts) \tf: Set PWM \tp: Unpower motor  \te: Reset Encoder \td: Read encoder(deg)  \ta: Read current sensor(ADC counts) \tb:  Read current sensor(mA)  \tg: Set Current Gains \th: Get current gains \tk: Test current control \tr: Get mode \ti: Set pos gains  \tj: get pos gains  \tq: Quit')  # '\t' is a tab    # read the user's choice


    selection = input('\nENTER COMMAND: ')
    selection_endline = selection+'\n'
    # send the command to the PIC32
    ser.write(selection_endline.encode()); # .encode() turns the string into a char array


    if selection == 'c':  # Read encoder counts
            #ser.write(b'c\n')  # Send the 'c' command
            counts_str = ser.read_until(b'\n').decode().strip()  # Read the response
            counts = int(counts_str)  # Convert to integer
            print(f'Encoder counts: {counts}\n')

    elif selection == 'a':  # Read current sensor (ADC counts)
        adc_str = ser.read_until(b'\n') # Read the response
        adc_counts = float(adc_str)  # Convert to float
        print(f'Current (ADC counts): {adc_counts}\n')  # Print the ADC counts

    elif selection == 'b':  # Read current sensor (mA)
        # ser.write(b'b\n')  # Send the 'b' command
        current_str = ser.read_until(b'\n').decode().strip()  # Read the response
        current_mA = float(current_str)  # Convert to float
        print(f'Current: {current_mA} mA\n')  # Print the current in mA

    elif selection == 'e':  # Reset encoder counts
        # ser.write(b'e\n')  # Send the 'e' command
        response = ser.read_until(b'\n').decode().strip()  # Read the response
        print(f'{response}\n')  # Print the confirmation message
        
    elif selection == 'd':  # Read encoder in degrees
        # ser.write(b'd\n')  # Send the 'e' command
        angle_str = ser.read_until(b'\n').decode().strip()  # Read the response
        angle = float(angle_str)  # Convert to float
        print(f'Encoder angle: {angle} degrees\n')

    elif selection == 'r':  # Get current mode
        # ser.write(b'r\n')  # Send the 'r' command
        mode_str = ser.read_until(b'\n').decode().strip()  # Read the response
        print(f'Current mode: {mode_str}\n')  # Print the mode

    elif selection == 'l':  # Go to Angle (deg)
        angle = input('Enter desired angle (degrees): ')  # Prompt user for angle
        ser.write(f"{angle}\n".encode())  # Send the angle as a string to the PIC32
        print(f"Angle command sent: {angle} degrees\n")  # Confirmation message

        
    elif (selection == 'q'):
        # ser.write(b'q\n')  # Send the 'q' command
        response = ser.read_until(b'\n').decode().strip()  # Read the response
        print(f'{response}\n')  # Print the confirmation message
        print('Exiting client')
        has_quit = True  # Exit client
        ser.close()  # Close the serial port
        
    # elif selection == 'f':  # Set PWM
    #     pwm_value = float(input('Enter PWM value (-100 to 100):'))  # Ask for PWM value
    #     #ser.write(pwm_value.encode() + b'\n')  # Send the PWM value to the PIC32
    #     ser.write(f"{pwm_value}\n".encode())  # Send the PWM value to the PIC32

    elif selection == 'f':  # Set PWM
        pwm_input = input("Enter PWM value (-100 to 100): ")  # Ask user for PWM value
        pwm_int = int(pwm_input)  # Convert to integer
        print(f"PWM value set to {pwm_int}\n")  # Print confirmation
        ser.write((str(pwm_int) + '\n').encode())  # Send the PWM value to the PIC32


    elif selection == 'g':  # Set current gains
        kp = float(input("Enter Kp for current control: "))
        ki = float(input("Enter Ki for current control: ")) 
        ser.write(f"{kp} {ki}\n".encode())  # Send command with values 
        # response = ser.read_until(b'\n').decode().strip()  # Read confirmation
        # print(f"Response: {response}\n")

    elif selection == 'h':  # Get current gains
        #ser.write(b"h\n")  # Send command to request current gains
        response = ser.read_until(b'\n').decode().strip()  # Read the response
        print(f"Current Gains: {response}\n")



    elif selection == 'i':  # Set position gains
            kp = float(input("Enter Kp for position control: "))
            ki = float(input("Enter Ki for position control: ")) 
            kd = float(input("Enter Ki for position control: ")) 
            ser.write(f"{kp} {ki} {kd}\n".encode())  # Send command with values 
            # response = ser.read_until(b'\n').decode().strip()  # Read confirmation
            # print(f"Response: {response}\n")

    elif selection == 'j':  # Get position gains
        response = ser.read_until(b'\n').decode().strip()  # Read the response
        print(f"Position Gains: {response}\n")

    elif selection == 'k':  # Test Current Control
        print("Starting ITEST Mode...")
        read_plot_matrix()  # Retrieve and plot results NEEEDSSS INTSSSS


    elif selection == 'p':  # Unpower motor
        response = ser.read_until(b'\n').decode().strip()  # Read the response
        print(f'{response}\n')  # Print the confirmation message

    elif (selection == 'm'):
            ref, accel_ref, a3, a2, a1 = genRef('step')
            size = len(ref)
            print(size)
            t = range(len(ref))
            plt.plot(t, ref, 'r*-')
            plt.ylabel('value')
            plt.xlabel('index')
            plt.show()
            ser.write(f"{size}\n".encode())
            for val in ref:
                ser.write(f"{val}\n".encode())


    # elif (selection == 'n'):
    #         ref, accel_ref, a3, a2, a1 = genRef('cubic')
    #         size = len(ref)
    #         print(size)
    #         t = range(len(ref))
    #         plt.plot(t, ref, 'r*-')
    #         plt.ylabel('value')
    #         plt.xlabel('index')
    #         plt.show()
    #         ser.write(f"{size}\n".encode())
    #         for val in ref:
    #             ser.write(f"{val}\n".encode())

    elif (selection == 'n'):
        ref, accel_ref, a3, a2, a1 = genRef('cubic')  
        size = len(ref)
        print(size)
        t = range(len(ref))
        plt.plot(t, ref, 'r*-')
        plt.ylabel('value')
        plt.xlabel('index')
        plt.show()
        ser.write(f"{size}\n".encode())

        # Compute acceleration from cubic polynomial
        ref_vel = [3*a3*t**2 + 2*a2*t + a1 for t in range(size)]
        ref_accel = [6*a3*t + 2*a2 for t in range(size)]

        for i in range(size):
            ser.write(f"{ref[i]} {ref_accel[i]}\n".encode())  # Send position & acceleration


 
    elif (selection == 'o'):
        print(f"Executing Trajectory!\n")
        # ser.write(b"o\n")  # Tell PIC32 to execute trajectory
        

        # Wait for the trajectory size
        while True:
            n_str = ser.read_until(b'\n').decode().strip()
            if n_str.isdigit():  # Ensure it's an actual number
                n_int = int(n_str)
                break  # Exit loop once a valid number is found
            print(f"Skipping unexpected message: {n_str}")  # Debug print for unexpected messages

        ref = []
        data = []
        for _ in range(n_int):
            dat_str = ser.read_until(b'\n').decode().strip()
            try:
                dat_int = list(map(float, dat_str.split()))
                if len(dat_int) == 2:  # Ensure there are exactly 2 values (reference and actual)
                    ref.append(dat_int[0])
                    data.append(dat_int[1])
                else:
                    print(f"Skipping invalid data: {dat_str}")  # Skip invalid data
            except ValueError:
                print(f"Skipping invalid data: {dat_str}")  # Skip invalid data

        plt.plot(range(len(ref)), ref, 'r*-', label="Reference")
        plt.plot(range(len(data)), data, 'b*-', label="Actual")
        plt.legend()
        plt.xlabel("Time Step")
        plt.ylabel("Angle (deg)")
        plt.title("Trajectory Tracking")
        plt.show()


    else:
        print('Invalid Selection ' + selection_endline) 

