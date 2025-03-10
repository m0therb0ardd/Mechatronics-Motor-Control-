import serial
ser = serial.Serial('/dev/ttyUSB0',230400)
print('Opening port: ')
print(ser.name)

has_quit = False
# menu loop
while not has_quit:
    print('PIC32 MOTOR DRIVER INTERFACE')
    # display the menu options; this list will grow
    print('\tc: Read encoder(counts) \te: Reset Encoder \td: Read encoder(deg)  \ta: Read current sensor(ADC counts) \tb:  Read current sensor(mA)  \tr: Get mode \tq: Quit')  # '\t' is a tab    # read the user's choice
    selection = input('\nENTER COMMAND: ')
    selection_endline = selection+'\n'
    # send the command to the PIC32
    ser.write(selection_endline.encode()); # .encode() turns the string into a char array


    if selection == 'c':  # Read encoder counts
            ser.write(b'c\n')  # Send the 'c' command
            counts_str = ser.read_until(b'\n').decode().strip()  # Read the response
            counts = int(counts_str)  # Convert to integer
            print(f'Encoder counts: {counts}\n')

    elif selection == 'a':  # Read current sensor (ADC counts)
        # ser.write(b'a\n')  # Send the 'a' command
        adc_str = ser.read_until(b'\n').decode().strip()  # Read the response
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
        
    elif (selection == 'q'):
        # ser.write(b'q\n')  # Send the 'q' command
        response = ser.read_until(b'\n').decode().strip()  # Read the response
        print(f'{response}\n')  # Print the confirmation message
        print('Exiting client')
        has_quit = True  # Exit client
        ser.close()  # Close the serial port


    else:
        print('Invalid Selection ' + selection_endline)