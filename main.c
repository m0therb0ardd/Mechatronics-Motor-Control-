#include "nu32dip.h"          // config bits, constants, funcs for startup and UART
#include "encoder.h"
#include "utilities.h"
#include "ina219.h"
#include "xc.h" //defines all special function registers


// include other header files here

//NOTES: the role of this function is to cal any functions initializing 
//peripherals or modules and to enter an infinite loop, dispatching commands that the PIC32 gets from the client

#define BUF_SIZE 200
#define INA219_REG_CURRENT 0x04  

int main()
{
  char buffer[BUF_SIZE];
  NU32DIP_Startup(); // cache on, min flash wait, interrupts on, LED/button init, UART init
  INA219_Startup(); //initialize my INA219 sensor 
  set_mode(IDLE); //start in idle mode 
  UART2_Startup(); //told to call UAR2_Startup
  NU32DIP_YELLOW = 1;  // turn off the LEDs
  NU32DIP_GREEN = 1;
  __builtin_disable_interrupts();
  // in future, initialize modules or peripherals here

  __builtin_enable_interrupts();

  while(1)
  {
    NU32DIP_ReadUART1(buffer,BUF_SIZE); // we expect the next character to be a menu command
    NU32DIP_GREEN = 1;                   // clear the error LED this is LED 2
    switch (buffer[0]) {

    case 'a':  // Read current sensor (ADC counts)
    {
        float adc_counts = readINA219(INA219_REG_CURRENT); //reads raw ADC count
        sprintf(buffer, "%d\r\n", adc_counts);  // Format ADC count as a string
        NU32DIP_WriteUART1(buffer);  // Send the ADC count to the client
        break;
    }

    case 'b':  // Read current sensor (mA milliamps)
    {
        float current_mA = INA219_read_current(); //reads current in mA
        sprintf(buffer, "%d\r\n", current_mA);  // Format current count as a string
        NU32DIP_WriteUART1(buffer);  // Send the current to the client
        break;
    }

    case 'c':  // Command to read encoder counts
    {
        // Request encoder count
        WriteUART2("a");  // Send request to encoder over UART2

        // Wait for encoder response
        while (!get_encoder_flag()) {}  // Poll the flag
        set_encoder_flag(0);  // Clear the flag

        // Read encoder count
        int p = get_encoder_count();  // Get the encoder count
        char m[50];
        sprintf(m, "%d\r\n", p);  // Format the count as a string
        NU32DIP_WriteUART1(m);  // Send the count to the client
        break;
    }

    case 'e':  // Command to reset encoder
    {
        WriteUART2("b");  // Send reset command to encoder over UART2
        sprintf(buffer, "Encoder reset\r\n");  // Format the confirmation message
        NU32DIP_WriteUART1(buffer);  // Send the confirmation to the client
        break;
    }

    case 'd':  // Command to read encoder angle in degrees
    {
        WriteUART2("a");  // Send request to encoder over UART2
        while (!get_encoder_flag()) {}  // Wait for encoder response
        set_encoder_flag(0);  // Clear the flag

        // Calculate the angle in degrees
        int counts = get_encoder_count();  // Read encoder counts
        float angle = (counts / (334.0 * 4.0)) * 360.0;  // Calculate angle in degrees
        sprintf(buffer, "%.2f\r\n", angle);  // Format the angle as a string
        NU32DIP_WriteUART1(buffer);  // Send the angle to the client
        break;
    }

    case 'r':  // Command to get current mode 
    {
        OperatingMode mode = get_mode(); //get current mode
        switch (mode) {
          case IDLE:
            sprintf(buffer, "IDLE\r\n");
            break;
          case PWM:
              sprintf(buffer, "PWM\r\n");
              break;
          case ITEST:
              sprintf(buffer, "ITEST\r\n");
              break;
          case HOLD:
              sprintf(buffer, "HOLD\r\n");
              break;
          case TRACK:
              sprintf(buffer, "TRACK\r\n");
              break;
          default:
              sprintf(buffer, "UNKNOWN\r\n");
              break; 
        }
        NU32DIP_WriteUART1(buffer);  // Send the MODE to the client
        break;
    }


    case 'q':  // Quit command
    {
        set_mode(IDLE);  // Set the mode to IDLE
        sprintf(buffer, "Exiting and setting mode to IDLE\r\n");
        NU32DIP_WriteUART1(buffer);  // Send the confirmation message
        break;
    }
      default:
        break;
    }
  }
  return 0;
}


                                                                                                                                                                                                                                  