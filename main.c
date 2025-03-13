#include "nu32dip.h" // config bits, constants, funcs for startup and UART
#include "encoder.h"
#include "utilities.h"
#include "ina219.h"
#include "xc.h" //defines all special function registers

// include other header files here

// NOTES: the role of this function is to cal any functions initializing
// peripherals or modules and to enter an infinite loop, dispatching commands that the PIC32 gets from the client

#define BUF_SIZE 200
#define INA219_REG_CURRENT 0x04

// PWM and Timer Config
#define PWM_PERIOD (2400 - 1) // PWM period in ticks --> Calculates the number of timer ticks required for one PWM period based on the system clock frequency (NU32DIP_SYS_FREQ)
#define PWM_DUTY_MAX 100      // Maximum PWM duty cycle (100%)
#define PWM_DUTY_MIN -100     // Minimum PWM duty cycle (-100%)

// Function prototypes
void timer2_init(void); // Initializes Timer2 for the 5 kHz interrupt service routine (ISR).
void pwm_init(void);    // Initializes the PWM module for a 20 kHz signal.
// void __ISR(_TIMER_2_VECTOR, IPL5SOFT) timer2_isr(void);

// Global variables
volatile int pwm_duty = 0;        // PWM duty cycle (-100 to 100)
volatile int motor_direction = 0; // Motor direction (0 = brake, 1 = forward, -1 = reverse)

// SECTION 28.4.10: PI CURRENT CONTROL AND ITEST
static volatile float Kp_mA = 0.03, Ki_mA = 0.05; // Default current control gains --> needs to be accessible in timer2 for ISR control
static float refCurrent[100];                     // stores desired current values in ITEST mode
static float actCurrent[100];                     // stores actual current as measured by INA219 sensor

// pwm_init 1. configures timer3 for 20kHz PWM signal
// 2. maps rb15 as pwm output
// enables output compare oc1
// starts woth OC1RS = 0 so motor is off
void pwm_init(void)
{
    // first we need to set pwm period which is timer 3
    PR3 = 2399;                     // this is how frequently timer3 resets
    __builtin_disable_interrupts(); // disable interrupts while configuring --> pasue all alarms while configuring

    // now we need to configure timer3 which is my pwm timer
    T3CONbits.TCKPS = 0; // prescaler set to 1 so we are not slowing anything down
    TMR3 = 0;            // reset timer counter
    T3CONbits.ON = 1;    // turn on timer3

    // next configure OC1 which is the output compare for pwm
    OC1CONbits.OCTSEL = 1;  // use timer 3 as the clock source for OC1 (output comapre timer select)
    OC1CONbits.OCM = 0b110; // pwm mode without fault pin
    OC1RS = 0;              // start with 0% duty cycle //when timer3 matches PWM output turns off
    OC1R = 0;               // init before turning OC1 //

    // now set up pwm pin for pic rb15
    TRISBbits.TRISB15 = 0;      // Set RB15 as output (Needed for PWM)
    RPB15Rbits.RPB15R = 0b0101; // maps OC1 to RB15 (output pwm signal)

    // now enable OC1 which is my pwm output
    OC1CONbits.ON = 1;

    // motor direction this is an output
    TRISBbits.TRISB2 = 0; // **Set RB2 as output**

    __builtin_enable_interrupts(); // re enable interrupts
}

// bens idea
void set_pwm(int pwm)
{
    char send_buffer[50];

    if (pwm > 0)
    {
        sprintf(send_buffer, "%d\n\r", pwm); // Send data length (100 samples)
        NU32DIP_WriteUART1(send_buffer);
        OC1RS = (pwm * PR3) / 100;  // Convert % to OC1RS value
        LATBbits.LATB2 = 0;         // Forward direction
    }
    else if (pwm < 0)
    {
        OC1RS = (-pwm * PR3) / 100; // Convert % to OC1RS value
        LATBbits.LATB2 = 1;         // Reverse direction
    }
}


// timer2_init configures timer2 for a 5khz interrupt
// sets up an interrupt that will call timer2 isr
// clears and enables
// starts timer 2 and begins sending interrupts
void timer2_init(void)
{
    __builtin_disable_interrupts(); // Disable interrupts while configuring

    // Step 1: Set Timer2 period for 5 kHz interrupt
    PR2 = 9600 - 1; // Sets the period for a 5 kHz interrupt (200 µs interval)

    // Step 2: Configure Timer2
    T2CONbits.TCKPS = 0b100; // Prescaler N=16 (slows the clock down)
    TMR2 = 0;                // Reset Timer2 counter

    // Step 3: Configure Interrupt for Timer2
    IPC2bits.T2IP = 5; // Interrupt priority = 5
    IPC2bits.T2IS = 0; // Sub-priority = 0
    IFS0bits.T2IF = 0; // Clear Timer2 interrupt flag
    IEC0bits.T2IE = 1; // Enable Timer2 interrupt

    // Step 4: Start Timer2
    T2CONbits.ON = 1; // Turn on Timer2

    __builtin_enable_interrupts(); // Re-enable interrupts
}

void __ISR(_TIMER_2_VECTOR, IPL5SOFT) timer2_isr(void)
{
    char buffer[100]; // Declare local buffer

    // adding ITEST LOGIC
    static int count = 0;             // counter for 100 samples
    static float integral_term = 0.0; // Integral term for PI control
    static float ref_current = 200.0; // Initial reference current (mA)

    switch (get_mode())
    {
    case IDLE:
        OC1RS = 0;          // Stop PWM (Brake mode)
        LATAbits.LATA0 = 0; // Motor OFF
        break;

    case PWM:
        // if (pwm_duty > 0)
        // {
        //     OC1RS = (pwm_duty * PR3) / 100; // Convert % to OC1RS value
        //     LATAbits.LATA0 = 0;             // Forward direction
        // }
        // else if (pwm_duty < 0)
        // {
        //     OC1RS = (-pwm_duty * PR3) / 100; // Convert % to OC1RS value
        //     LATAbits.LATA0 = 1;              // Reverse direction
        // }
        // else
        // {
        //     OC1RS = 0; // Stop motor
        //     LATAbits.LATA0 = 0;
        // }

        // sprintf(buffer, "AFTER ISR: pwm_duty = %d, OC1RS = %d, LATA0 = %d\r\n",
        //         pwm_duty, OC1RS, LATAbits.LATA0);
        // NU32DIP_WriteUART1(buffer);
        break;

    // case ITEST:
    //     // 🔹 Ensure reference current follows a strict square wave pattern
    //     if (count < 25)
    //     {
    //         ref_current = 200.0; // First 25 samples: +200 mA
    //     }
    //     else if (count < 50)
    //     {
    //         ref_current = -200.0; // Next 25 samples: -200 mA
    //     }
    //     else if (count < 75)
    //     {
    //         ref_current = 200.0; // Next 25 samples: +200 mA
    //     }
    //     else if (count < 100)
    //     {
    //         ref_current = -200.0; // Last 25 samples: -200 mA
    //     }

    //     // ref_current = 200.0;

    //     // 🔹 Read actual current
    //     float actual_current = INA219_read_current();

    //     // 🔹 Compute Error
    //     float error = ref_current - actual_current;

    //     // 🔹 Accumulate Integral Term
    //     static float eint = 0.0;
    //     eint += error;

    //     // 🔹 Compute PI Controller Output
    //     float control_signal = Kp_mA * error + Ki_mA * eint;

    //     // 🔹 Limit PWM Output
    //     if (control_signal > 100)
    //         control_signal = 100;
    //     if (control_signal < -100)
    //         control_signal = -100;
    //     pwm_duty = (int)control_signal;

    //     // 🔹 Apply PWM Output
    //     // 🔹 Apply PWM Output with Proper Direction Handling
    //     if (pwm_duty > 0)
    //     {
    //         OC1RS = (unsigned int)(pwm_duty / 100.0 * PR3); // Set PWM duty cycle
    //         //LATAbits.LATA0 = 0;                             // Forward direction
    //         LATBbits.LATB2 = 0; 
    //     }
    //     else if (pwm_duty < 0)
    //     {
    //         OC1RS = (unsigned int)(-pwm_duty / 100.0 * PR3); // Convert to positive duty cycle
    //         //LATAbits.LATA0 = 1;                              // Reverse direction
    //         LATBbits.LATB2 = 0; 
    //     }
    //     else
    //     {
    //         OC1RS = 0; // Stop motor if PWM is zero
    //     }

    //     // 🔹 Store Data for Plotting
    //     refCurrent[count] = ref_current;
    //     actCurrent[count] = actual_current;

    //     count++; // Increment counter

    //     // 🔹 Stop ITEST Mode After 100 Samples
    //     if (count >= 100)
    //     {
    //         set_mode(IDLE); // Stop ITEST mode
    //         count = 0;      // Reset counter
    //         eint = 0.0;     // Reset integral term
    //     }
    //     break;

    case ITEST:
        // 🔹 Ensure reference current follows a strict square wave pattern
        if (count < 25) {
            ref_current = 200.0;  // First 25 samples: +200 mA
        } else if (count < 50) {
            ref_current = -200.0; // Next 25 samples: -200 mA
        } else if (count < 75) {
            ref_current = 200.0;  // Next 25 samples: +200 mA
        } else if (count < 100) {
            ref_current = -200.0; // Last 25 samples: -200 mA
        }

        // 🔹 Read actual current
        float actual_current = INA219_read_current();

        // 🔹 Compute PI Controller Output
        float error = ref_current - actual_current;
        static float eint = 0.0;
        eint += error;  // Accumulate integral term

        float control_signal = Kp_mA * error + Ki_mA * eint;

        // 🔹 Limit PWM Output
        if (control_signal > 100) control_signal = 100;
        if (control_signal < -100) control_signal = -100;
        pwm_duty = (int)control_signal;

        // 🔹 Apply PWM Output with Proper Direction Handling
        if (pwm_duty > 0) {
            OC1RS = (unsigned int)(pwm_duty / 100.0 * PR3); // Set PWM duty cycle
            LATBbits.LATB2 = 0;  // ✅ Forward direction
        } else if (pwm_duty < 0) {
            OC1RS = (unsigned int)(-pwm_duty / 100.0 * PR3); // Convert to positive duty cycle
            LATBbits.LATB2 = 1;  // ✅ Reverse direction
        } else {
            OC1RS = 0; // Stop motor if PWM is zero
        }

        // 🔹 Store Data for Plotting
        refCurrent[count] = ref_current;
        actCurrent[count] = actual_current;

        count++; // ✅ Only increment count when ITEST is running

        // 🔹 Stop ITEST Mode After 100 Samples
        if (count >= 100) {  
            set_mode(IDLE);  // ✅ Stop ITEST mode only after 100 samples
            count = 0;  // ✅ Reset count properly
            eint = 0.0;  // ✅ Reset integral term
        }
        break;

    default:
        break;
    }

    IFS0bits.T2IF = 0; // Clear Timer 2 interrupt flag
}

int main()
{
    char buffer[BUF_SIZE];
    NU32DIP_Startup();  // cache on, min flash wait, interrupts on, LED/button init, UART init
    INA219_Startup();   // initialize my INA219 sensor
    set_mode(IDLE);     // start in idle mode
    UART2_Startup();    // told to call UAR2_Startup
    timer2_init();      // Initialize Timer2 for 5 kHz ISR
    pwm_init();         // Initialize PWM for 20 kHz signal
    NU32DIP_YELLOW = 1; // turn off the LEDs
    NU32DIP_GREEN = 1;
    __builtin_disable_interrupts();
    // in future, initialize modules or peripherals here

    __builtin_enable_interrupts();

    sprintf(buffer, "Timer3 ON: %d, PR3: %d, TMR3: %d, OC1RS: %d\r\n",
            T3CONbits.ON, PR3, TMR3, OC1RS);
    NU32DIP_WriteUART1(buffer);

    while (1)
    {
        NU32DIP_ReadUART1(buffer, BUF_SIZE); // we expect the next character to be a menu command
        NU32DIP_GREEN = 1;                   // clear the error LED this is LED 2
        switch (buffer[0])
        {

        case 'a': // Read current sensor (ADC counts)
        {
            float adc_counts = readINA219(INA219_REG_CURRENT); // reads raw ADC count
            sprintf(buffer, "%f\r\n", adc_counts);             // Format ADC count as a string
            NU32DIP_WriteUART1(buffer);                        // Send the ADC count to the client
            break;
        }

        case 'b': // Read current sensor (mA milliamps)
        {
            float current_mA = INA219_read_current(); // reads current in mA
            sprintf(buffer, "%f\r\n", current_mA);    // Format current count as a string
            NU32DIP_WriteUART1(buffer);               // Send the current to the client

            break;
        }

        case 'c': // Command to read encoder counts
        {
            // Request encoder count
            WriteUART2("a"); // Send request to encoder over UART2

            // Wait for encoder response
            while (!get_encoder_flag())
            {
            }                    // Poll the flag
            set_encoder_flag(0); // Clear the flag

            // Read encoder count
            int p = get_encoder_count(); // Get the encoder count
            char m[50];
            sprintf(m, "%d\r\n", p); // Format the count as a string
            NU32DIP_WriteUART1(m);   // Send the count to the client
            break;
        }

        case 'e': // Command to reset encoder
        {
            WriteUART2("b");                      // Send reset command to encoder over UART2
            sprintf(buffer, "Encoder reset\r\n"); // Format the confirmation message
            NU32DIP_WriteUART1(buffer);           // Send the confirmation to the client
            break;
        }

            // case 'f':  // Set PWM duty cycle (-100 to 100)
            // {
            //     float n = 0.0f;
            //     NU32DIP_ReadUART1(buffer, BUF_SIZE);  // Read the PWM value
            //     sscanf(buffer, "%f", &n);  // Parse the PWM value

            //     // Debug: Print received PWM value BEFORE updating
            //     sprintf(buffer, "Received PWM value: %.2f\r\n", n);
            //     NU32DIP_WriteUART1(buffer);

            //     if (n >= -100.0f && n <= 100.0f) {
            //         pwm_duty = (int)n;  // Set the PWM duty cycle (cast to int if needed)
            //         set_mode(PWM);  // Switch to PWM mode
            //     } else {
            //         NU32DIP_WriteUART1("Invalid PWM value\r\n");
            //     }
            //     break;
            // }

        case 'f': // Set PWM duty cycle (-100 to 100)
        {
            int pwm_value = 0;
            NU32DIP_ReadUART1(buffer, BUF_SIZE); // Read PWM value from UART
            sscanf(buffer, "%d", &pwm_value);    // Parse as integer

            // Debug: Print received PWM value BEFORE updating
            sprintf(buffer, "Received PWM value: %d\r\n", pwm_value);
            NU32DIP_WriteUART1(buffer);

            if (pwm_value >= -100 && pwm_value <= 100)
            {
                // pwm_duty = pwm_value; // Set PWM duty cycle
                set_pwm(pwm_value);
                set_mode(PWM);        // Switch to PWM mode
            }
            else
            {
                NU32DIP_WriteUART1("Invalid PWM value\r\n");
            }
            break;
        }

        case 'g': // Set current gains (Kp_mA, Ki_mA)
        {
            NU32DIP_ReadUART1(buffer, BUF_SIZE);     // Read user input
            sscanf(buffer, "%f %f", &Kp_mA, &Ki_mA); // Parse values
            // sprintf(buffer, "Current Gains Set: Kp_mA = %.2f, Ki_mA = %.2f\r\n", Kp_mA, Ki_mA);
            // NU32DIP_WriteUART1(buffer);
            // sprintf(buffer, "Updated Gains: Kp = %.4f, Ki = %.4f\r\n", Kp_mA, Ki_mA);
            // NU32DIP_WriteUART1(buffer);

            break;
        }

        case 'h':
        {
            sprintf(buffer, "Kp: %f, Ki: %f\r\n", Kp_mA, Ki_mA);
            NU32DIP_WriteUART1(buffer);
            break;
        }

        case 'p': // Unpower the motor
        {
            set_mode(IDLE); // Switch to IDLE mode
            break;
        }

        case 'd': // Command to read encoder angle in degrees
        {
            WriteUART2("a"); // Send request to encoder over UART2
            while (!get_encoder_flag())
            {
            }                    // Wait for encoder response
            set_encoder_flag(0); // Clear the flag

            // Calculate the angle in degrees
            int counts = get_encoder_count();               // Read encoder counts
            float angle = (counts / (334.0 * 4.0)) * 360.0; // Calculate angle in degrees
            sprintf(buffer, "%.2f\r\n", angle);             // Format the angle as a string
            NU32DIP_WriteUART1(buffer);                     // Send the angle to the client
            break;
        }

        case 'r': // Command to get current mode
        {
            OperatingMode mode = get_mode(); // get current mode
            switch (mode)
            {
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
            NU32DIP_WriteUART1(buffer); // Send the MODE to the client
            break;
        }

        case 'k': // Start ITEST Mode
        {
            set_mode(ITEST); // Set PIC32 to ITEST mode

            // Wait for ITEST to complete
            while (get_mode() == ITEST)
            {
            } // Block until test finishes

            // Send data back to client for plotting
            char send_buffer[50];
            sprintf(send_buffer, "%d\n", 100); // Send data length (100 samples)
            NU32DIP_WriteUART1(send_buffer);

            for (int i = 0; i < 100; i++)
            {
                sprintf(send_buffer, "%.2f %.2f\n", refCurrent[i], actCurrent[i]);
                NU32DIP_WriteUART1(send_buffer);
            }

            break;
        }

        case 'q': // Quit command
        {
            set_mode(IDLE); // Set the mode to IDLE
            sprintf(buffer, "Exiting and setting mode to IDLE\r\n");
            NU32DIP_WriteUART1(buffer); // Send the confirmation message
            break;
        }
        default:
            break;
        }
    }
    return 0;
}

// // config bits, constants, funcs for startup and UART
// #include "encoder.h"
// #include "utilities.h"
// #include "ina219.h"
// // include other header files here
// #define BUF_SIZE 200
// #define CPR 334
// #define INA219_REG_CURRENT 0x04
// #define PLOTPTS 100
// #define MAXPLOTPTS 400
// #define DECIMATION 10
// #define MAX_ARRAY_SIZE 1000

// volatile int dutyCycle = 0;
// // Kp = 0.03 and Ki = 0.05 for current control
// static volatile float Kp_mA = 0.03, Ki_mA = 0.05;
// // Kp = 8, Ki = 0.05 and Kd = 1500 for position control
// static volatile float Kp_pos = 8.0, Ki_pos = 0.01, Kd_pos = 1500.0;
// static volatile float desiredAngle = 0.0;
// static volatile float commandedCurrent = 0.0;
// static volatile int trajectorySize = 0;
// static float refCurrent[PLOTPTS];
// static float actCurrent[PLOTPTS];
// static float refAngle[MAXPLOTPTS];
// static float curAngle[MAXPLOTPTS];
// static float motorAngle[MAX_ARRAY_SIZE];
// static float refTraj[MAX_ARRAY_SIZE];

// const char *modeToString(enum mode_t mode)
// {
// 	switch (mode)
// 	{
// 	case IDLE:
// 		return "IDLE";
// 	case PWM:
// 		return "PWM";
// 	case ITEST:
// 		return "ITEST";
// 	case HOLD:
// 		return "HOLD";
// 	case TRACK:
// 		return "TRACK";
// 	default:
// 		return "UNKNOWN";
// 	}
// }

// void __ISR(_TIMER_2_VECTOR, IPL5SOFT) Controller(void)
// {
// 	enum mode_t currentMode = get_mode();
// 	static int counter = 0;
// 	static int plotind = 0;
// 	static int decctr = 0;
// 	static float amplitude = -200.0;
// 	static float eint = 0;
// 	switch (currentMode)
// 	{
// 	case IDLE:
// 	{
// 		dutyCycle = 0;
// 		OC1RS = PR3 * dutyCycle / 100;
// 		LATAbits.LATA0 = 0;
// 		break;
// 	}
// 	case PWM:
// 	{
// 		if (dutyCycle > 0)
// 		{
// 			OC1RS = PR3 * dutyCycle / 100;
// 			LATAbits.LATA0 = 0;
// 		}
// 		else if (dutyCycle < 0)
// 		{
// 			OC1RS = -PR3 * dutyCycle / 100;
// 			LATAbits.LATA0 = 1;
// 		}
// 		break;
// 	}
// 	case ITEST:
// 	{
// 		if (counter % 25 == 0)
// 		{
// 			amplitude = -amplitude;
// 		}
// 		float actualCurrent = INA219_read_current();

// 		float e = amplitude - actualCurrent;

// 		eint = eint + e;

// 		dutyCycle = (Kp_mA * e) + (Ki_mA * eint);

// 		if (dutyCycle > 100.0)
// 		{
// 			dutyCycle = 100.0;
// 		}
// 		if (dutyCycle <= -100.0)
// 		{
// 			dutyCycle = -100.0;
// 		}
// 		OC1RS = (unsigned int)(abs(dutyCycle)/100.0 * PR3);
// 		LATAbits.LATA0 = (dutyCycle >= 0) ? 0 : 1;

// 		refCurrent[counter] = amplitude;
// 		actCurrent[counter] = actualCurrent;
// 		counter++;
// 		if (counter >= 100)
// 		{
// 			set_mode(IDLE);
// 			counter = 0;
// 			eint = 0;
// 		}
// 		break;
// 	}
// 	case HOLD:
// 	{

// 		float actualCurrent = INA219_read_current();

// 		float e = commandedCurrent - actualCurrent;
// 		eint = eint + e;

// 		dutyCycle = (Kp_mA * e) + (Ki_mA * eint);

// 		if (dutyCycle > 100.0)
// 		{
// 			dutyCycle = 100.0;
// 		}
// 		if (dutyCycle <= -100.0)
// 		{
// 			dutyCycle = -100.0;
// 		}
// 		OC1RS = (unsigned int)(abs(dutyCycle)/100.0 * PR3);
// 		LATAbits.LATA0 = (dutyCycle >= 0) ? 0 : 1;

// 		counter++;
// 		if(counter >= 10000){
// 			set_mode(IDLE);
// 			counter = 0;
// 			eint = 0;
// 		}
// 		break;
// 	}
// 	case TRACK:
// 	{
// 		float actualCurrent = INA219_read_current();

// 		float e = commandedCurrent - actualCurrent;
// 		eint = eint + e;

// 		dutyCycle = (Kp_mA * e) + (Ki_mA * eint);

// 		if (dutyCycle > 100.0)
// 		{
// 			dutyCycle = 100.0;
// 		}
// 		if (dutyCycle <= -100.0)
// 		{
// 			dutyCycle = -100.0;
// 		}
// 		OC1RS = (unsigned int)(abs(dutyCycle)/100.0 * PR3);
// 		LATAbits.LATA0 = (dutyCycle >= 0) ? 0 : 1;

// 		// counter++;
// 		// if(counter >= 10000){
// 		// 	set_mode(IDLE);
// 		// 	counter = 0;
// 		// 	eint = 0;
// 		// }
// 		break;
// 	}

// 	default:
// 	{
// 		break;
// 	}
// 	}

// 	// insert line to clear interrupt flag
// 	IFS0bits.T2IF = 0;
// }

// void __ISR(_TIMER_4_VECTOR, IPL6SOFT) PositionController(void)
// {
// 	static float eprev = 0.0;
// 	static float eint = 0.0;
// 	static int index = 0;
// 	enum mode_t picMode = get_mode();
// 	if(picMode == HOLD){
// 		WriteUART2("a");
// 		while (!get_encoder_flag())
// 		{
// 		}
// 		set_encoder_flag(0);
// 		int cnt = get_encoder_count();
// 		float degreesPerCount = 360.0 / (CPR * 4);
// 		float encoderAngle = cnt * degreesPerCount;

// 		float e = desiredAngle - encoderAngle;
// 		float edot = e - eprev;
// 		eint = eint + e;
// 		commandedCurrent = (Kp_pos*e) + (Ki_pos*eint) + (Kd_pos*edot);
// 		eprev = e;
// 		curAngle[index] = encoderAngle;
// 		refAngle[index] = desiredAngle;
// 		index++;
// 	}
// 	if(picMode == IDLE){
// 		index = 0;
// 		eint = 0.0;
// 		eprev = 0.0;
// 	}

// 	if(picMode == TRACK){
// 		WriteUART2("a");
// 		while (!get_encoder_flag())
// 		{
// 		}
// 		set_encoder_flag(0);
// 		int cnt = get_encoder_count();
// 		float degreesPerCount = 360.0 / (CPR * 4);
// 		float encoderAngle = cnt * degreesPerCount;
// 		float desAngle = refTraj[index];
// 		float e = desAngle - encoderAngle;
// 		float edot = e - eprev;
// 		eint = eint + e;
// 		commandedCurrent = (Kp_pos*e) + (Ki_pos*eint) + (Kd_pos*edot);
// 		eprev = e;
// 		motorAngle[index] = encoderAngle;
// 		index++;
// 		if(index >= trajectorySize){
// 			desiredAngle = refTraj[trajectorySize - 1];
// 			set_mode(HOLD);
// 			eint = 0;
// 		}
// 	}

// 	IFS0bits.T4IF = 0;
// }

// void Setup_Timer4(){
// 	PR4 = 7500 - 1; // 200Hz ISR
// 	__builtin_disable_interrupts();
// 	T4CONbits.TCKPS = 0b101; // prescaler 32
// 	IPC4bits.T4IP = 6;
// 	IPC4bits.T4IS = 0;
// 	IFS0bits.T4IF = 0;
// 	IEC0bits.T4IE = 1;
// 	T4CONbits.ON = 1;
// 	TMR4 = 0;
// 	__builtin_enable_interrupts();
// }

// void Setup_PWM()
// {
// 	PR3 = 2400 - 1; // 20kHz PWM Waveform
// 	__builtin_disable_interrupts();
// 	T2CONbits.TCKPS = 0;
// 	PR2 = 9600 - 1; // 5kHz ISR
// 	TMR2 = 0;
// 	// setting priority for the interrup timer
// 	IPC2bits.T2IP = 5;
// 	IPC2bits.T2IS = 0;
// 	IFS0bits.T2IF = 0;
// 	IEC0bits.T2IE = 1;
// 	RPB15Rbits.RPB15R = 0b0101;
// 	OC1CONbits.OCTSEL = 1;
// 	T3CONbits.TCKPS = 0;	// Timer3 prescaler N=1 (1:1)
// 	TMR3 = 0;				// initial TMR3 count is 0
// 	OC1CONbits.OCM = 0b110; // PWM mode without fault pin; other OC1CON bits are defaults
// 	OC1RS = 0;				// duty cycle = OC1RS/(PR3+1)
// 	OC1R = 0;				// initialize before turning OC1 on; afterward it is read-only
// 	T3CONbits.ON = 1;		// turn on Timer3
// 	OC1CONbits.ON = 1;		// turn on OC1
// 	T2CONbits.ON = 1;
// 	__builtin_enable_interrupts();
// }

// int main()
// {
// 	char buffer[BUF_SIZE];
// 	UART2_Startup();   // Pico and PIC32 communcation
// 	NU32DIP_Startup(); // cache on, min flash wait, interrupts on, LED/button init, UART init
// 	INA219_Startup();  // current sensor startup
// 	Setup_PWM();
// 	Setup_Timer4();
// 	TRISBbits.TRISB10 = 0; // set pin B10 as output
// 	LATBbits.LATB10 = 0; // set the initial bit state
// 	TRISAbits.TRISA0 = 0; // set pin A0 as output
// 	LATAbits.LATA0 = 0;	  // set the initial direction of the motor rotation (depends on pin state)
// 	NU32DIP_YELLOW = 1;	  // turn off the LEDs
// 	NU32DIP_GREEN = 1;
// 	set_mode(IDLE);
// 	__builtin_disable_interrupts();
// 	// in future, initialize modules or peripherals here
// 	__builtin_enable_interrupts();

// 	while (1)
// 	{
// 		NU32DIP_ReadUART1(buffer, BUF_SIZE); // we expect the next character to be a menu command
// 		NU32DIP_GREEN = 1;					 // clear the error LED
// 		switch (buffer[0])
// 		{
// 		case 'a':
// 		{
// 			// read the current sensor (ADC counts)
// 			float adcReading = readINA219(INA219_REG_CURRENT);
// 			sprintf(buffer, "%f\r\n", adcReading);
// 			NU32DIP_WriteUART1(buffer);
// 			break;
// 		}

// 		case 'b':
// 		{
// 			// read the current sensors (mA current)
// 			float currentReading = INA219_read_current();
// 			sprintf(buffer, "%f\r\n", currentReading);
// 			NU32DIP_WriteUART1(buffer);
// 			break;
// 		}
// 		case 'c':
// 		{
// 			// read the encoder count
// 			WriteUART2("a");
// 			while (!get_encoder_flag())
// 			{
// 			}
// 			set_encoder_flag(0);
// 			sprintf(buffer, "%d\r\n", get_encoder_count());
// 			NU32DIP_WriteUART1(buffer);
// 			break;
// 		}

// 		case 'd':
// 		{
// 			// read the encoder count (degrees)
// 			WriteUART2("a");
// 			while (!get_encoder_flag())
// 			{
// 			}
// 			set_encoder_flag(0);
// 			int cnt = get_encoder_count();
// 			float degreesPerCount = 360.0 / (CPR * 4);
// 			float encoderAngle = cnt * degreesPerCount;
// 			sprintf(buffer, "%f\r\n", encoderAngle);
// 			NU32DIP_WriteUART1(buffer);
// 			break;
// 		}

// 		case 'e':
// 		{
// 			// reset the encoder count
// 			WriteUART2("b");
// 			break;
// 		}

// 		case 'f':
// 		{
// 			// set PWM duty cycle
// 			NU32DIP_ReadUART1(buffer, BUF_SIZE);
// 			sscanf(buffer, "%d", &dutyCycle);
// 			set_mode(PWM);
// 			break;
// 		}

// 		case 'g':
// 		{
// 			NU32DIP_ReadUART1(buffer, BUF_SIZE);
// 			sscanf(buffer, "%f %f", &Kp_mA, &Ki_mA);
// 			break;
// 		}

// 		case 'h':
// 		{
// 			sprintf(buffer, "Kp: %f, Ki: %f\r\n", Kp_mA, Ki_mA);
// 			NU32DIP_WriteUART1(buffer);
// 			break;
// 		}

// 		case 'i':
// 		{
// 			NU32DIP_ReadUART1(buffer, BUF_SIZE);
// 			sscanf(buffer, "%f %f %f", &Kp_pos, &Ki_pos, &Kd_pos);
// 			break;
// 		}

// 		case 'j':
// 		{
// 			sprintf(buffer, "Kp: %f, Ki: %f, Kd: %f\r\n", Kp_pos, Ki_pos, Kd_pos);
// 			NU32DIP_WriteUART1(buffer);
// 			break;
// 		}

// 		case 'k':
// 		{
// 			set_mode(ITEST);
// 			enum mode_t currentMode = get_mode();
// 			while (currentMode != IDLE)
// 			{
// 				currentMode = get_mode();
// 			}
// 			sprintf(buffer, "%d\r\n", PLOTPTS);
// 			NU32DIP_WriteUART1(buffer);
// 			if (currentMode == IDLE)
// 			{
// 				for (int i = 0; i < PLOTPTS; i++)
// 				{
// 					sprintf(buffer, "%f %f\r\n", refCurrent[i], actCurrent[i]);
// 					NU32DIP_WriteUART1(buffer);
// 				}
// 			}
// 			break;
// 		}

// 		case 'l':
// 		{
// 			NU32DIP_ReadUART1(buffer, BUF_SIZE);
// 			sscanf(buffer, "%f", &desiredAngle);
// 			set_mode(HOLD);
// 			enum mode_t currentMode = get_mode();
// 			while (currentMode != IDLE)
// 			{
// 				currentMode = get_mode();
// 			}
// 			sprintf(buffer, "%d\r\n", MAXPLOTPTS);
// 			NU32DIP_WriteUART1(buffer);
// 			if (currentMode == IDLE)
// 			{
// 				for (int i = 0; i < MAXPLOTPTS; i++)
// 				{
// 					sprintf(buffer, "%f %f\r\n", refAngle[i], curAngle[i]);
// 					NU32DIP_WriteUART1(buffer);
// 				}
// 			}
// 			break;
// 		}

// 		case 'm':
// 		{
// 			int index = 0;
// 			NU32DIP_ReadUART1(buffer, BUF_SIZE);
// 			sscanf(buffer, "%d", &trajectorySize);
// 			for(index = 0; index < trajectorySize; index++){
// 				NU32DIP_ReadUART1(buffer,BUF_SIZE);
// 				sscanf(buffer, "%f", &refTraj[index]);
// 			}

// 			break;
// 		}

// 		case 'n':
// 		{
// 			int index = 0;
// 			NU32DIP_ReadUART1(buffer, BUF_SIZE);
// 			sscanf(buffer, "%d", &trajectorySize);
// 			for(index = 0; index < trajectorySize; index++){
// 				NU32DIP_ReadUART1(buffer,BUF_SIZE);
// 				sscanf(buffer, "%f", &refTraj[index]);
// 			}
// 			break;
// 		}

// 		case 'o':
// 		{
// 			set_mode(TRACK);
// 			enum mode_t currentMode = get_mode();
// 			while(currentMode != HOLD){
// 				currentMode = get_mode();
// 			}
// 			sprintf(buffer, "%d\r\n", trajectorySize);
// 			NU32DIP_WriteUART1(buffer);
// 			if (currentMode == HOLD)
// 			{
// 				for (int i = 0; i < trajectorySize; i++)
// 				{
// 					sprintf(buffer, "%f %f\r\n", refTraj[i], motorAngle[i]);
// 					NU32DIP_WriteUART1(buffer);
// 				}
// 			}
// 			break;
// 		}

// 		case 'p':
// 		{
// 			// set PIC mode
// 			set_mode(IDLE);
// 			break;
// 		}

// 		case 'q':
// 		{
// 			// handle q for quit. Later you may want to return to IDLE mode here.
// 			enum mode_t currentMode = IDLE;
// 			set_mode(currentMode);
// 			break;
// 		}
// 		case 'r':
// 		{
// 			// read the current mode of the PIC32
// 			enum mode_t currentMode = get_mode();
// 			sprintf(buffer, "%s\r\n", modeToString(currentMode));
// 			NU32DIP_WriteUART1(buffer);
// 			break;
// 		}

// 		default:
// 		{
// 			NU32DIP_GREEN = 0; // turn on LED2 to indicate an error
// 			break;
// 		}
// 		}
// 	}
// 	return 0;
// }
