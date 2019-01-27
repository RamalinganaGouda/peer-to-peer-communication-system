
//-----------------------------------------------------------------------------
// SUBJECT : EE5314
// AUTHOR : RAMALINGANA GOUDA YELUBENCHI
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <tm4c123gh6pm.h>
#include <hw_nvic.h>
#include <hw_types.h>
//------------------------------------------------------------------------------
// SOURCE ADDRESS
//------------------------------------------------------------------------------
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t SRC_ADDR = 10;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define TX_LED       (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4))) //USE BITBANDING FOR DEFINING TX LED
#define RX_LED       (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4))) //USE BITBANDING FOR DEFINING RX LED
#define D_EN         (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4))) //USE BITBANDING FOR DEFINING DATA ENABLE FOR UART1

#define RED_LED      PWM1_2_CMPB_R                               // Give a label for PWM1_2_b compare register
#define BLUE_LED     PWM1_3_CMPA_R                               // Give a label for PWM1_3_a compare register
#define GREEN_LED    PWM1_3_CMPB_R                               // Give a label for PWM1_3_b compare register


#define MAX_CHARS 80                                       //Maximum number of characters in the input string
#define MAX_MESSAGES 25                                    //Define the max number of messages that can be stored
#define MAX_RETRANSCOUNT 5                                 //Define the max number of times a message can be retransmitted
#define LED_TIMEOUT 15
#define MAX_PACKET_SIZE 100                                //Define the max size of the received packet
#define BROADCAST_ADDR 255                                 //Define the broadcast address
#define PACKET_HEADER_SIZE 7                               //Define the overhead of each packet like addr, channel, cmd, checksum, size etc.
#define INVALID_COMMAND "Invalid command entered!!\n"      //storing the error message
#define INVALID_PARAMETER "Invalid parameter found\n"      //storing the error message
#define TABLE_FULL "The message table is full\n"           //storing table status message
#define T 500                                              //fixed time interval for retransmission time
#define T0 100                                            //variable time interval for retransmission time
#define DEADLOCK_TIMEOUT 100

//CONTROL COMMANDS
#define CMD_SET 0x00
#define CMD_PIECEWISE 0x01
#define CMD_PULSE 0x02
#define CMD_SQUARE 0x03
#define CMD_SAWTOOTH 0x04
#define CMD_TRIANGLE 0x05

//DATA COMMANDS
#define CMD_DATA_REQUEST 0x20
#define CMD_DATA_REPORT 0x21
#define CMD_REPORT_CONTROL 0x22

//UI COMMANDS
#define CMD_LCD_DISPLAY_TEXT 0x40
#define CMD_RGB 0x48
#define CMD_RGB_PIECEWISE 0x49

//SERIAL COMMANDS
#define CMD_UART_DATA 0x50
#define CMD_UART_CONTROL 0x51
#define CMD_I2C_COMMAND 0x54

//SYSTEM COMMANDS
#define CMD_ACKNOWLEDGE 0x70
#define CMD_POLL_REQUEST 0x78
#define CMD_POLL_RESPONSE 0x79
#define CMD_SET_ADDRESS 0x7A
#define CMD_NODE_CONTROL 0x7D
#define CMD_BOOTLOAD 0x7E
#define CMD_RESET 0x7F

//COMMAND STRING RELATED VARIABLES
uint8_t fields = 0;                             //Holds the value of the number of fields in the commands
uint8_t pos[80];                                //Array that holds the starting index of each parsed command
char type[80];                                  //String that holds the type of the parsed command
char str[MAX_CHARS+1];                          //String that holds the input command
uint8_t dat[100];                               //Global variable that holds the data field to be transmitted

//data plane
uint8_t s_id = 0;
uint8_t dst_addr[MAX_MESSAGES];
uint8_t seq_id[MAX_MESSAGES];
uint8_t cmd[MAX_MESSAGES];
uint8_t channel[MAX_MESSAGES];
uint8_t size[MAX_MESSAGES];
uint8_t checksum[MAX_MESSAGES];
uint8_t data[MAX_MESSAGES][100];
uint32_t timeout[MAX_MESSAGES];

//control plane
bool valid[MAX_MESSAGES];
bool ackreq[MAX_MESSAGES];
uint8_t retranscount[MAX_MESSAGES];
uint8_t n[MAX_MESSAGES] = {0};                                      // incrementing variable for retransmission timeout

//global variables for timer2isr
uint8_t N;
uint8_t value[10];
uint8_t valptr = 0;
uint16_t dwell;
uint16_t dwellcount = 0;
uint16_t cycles;
uint16_t t1;
uint16_t t2;
int delta;
int d12;
int d21;
uint8_t signal;                                         //signal values 1-piecewise,2-pulse,3-square,4-sawtooth,5-triangular
uint16_t cycle_count = 0;
uint8_t r = 0;

//global variables for timer1isr
bool in_progress = false;
uint8_t current_index = 0;
uint8_t current_phase = 0;
uint8_t old_current_phase = 0;
uint8_t tx_message[100];                                            // Global variable that holds the message to be transmitted
uint8_t rx_phase = 0;
uint8_t old_rx_phase = 0;
uint8_t rx_data[MAX_PACKET_SIZE];

//general global variables
bool CS_ENABLE = false;
bool ACK = false;
bool RANDOM = false;
uint16_t randtb[30] = {100,27,500,3,8,11,56,258,4,100,35,29,65,87,56,22,2,49,93,685,38,6,489,398,47,28,77,572,212,243};
uint8_t randtbsize = 30;
uint8_t randptr = 0;
uint8_t RX_LED_TIMEOUT = 0;
uint8_t TX_LED_TIMEOUT = 0;
bool in_tx = false;
bool in_rx = false;
bool current_eps = true;
bool invalid_arg = false;
bool reset_flag = false;
uint8_t rx_deadlock_count;
uint8_t tx_deadlock_count;
char uart0[MAX_CHARS+1];

//TEST values
char* str2;
uint16_t t;
uint8_t k;
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// DELAY FUNCTION
 void waitMicrosecond(uint32_t us)
 {
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
 }

// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV| (4 << SYSCTL_RCC_SYSDIV_S);

    // Enable GPIO port F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOC | SYSCTL_RCGC2_GPIOF;

    // Configure RX and TX LEDs
    GPIO_PORTA_DIR_R = 0x18;    // bits 4 and 3 are outputs, other pins are inputs
    GPIO_PORTA_DR2R_R = 0x18;   // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTA_DEN_R = 0x18;    // enable LEDs
    GPIO_PORTA_PDR_R = 0x18;    // enable internal pull down for LEDs



    // Configure PF1,PF2,PF3 as PWM pins
    GPIO_PORTF_DIR_R |= 0x0E;   // make bit5 an output
    GPIO_PORTF_DR2R_R |= 0x0E;  // set drive strength to 2mA
    GPIO_PORTF_DEN_R |= 0x0E;   // enable bit5 for digital
    GPIO_PORTF_AFSEL_R |= 0x0E; // select auxilary function for bit 5
    GPIO_PORTF_PCTL_R = GPIO_PCTL_PF1_M1PWM5 | GPIO_PCTL_PF2_M1PWM6 | GPIO_PCTL_PF3_M1PWM7; // enable PWM on bit 5

    //SYSCTL_RCGC0_R |= SYSCTL_RCGC0_PWM0;             // turn-on PWM0 module
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;           // turn-on PWM1 module
    __asm(" NOP");                                   // wait 3 clocks
    __asm(" NOP");
    __asm(" NOP");
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;                // reset PWM1 module
    SYSCTL_SRPWM_R = 0;                              // clear reset PWM1 module
    PWM1_2_CTL_R = 0;                                // turn-off PWM1 generator 2
    PWM1_3_CTL_R = 0;                                // turn-off PWM1 generator 3
    PWM1_2_GENB_R = PWM_1_GENB_ACTCMPBD_ZERO | PWM_1_GENB_ACTLOAD_ONE;     // output 5 on PWM1, gen 2b, cmpb
    PWM1_3_GENA_R = PWM_1_GENA_ACTCMPAD_ZERO | PWM_1_GENA_ACTLOAD_ONE;     // output 6 on PWM1, gen 3a, cmpa
    PWM1_3_GENB_R = PWM_1_GENB_ACTCMPBD_ZERO | PWM_1_GENB_ACTLOAD_ONE;     // output 7 on PWM1, gen 3b, cmpb

    PWM1_2_LOAD_R = 256;
    PWM1_3_LOAD_R = 256;

    PWM1_INVERT_R = PWM_INVERT_PWM5INV | PWM_INVERT_PWM6INV | PWM_INVERT_PWM7INV;  // invert outputs for duty cycle increases with increasing compare values

    PWM1_2_CMPB_R = 0;                                                           // red off (0=always low, 1023=always high)
    PWM1_3_CMPA_R = 0;                                                           // green off
    PWM1_3_CMPB_R = 0;                                                           // blue off

    PWM1_2_CTL_R = PWM_1_CTL_ENABLE;                                             // turn-on PWM1 generator 2
    PWM1_3_CTL_R = PWM_1_CTL_ENABLE;                                             // turn-on PWM1 generator 3
    PWM1_ENABLE_R = PWM_ENABLE_PWM5EN | PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;   // enable outputs

    SYSCTL_RCGCEEPROM_R = SYSCTL_RCGCEEPROM_R0;
    EEPROM_EESIZE_R = 0x10000;
    EEPROM_EEBLOCK_R = 0;
    EEPROM_EEOFFSET_R = 0;

   // EEPROM_EERDWR_R = SRC_ADDR;

    // Configure UART0 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;                           // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                                             // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                                           // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;       // define peripheral control for TX and RX of UART0

    // configure UART1 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;                           // turn-on UART1
    GPIO_PORTC_DIR_R |= 0x60;                                          // define TX(PC5),D_EN(PC6) as output and RX(PC4) as input
    GPIO_PORTC_DEN_R |= 0x70;                                          // enable TX,D_EN and RX as digital pins
    GPIO_PORTC_PDR_R |= 0x70;                                          // pull down TX,D_EN and RX as digital pins
    GPIO_PORTC_AFSEL_R |= 0x30;                                        // enable alternate function on PC4 and PC5
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC5_U1TX | GPIO_PCTL_PC4_U1RX;      // define peripheral control for TX and RX of UART1

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                                   // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                                    // use system clock (40 MHz)
    UART0_IBRD_R = 21;                                                 // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                                                 // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;                   // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;       // enable TX, RX, and module

    // Configure UART1 to 38400 baud
    UART1_CTL_R = 0;                                                                                   // turn-off UART1 to allow safe programming
    UART1_CC_R = UART_CC_CS_SYSCLK;                                                                    // use system clock (40 MHz)
    UART1_IBRD_R = 65;                                                                                 // r = 40 MHz / (Nx38400Hz), set floor(r)=65, where N=16
    UART1_FBRD_R = 7;                                                                                  // round(fract(r)*64)=7
    UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN | UART_LCRH_SPS | UART_LCRH_PEN | UART_LCRH_EPS;                   // configure for 8N1 w/ 16-level FIFO
    UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;                                       // enable TX, RX and module

    int i;
    for(i = 0;i <= MAX_MESSAGES;i++)                                                                   // clear the valid flag of the message table
    {
        valid[i] = false;
        timeout[i] = 0;
    }
}


// Configure Timer 1 as the time base
void initTr1()
{
SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer
TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
TIMER1_TAILR_R = 0x9C40;                         // set load value to 40e6 for 1 Hz interrupt rate
TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
}

// Configure Timer 1 as the time base
void initTr2()
{
SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2;       // turn-on timer
TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
TIMER2_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
TIMER2_TAILR_R = 0x61A80;                         // set load value to 40e6 for 1 Hz interrupt rate
TIMER2_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
NVIC_EN0_R |= 1 << (INT_TIMER2A-16);             // turn-on interrupt 37 (TIMER1A)
//TIMER2_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);     // wait till the TX FIFO is empty
    UART0_DR_R = c;                        // put the character in the FIFO
}

//
void nextLine()
{
    putcUart0('\n');
    putcUart0('\r');
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str,bool nextline)
{
    uint8_t i;                            // define a counter
    for (i = 0; i < strlen(str); i++)     // increment the counter till the end of the string
      putcUart0(str[i]);                  // put individual characters on the FIFO
    if(nextline)
        nextLine();

}

// Blocking function that writes a number when the UART buffer is not full
void putnUart0(uint8_t c)
{
    char snum[10];
    uint8_t a = c;
    uint8_t n,i = 0,j = 0;
    while(a!=0)
    {
        //a = a%10;
        a = a/10;
        i++;
    }
    snum[i] = 0;
    a = c;
    if(c == 0)
    putsUart0("0", false);
    while(a != 0)
    {
        n = a%10;
        a = a/10;
        snum[i-1-j] = n+48;
        j++;
    }
    putsUart0(snum, false);                   //prints a number after convertig it to ASCII
}


// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);    // wait till the RX FIFO is full
    return UART0_DR_R & 0xFF;             // return the received character after masking the control information
}

//Blocking function that reads a string when the UART buffer is full
void getsUart0()
{
    uint32_t count = 0;                  // define counter
    char c;                              // define a temporary storage variable
    get : c = getcUart0();               // receive a character
    if(c == 8)                           //check if character entered is a backspace
    {
        if(count > 0)                    // check if count is greater than 0
            count--;                     // decrement count if count is greater than 0
        goto get;                        // get another character
    }

    else if(c == 13)                     //check if the character entered is a carriage return
    {
       done : str[count++] = 0;          // end the string if a carriage return is entered
       return;                           //return from the loop
    }
    else if(c >= ' ')                    //check if the entered character is valid
    {
        str[count++] = c;                // store the character
        if(count >= MAX_CHARS)           // check if the count exceeds the max number of characters allowed
            goto done;                   // end the string and return from loop
        else
            goto get;                    // get another character
    }
    else
        goto get;                        // get another character
}

//Function for parsing the input command
void parseCommand(char* str)
{
    uint8_t i,j=0,skip=0,s;                                                            // define local variables
    fields = 0;                                                                        // initialize fields
    s = strlen(str);                                                                   // compute string length of the command
    for(i = 0;i<s;i++)
    {
        uart0[i] = str[i];
    }
    for(i=0;i<s;i++)                                                                   // check each character for its type
    {
        if(((str[i] >= 'A')&&(str[i] <= 'Z'))||((str[i] >= 'a')&&(str[i] <= 'z')))     // check if the character is an alphabet
        {
            if((str[i] >= 'a')&&(str[i] <= 'z'))                                       // make the string case insensitive
                str[i] = str[i]-32;
            if(skip == 0)                                                              // check the skip flag to assign position and type of the field
            {
                fields++;                                                              // increment fields if a valid field is found
                pos[j] = i;                                                            // store the position of the field
                type[j] = 'a';                                                         // store the data type of the field
                j++;                                                                   // increment the local index
                skip = 1;                                                              // set the skip flag
            }

        }
        else if((str[i] >= '0')&&(str[i] <= '9'))                                      // check if the valid character is a number
        {
            if(skip == 0)                                                              // check the skip flag to assign position and type of the field
            {
                fields++;                                                              // increment fields if a valid field is found
                pos[j] = i;                                                            // store the position of the field
                type[j] = 'n';                                                         // store the data type of the field
                j++;                                                                   // increment the local index
                skip = 1;                                                              // set the skip flag
            }

        }
        else
        {
            str[i] = 0;                                                               // insert a null for each delimiter chaerecter found
            skip = 0;                                                                 // clear the skip flag
        }

    }

}

//Function for checking command validity
bool isCommand(char strcmd[],uint8_t min_args)
{
    if((strcmp(strcmd,&str[pos[0]]) == 0)&&(fields > min_args))       // check if the entered command and arguments match with the required command
        return true;                                                  // return true if the command is valid
    else
        return false;                                                 // return false if the command is invalid
}

// function that returns the string at a given position
char* getString(uint8_t field)
{
   if(type[field] == 'a')                                             // check for the validity of the arguments of the command
   {
       invalid_arg |= false;                                           // clear invalid flag if a valid argument is found
       return &str[pos[field]];                                       // return the position of the string if valid
   }
   else
   {
       putsUart0(INVALID_PARAMETER, true);                            // if argument invalid then show error to user and return 0
       invalid_arg |= true;                                            // set invalid flag if an invalid argument is found
       return 0;
   }
}

char* getStringUART(uint8_t field)
{
   if(type[field] == 'a')                                             // check for the validity of the arguments of the command
   {
       invalid_arg |= false;                                           // clear invalid flag if a valid argument is found
       return &uart0[pos[field]];                                       // return the position of the string if valid
   }
   else
   {
       putsUart0(INVALID_PARAMETER, true);                            // if argument invalid then show error to user and return 0
       invalid_arg |= true;                                            // set invalid flag if an invalid argument is found
       return 0;
   }
}
// function that returns the number at a given position
uint16_t getNumber(uint8_t field)
{
    uint8_t number;
    char* c;
    if(type[field] == 'n')                                            // check for the validity of the arguments of the command
    {
        c = &str[pos[field]];                                         // obtain the position of the number if valid
        number = atoi(c);                                             // convert the string to an integer
        invalid_arg = false;                                          // clear invalid flag if a valid argument is found
        return number;                                                // return the number
    }
    else
    {
        invalid_arg = true;                                          // set invalid flag if an invalid argument is found
        putsUart0(INVALID_PARAMETER, true);                          // if argument invalid then show error to user and return 0
        return 0;
    }
}

// function that stores commands to be transmitted in the message table
void sendPackets(uint8_t d_add, uint8_t c, uint8_t ch, uint8_t sz, uint8_t d[])
{
    uint8_t i = 0;
    uint8_t j = 0;
    uint8_t check = 0;
    while(valid[i])                                                           // search for an empty location in the table
    {
        i++;
        if(i > MAX_MESSAGES-1)
        {
            putsUart0(TABLE_FULL, true);
            return;                                                           // take care if the message table is full
        }
    }
    dst_addr[i] = d_add;                                                      // store address
    seq_id[i] = s_id;                                                         // store sequence id
    s_id++;                                                                   // increment sequence id
    if(ACK && (c != CMD_ACKNOWLEDGE))                                         // check if acknowledgment is enabled
    {
        cmd[i] = c|0x80;                                                      // set 7th bit if ACK ON
        ackreq[i] = true;                                                     // set ackreq field for that message
    }
    else
    {
        cmd[i] = c;                                                           // clear 7th bit if ACK OFF
        ackreq[i] = false;                                                    // clear the ackreq field of the message
    }
    channel[i] = ch;                                                          // store the channel
    size[i] = sz;                                                             // store the size of the data
    for(j = 0;j < sz;j++)
        data[i][j] = d[j];                                                    // store the data
    for(j = 0;j < sz;j++)
        check += data[i][j];                                                  // compute the checksum
    check += SRC_ADDR+dst_addr[i]+seq_id[i]+cmd[i]+channel[i]+size[i];
    checksum[i] = ~check;
    valid[i] = true;                                                          // validate the entry

}

// function that finds the entered command
void findCmd()
{
     bool valid = false;
     bool ext = false;
     uint8_t dstadr,chan,size,i,cmd;
     invalid_arg = false;
     char* c;

     if(isCommand("RESET",1))                                           // check if command entered is RESET
     {
         ext = true;                                                    // set ext flag as RESET is an external command
         dstadr = getNumber(1);
         chan = 0;                                                      // get channel
         size = 0;                                                      // define size
         dat[0] = 0;                                                    // get the data
         cmd = CMD_RESET;
         if(~invalid_arg)                                               // set valid flag if all the arguments are valid
             valid = true;
         else                                                           // clear valid flag if the arguments are invalid
         {
             putsUart0("Invalid argument", true);
             valid = false;
         }
     }
     if(isCommand("CS",1))                                              // check if command entered is CS
     {
         ext = false;                                                   // clear external flag and CS is an internal command
         c = getString(1);                                              // get the argument
         if(~invalid_arg)                                               // check argument for validity
         {
             if (strcmp(c,"ON") == 0)                                   // if argument is "ON", set CS
                  {
                     CS_ENABLE = true;
                     putsUart0("Carrier sense is enabled", true);
                     valid = true;                                      // set the valid flag
                  }
             else if (strcmp(c,"OFF") == 0)                             // if argument is "OFF", clear CS
                 {
                     CS_ENABLE = false;
                     putsUart0("Carrier sense is disabled", true);
                     valid = true;                                      // set the valid flag
                 }
             else
             {
             putsUart0("Invalid argument", true);
             valid = false;                                              // set the valid flag
             }
         }
         else                                                           // clear valid if argument is invalid
             valid = false;
     }
     if(isCommand("RANDOM",1))                                          // check if command entered is RANDOM
     {
         ext = false;                                                   // clear external flag and CS is an internal command
         c = getString(1);                                              // get the argument
         if(~invalid_arg)                                               // check argument for validity
         {
             if (strcmp(c,"ON") == 0)                                   // if argument is "ON", set RANDOM
                  {
                     RANDOM = true;
                     putsUart0("Random Timeout is enabled", true);
                     valid = true;                                      // set the valid flag
                  }
                 else if (strcmp(c,"OFF") == 0)                         // if argument is "OFF", clear RANDOM
                 {
                     RANDOM = false;
                     putsUart0("Random Timeout is disabled", true);
                     valid = true;                                      // set the valid flag
                 }
                 else
                 {
                     putsUart0("Invalid argument", true);
                     valid = false;
                 }
         }
         else                                                            // clear valid if argument is invalid
             valid = false;
     }
     if(isCommand("POLL",0))                                             // check if command entered is
     {
         ext = true;                                                     // set the external flag as the command is external
         dstadr = BROADCAST_ADDR;
         chan = 0;                                                       // get channel
         size = 0;                                                       // define size
         dat[0] = 0;                                                     // get the data
         cmd = CMD_POLL_REQUEST;
         if(~invalid_arg)
             valid = true;                                               // set valid flag
         else                                                            // clear valid flag if the arguments are invalid
         {
             putsUart0("Invalid argument", true);
             valid = false;
         }
     }
     if(isCommand("SA",2))                                               // check if command entered is SA
     {
         ext = true;                                                     // set the external flag as the command is external
         dstadr = getNumber(1);
         chan = 0;
         size = 1;
         dat[0] = getNumber(2);
         cmd = CMD_SET_ADDRESS;
         if(~invalid_arg)
             valid = true;                                                // set valid flag
         else                                                             // clear valid if argument is invalid
         {
             putsUart0("Invalid argument", true);
             valid = false;
         }

     }
     if(isCommand("ACK",1))                                              // check if command entered is ACK
     {
         ext = false;                                                    // clear external flag and CS is an internal command
         c = getString(1);                                               // get the argument
         if(~invalid_arg)                                                // check for validity
         {
             if (strcmp(c,"ON") == 0)                                    // if argument is "ON", set ACK flag
             {
                 ACK = true;
                 putsUart0("Ack is enabled", true);
                 valid = true;                                           // set valid flag
             }
             else if (strcmp(c,"OFF") == 0)                              // if argument  is "OFF", clear ACK flag
             {
                 ACK = false;
                 putsUart0("ACK is disabled", true);
                 valid = true;                                           // set valid flag
             }
             else
             {
                 putsUart0("Invalid argument", true);
                 valid = false;
             }

         }
         else                                                            // clear valid if argument is invalid
         {
             putsUart0("Invalid argument", true);
             valid = false;
         }
     }
     if(isCommand("SET",3))                                              // check if command entered is SET
     {
         ext = true;                                                     // set the external flag as the command is external
         dstadr = getNumber(1);                                          // get destination address
         chan = getNumber(2);                                            // get channel
         size = 1;                                                       // define size
         dat[0] = getNumber(3);                                          // get the data
         cmd = CMD_SET;
         if(~invalid_arg)
             valid = true;                                               // set valid flag
         else                                                            // clear valid if argument is invalid
         {
             putsUart0("Invalid argument", true);
             valid = false;
         }

     }

     if(isCommand("RGB",5))                                             // check if command entered is RGB
     {
         ext = true;                                                    // set the external flag as the command is external
         dstadr = getNumber(1);                                         // get destination address
         chan = getNumber(2);                                           // get destination channel
         size = 3;                                                      // define the size
         for(i = 0;i < size;i++)                                        // get data
             dat[i] = getNumber(3+i);
         cmd = CMD_RGB;
         if(~invalid_arg)
             valid = true;                                              // set valid flag
         else                                                           // clear valid if argument is invalid
         {
             putsUart0("Invalid argument", true);
             valid = false;
         }
     }

     if(isCommand("GET",2))                                             // check if command entered is LCDTEXT
     {
         ext = true;                                                    // set the external flag as the command is external
         dstadr = getNumber(1);
         chan = getNumber(2);                                           //must do this
         cmd = CMD_DATA_REQUEST;
         size = 0;
         dat[0] = 0;
         if(~invalid_arg)
             valid = true;                                              // set valid flag
         else                                                           // clear valid flag if the arguments are invalid
         {
             putsUart0("Invalid argument", true);
             valid = false;
         }
     }

     if(isCommand("UART",3))                                            // check if command entered is UART COMMAND
     {
         ext = true;                                                    // set the external flag as the command is external
         dstadr = getNumber(1);
         chan = getNumber(2);
         c = getStringUART(3);
         size = strlen(uart0)-pos[3];
         for(i = 0;i<size;i++)
             dat[i] = (uint8_t) uart0[i+pos[3]];
         cmd = CMD_UART_DATA;
         if(~invalid_arg)
             valid = true;                                              // set valid flag
         else                                                           // clear valid flag if the arguments are invalid
         {
             putsUart0("Invalid argument", true);
             valid = false;
         }
     }
     if(isCommand("PIECEWISE",8))                                      // check if command is PIECEWISE
     {
         ext = true;                                                   // set external flag as the command is external
         dstadr = getNumber(1);
         chan = getNumber(2);
         dat[0] = getNumber(3);
         for(i = 0;i<dat[0];i++)
             dat[i+1] = getNumber(i+4);
         i = dat[0];
         uint16_t dwell = getNumber(i+4);
         i++;
         uint16_t cycles = getNumber(i+4);
         i++;
         i = dat[0];
         uint8_t dwellupper = (dwell & 0xFF00)>>8;
         uint8_t dwelllower = (dwell & 0xFF);
         dat[i+1] = dwellupper;
         i++;
         dat[i+1] = dwelllower;
         i++;
         uint8_t cyclesupper = (cycles & 0xFF00)>>8;
         uint8_t cycleslower = cycles & 0xFF;
         dat[i+1] = cyclesupper;
         i++;
         dat[i+1] = cycleslower;
         cmd = CMD_PIECEWISE;
         size = dat[0]+5;
         if(~invalid_arg)
             valid = true;                                              // set valid flag
         else                                                           // clear valid flag if the arguments are invalid
         {
             putsUart0("Invalid argument", true);
             valid = false;
         }

     }
     if(isCommand("SQUARE",7))                                          // check if command is SQUARE
          {
              ext = true;                                               // set external flag as command is external
              dstadr = getNumber(1);
              chan = getNumber(2);
              dat[0] = getNumber(3);
              dat[1] = getNumber(4);
              uint8_t t1 = getNumber(5);
              uint8_t t1upper = (t1&0xFF00)>>8;
              uint8_t t1lower = t1 & 0xFF;
              dat[2] = t1upper;
              dat[3] = t1lower;
              uint8_t t2 = getNumber(6);
              uint8_t t2upper = (t2 & 0xFF00)>>8;
              uint8_t t2lower = t2 & 0xFF;
              dat[4] = t2upper;
              dat[5] = t2lower;
              uint8_t cycle = getNumber(7);
              uint8_t cycleupper = (cycle & 0xFF00)>>8;
              uint8_t cyclelower = cycle & 0xFF;
              dat[6] = cycleupper;
              dat[7] = cyclelower;
              cmd = CMD_SQUARE;
              size = 8;
              if(~invalid_arg)
                  valid = true;                                              // set valid flag
              else                                                           // clear valid flag if the arguments are invalid
              {
                  putsUart0("Invalid argument", true);
                  valid = false;
              }

          }
     if(isCommand("PULSE",4))                                                // check if the command entered is PULSE
          {
              ext = true;                                                    // set the external flag as the command is external
              dstadr = getNumber(1);
              chan = getNumber(2);
              dat[0] = getNumber(3);
              uint8_t t1 = getNumber(4);
              uint8_t t1upper = (t1 & 0xFF00)>>8;
              uint8_t t1lower = t1 & 0xFF;
              dat[1] = t1upper;
              dat[2] = t1lower;
              cmd = CMD_PULSE;
              size = 3;
              if(~invalid_arg)
                  valid = true;                                              // set valid flag
              else                                                           // clear valid and external, if argument is invalid
              {
                  putsUart0("Invalid argument", true);
                  valid = false;
              }

          }
     if(isCommand("SAWTOOTH",7))                                           // check if the command entered is SAWTOOTH
          {
              ext = true;                                                  // set the external flag as the command is external
              dstadr = getNumber(1);
              chan = getNumber(2);
              dat[0] = getNumber(3);
              dat[1] = getNumber(4);
              dat[2] = getNumber(5);
              uint8_t dwell = getNumber(6);
              uint8_t dwellupper = (dwell&0xFF00)>>8;
              uint8_t dwelllower = dwell&0xFF;
              dat[3] = dwellupper;
              dat[4] = dwelllower;
              uint8_t cycle = getNumber(7);
              uint8_t cycleupper = (cycle&0xFF00)>>8;
              uint8_t cyclelower = cycle&0xFF;
              dat[5] = cycleupper;
              dat[6] = cyclelower;
              cmd = CMD_SAWTOOTH;
              size = 7;
              if(~invalid_arg)
                  valid = true;                                              // set valid flag
              else                                                           // clear valid flag if the arguments are invalid
              {
                  putsUart0("Invalid argument", true);
                  valid = false;
              }

          }
     if((~invalid_arg) && ext && valid)                                      // send the packet to the message table if the arguments are valid and the message is external
     {
         sendPackets(dstadr,cmd,chan,size,&dat[0]);                          // display message info to user
         putsUart0("Queuing Msg  ", false);
         putnUart0(s_id - 1);
         putsUart0(" ", true);
     }
     if(!valid)                                                              // print error if no valid message was found
     {
         putsUart0(INVALID_COMMAND, true);
     }
}

//function to process the received data
void processPacket()
{
    uint8_t i;
    uint8_t rxdsrc_addr,rxddst_addr,rxdsq_id,rxdcmd,rxdchannel,rxdsize,rxdchecksum;
    uint8_t k[100];
    uint8_t temp[5];
    uint8_t in_check = 0;
    char st[100];
    if(((rx_data[0] == SRC_ADDR)||(rx_data[0] == BROADCAST_ADDR))&&(rx_data[1] != SRC_ADDR))    // check destination and source address
    {
        rxddst_addr = rx_data[0];                                                               //store the received data in to local variables
        rxdsrc_addr = rx_data[1];
        rxdsq_id = rx_data[2];
        rxdcmd = rx_data[3];
        rxdchannel = rx_data[4];
        rxdsize = rx_data[5];
        for(i = 0;i < rxdsize;i++)
            k[i] = rx_data[PACKET_HEADER_SIZE-1+i];
        rxdchecksum = rx_data[PACKET_HEADER_SIZE-1+rxdsize];
        in_check = rxddst_addr+rxdsrc_addr+rxdsq_id+rxdcmd+rxdchannel+rxdsize;
        for(i = 0;i < rxdsize;i++)
            in_check += k[i];
        in_check = ~in_check;
        if(rxdchecksum == in_check)
        {
            if(rxdcmd & 0x80)                                                   // send ACK if requested
                {
                    temp[0] = rxdsq_id;
                    sendPackets(rxdsrc_addr, CMD_ACKNOWLEDGE, 0, 1, &temp[0]);
                    putsUart0("Sending ACK", true);
                 }
            if((rxdcmd&0x7F) == CMD_SET)                                       // process data if command received is set
            {
                if(rxdchannel == 6)
                    RED_LED = k[0];
                if(rxdchannel == 7)
                    GREEN_LED = k[0];
                if(rxdchannel == 8)
                    BLUE_LED = k[0];
                putsUart0("Received SET from ", false);
                putnUart0(rxdsrc_addr);
                putsUart0(", sequence id ", false);
                putnUart0(rxdsq_id);
                putsUart0(", channel ", false);
                putnUart0(rxdchannel);
                putsUart0(", value ", false);
                putnUart0(k[0]);
                putsUart0(" ", true);

            }
            if(((rxdcmd&0x7F) == CMD_RGB)&&(rxdchannel == 9))              // process data if command received is RGB
            {
                    RED_LED = k[0];
                    GREEN_LED = k[1];
                    BLUE_LED = k[2];
                putsUart0("Received RGB from ", false);
                putnUart0(rxdsrc_addr);
                putsUart0(", sequence id ", false);
                putnUart0(rxdsq_id);
                putsUart0(", channel ", false);
                putnUart0(rxdchannel);
                putsUart0(", value R ", false);
                putnUart0(k[0]);
                putsUart0(", value G ", false);
                putnUart0(k[1]);
                putsUart0(", value B ", false);
                putnUart0(k[2]);
                putsUart0(" ", true);
            }
            if(((rxdcmd&0x7F) == CMD_UART_DATA)&&(rxdchannel == 5))                          // process data if command received is UART COMMAND
            {
                putsUart0("Received UART DATA command from ",false);
                putnUart0(rxdsrc_addr);
                putsUart0(", sequence id ", false);
                putnUart0(rxdsq_id);
                putsUart0(", channel ", false);
                putnUart0(rxdchannel);
                putsUart0(" ", true);
                for(i = 0;i<rxdsize;i++)
                {
                    k[i] = rx_data[6+i];
                    st[i] = (char)k[i];
                }
                st[rxdsize] = 0;
                putsUart0(st,true);

            }
            if((rxdcmd&0x7F) == CMD_DATA_REQUEST)                                     // process data if command received is DATA REQUEST
            {
                putsUart0("Received data request from ", false);
                putnUart0(rxdsrc_addr);
                putsUart0(", sequence id ", false);
                putnUart0(rxdsq_id);
                putsUart0(", channel ", false);
                putnUart0(rxdchannel);
                putsUart0(" ", true);
                if(rxdchannel == 6)
                {
                    temp[0] = RED_LED;
                    sendPackets(rxdsrc_addr, CMD_DATA_REPORT, 0, 1, &temp[0]);
                }
                if(rxdchannel == 7)
                {
                    temp[0] = GREEN_LED;
                    sendPackets(rxdsrc_addr, CMD_DATA_REPORT, 0, 1, &temp[0]);
                }
                if(rxdchannel == 8)
                {
                    temp[0] = BLUE_LED;
                    sendPackets(rxdsrc_addr, CMD_DATA_REPORT, 0, 1, &temp[0]);
                }
                putsUart0("Sending data report to ", false);
                putnUart0(rxdsrc_addr);
                putsUart0(" ", true);

            }

            if((rxdcmd&0x7F) == CMD_DATA_REPORT)                                             // process data if command received is DATA REPORT
            {
                putsUart0("Received data report from ", false);
                putnUart0(rxdsrc_addr);
                putsUart0(", sequence id ", false);
                putnUart0(rxdsq_id);
                putsUart0(", channel ", false);
                putnUart0(rxdchannel);
                putsUart0(" ,value ", false);
                putnUart0(rx_data[6]);
                putsUart0(" ",true);
            }

            if((rxdcmd&0x7F) == CMD_ACKNOWLEDGE)                                             // process data if command received is ACK
            {
                for(i = 0;i<MAX_MESSAGES;i++)
                {
                    if(valid[i] && ackreq[i])
                    {
                        if(seq_id[i] == k[0])
                        {
                            valid[i] = false;
                            timeout[i] = 0;
                            retranscount[i] = 0;
                            ackreq[i] = false;
                            break;
                        }
                    }
                }
                putsUart0("Received ACK from ", false);
                putnUart0(rxdsrc_addr);
                putsUart0(", sequence id ", false);
                putnUart0(rxdsq_id);
                putsUart0(", value ", false);
                putnUart0(k[0]);
                putsUart0(" ", true);
            }
            if((rxdcmd&0x7F) == CMD_POLL_REQUEST)                                           // process data if command received is POLL REQUEST
            {
                temp[0] = SRC_ADDR;
                putsUart0("Received poll request from ", false);
                putnUart0(rxdsrc_addr);
                putsUart0(", sequence id ", false);
                putnUart0(rxdsq_id);
                putsUart0(", channel ", false);
                putnUart0(rxdchannel);
                putsUart0(" ", true);
                sendPackets(rxdsrc_addr, CMD_POLL_RESPONSE, 0, 1, &temp[0]);
                putsUart0("Sending poll response to ", false);
                putnUart0(rxdsrc_addr);
                putsUart0(" ", true);
            }
            if((rxdcmd&0x7F) == CMD_POLL_RESPONSE)                                         // process data if command received is POLL RESPONSE
            {
                putsUart0("Received poll response from ", false);
                putnUart0(rxdsrc_addr);
                putsUart0(", sequence id ", false);
                putnUart0(rxdsq_id);
                putsUart0(", channel ", false);
                putnUart0(rxdchannel);
                putsUart0(" ", true);
            }
            if((rxdcmd&0x7F) == CMD_RESET)                                                // process data if command received is RESET
            {
                putsUart0("Received RESET command from ", false);
                putnUart0(rxdsrc_addr);
                putsUart0(", sequence id ", false);
                putnUart0(rxdsq_id);
                putsUart0(", channel ", false);
                putnUart0(rxdchannel);
                putsUart0(" ", true);
                waitMicrosecond(500);
                reset_flag = true;                                                        // set reset flag
            }

            if((rxdcmd&0x7F) == CMD_SET_ADDRESS)                                          // process data if command received is SET ADDRESS
            {
                putsUart0("Received SA command from ", false);
                putnUart0(rxdsrc_addr);
                putsUart0(", sequence id ", false);
                putnUart0(rxdsq_id);
                putsUart0(", channel ", false);
                putnUart0(rxdchannel);
                putsUart0(" ", true);
                putsUart0("Changing address from ", false);
                putnUart0(SRC_ADDR);
                putsUart0(" to ", false);
                putnUart0(rx_data[6]);
                putsUart0(" ", true);
                EEPROM_EEOFFSET_R = 0;
                EEPROM_EERDWR_R = rx_data[6];
                SRC_ADDR = rx_data[6];
            }
            if(((rxdcmd&0x7F) == CMD_PIECEWISE)&&(rxdchannel == 8))                             // process data if command received is PIECEWISE
            {
                putsUart0("Received PIECEWISE command from ", false);
                putnUart0(rxdsrc_addr);
                putsUart0(", sequence id ", false);
                putnUart0(rxdsq_id);
                putsUart0(", channel ", false);
                putnUart0(rxdchannel);
                putsUart0(" ", true);
                signal = 1;
                N = k[0];
                for(i = 0;i<N;i++)
                    value[i] = k[i+1];
                dwell = (k[i+1]<<8)+k[i+2];
                i+=2;
                cycles = (k[i+1]<<8)+k[i+2];
                dwellcount = 0;
                TIMER2_CTL_R |= TIMER_CTL_TAEN;
            }
            if(((rxdcmd&0x7F) == CMD_PULSE)&&(rxdchannel == 8))                                    // process data if command received is PULSE
            {
                putsUart0("Received PULSE command from ", false);
                putnUart0(rxdsrc_addr);
                putsUart0(", sequence id ", false);
                putnUart0(rxdsq_id);
                putsUart0(", channel ", false);
                putnUart0(rxdchannel);
                putsUart0(" ", true);
                signal = 2;
                value[0] = k[0];
                dwell = (k[1]<<8)+k[2];
                dwellcount = 0;
                cycles = 1;
                TIMER2_CTL_R |= TIMER_CTL_TAEN;
            }
            if(((rxdcmd&0x7F) == CMD_SQUARE)&&(rxdchannel == 8))                                 // process data if command received is SQUARE
            {
                putsUart0("Received SQUARE command from ", false);
                putnUart0(rxdsrc_addr);
                putsUart0(", sequence id ", false);
                putnUart0(rxdsq_id);
                putsUart0(", channel ", false);
                putnUart0(rxdchannel);
                putsUart0(" ", true);
                signal = 3;
                value[0] = k[0];
                value[1] = k[1];
                t1 = (k[2]<<8)+k[3];
                t2 = (k[4]<<8)+k[5];
                cycles = (k[6]<<8)+k[7];
                dwellcount = 0;
                TIMER2_CTL_R |= TIMER_CTL_TAEN;
            }
            if(((rxdcmd&0x7F) == CMD_SAWTOOTH)&&(rxdchannel == 8))                                 // process data if command received is SAWTOOTH
            {
                putsUart0("Received SAWTOOTH command from ", false);
                putnUart0(rxdsrc_addr);
                putsUart0(", sequence id ", false);
                putnUart0(rxdsq_id);
                putsUart0(", channel ", false);
                putnUart0(rxdchannel);
                putsUart0(" ", true);
                signal = 4;
                value[0] = k[0];
                value[1] = k[1];
                delta = k[2];
                dwell = (k[3]<<8)+k[4];
                cycles = (k[5]<<8)+k[6];
                RED_LED = value[0];
                dwellcount = 0;
                TIMER2_CTL_R |= TIMER_CTL_TAEN;
            }
            RX_LED = 0;
        }
        else
           {
               RX_LED = 1;                              //step 10 rx led goes solid if checksum error occurs
               putsUart0("Received bad checksum", true);
           }
    }

}

//function that stores the message to be transmitted in tx message
void storeTxMessage(uint8_t k)
{
    uint8_t i;
    current_phase = 0;
    tx_message[0] = dst_addr[k];
    tx_message[1] = SRC_ADDR;
    tx_message[2] = seq_id[k];
    tx_message[3] = cmd[k];
    tx_message[4] = channel[k];
    tx_message[5] = size[k];
    for(i = 0;i < size[k];i++)
        tx_message[i+6] = data[current_index][i];
    tx_message[5+size[k]+1] = checksum[k];
}

//function to update retransmission count
void updateTimeout()
{
    uint8_t j;
    for(j = 0;j < MAX_MESSAGES;j++)
              if(timeout[j] > 0)
                  timeout[j]--;
}

//
void updateTxLed()
{
    TX_LED = 1;
    TX_LED_TIMEOUT++;
    if(TX_LED_TIMEOUT>LED_TIMEOUT)
    {
        TX_LED = 0;
        TX_LED_TIMEOUT = 0;
    }
}

//
void transmit(uint8_t i)
{
    if(in_progress)
    {
        if(CS_ENABLE)                                                    //check RX FIFO before tx if cs is enabled
            if(!(UART1_FR_R & UART_FR_RXFE))
            {
                putsUart0("Line busy", true);
                TIMER1_ICR_R = TIMER_ICR_TATOCINT;
                return;

            }
        D_EN = 1;
        if(current_phase == 0)
        {
            UART1_LCRH_R &= ~UART_LCRH_EPS;                                 // clear EPS for first byte parity to be high
            current_eps = false;
            if(!(UART1_FR_R & UART_FR_BUSY))
            {
                UART1_DR_R = tx_message[current_phase];
                updateTxLed();
                in_tx = true;
            }
            current_phase++;
            if(!(UART1_FR_R & UART_FR_BUSY))
            return;
        }
        if(!(UART1_FR_R & UART_FR_BUSY))
        {
            UART1_LCRH_R |= UART_LCRH_EPS;                                // set EPS for rest of the bytes
            current_eps = true;
            while(!(UART1_FR_R & UART_FR_TXFF))
            {
                //TX_LED = 1;
                UART1_DR_R = tx_message[current_phase];
                current_phase++;
                updateTxLed();
                if(current_phase > (size[i]+PACKET_HEADER_SIZE-1))
                {
                    in_progress = false;
                    in_tx = false;
                    if(!ackreq[i])                                            //invalidate the message if ack not required
                    {
                        valid[i] = false;
                        current_phase = 0;
                        TX_LED = 0;
                    }
                    else
                    {
                        current_phase = 0;
                        retranscount[i]++;
                        putsUart0("Transmitting message ", false);
                        putnUart0(seq_id[i]);
                        //putsUart0("\n\r", false);
                        putsUart0(", Attempt ", false);
                        putnUart0(retranscount[i] - 1);
                        putsUart0("\n\r", false);
                    }
                    if(retranscount[i] >= MAX_RETRANSCOUNT)
                    {
                        valid[i] = false;
                        retranscount[i] = 0;
                        current_phase = 0;
                        timeout[i] = 0;
                        n[i] = 0;                                    //make the incrementing variable 0
                        TX_LED = 1;                                  //step 10 tx led goes solid if checksum error occurres
                        putsUart0("Error sending message ", false);
                        putnUart0(seq_id[i]);
                        putsUart0("\n\r", false);
                        return;
                    }
                    else
                    {
                        if(RANDOM)
                        {
                            timeout[i] = T+randtb[randptr]*T0;
                            randptr = (randptr+1)%randtbsize;
                        }
                        else
                        {
                            n[i]++;
                            timeout[i] = T+((2^n[i])*T0);
                        }
                    }
                    TX_LED = 0;
                    return;
                }
            }
        }
    }
}

//function to monitor TX deadlock
void updateTxDeadlock()
{
    if((old_current_phase == current_phase)&&(current_phase != 0))
    {
        tx_deadlock_count++;
        if(tx_deadlock_count > DEADLOCK_TIMEOUT)
        {
            current_phase = 0;
            in_progress = false;
            tx_deadlock_count = 0;
            putsUart0("exiting tx due to dead lock",true);
        }
    }
    else
    {
        tx_deadlock_count = 0;
        old_current_phase = current_phase;
    }
}

// function to monitor RX deadlock
void updateRxDeadlock()
{
    if((old_rx_phase == rx_phase)&&(rx_phase != 0))
    {
        rx_deadlock_count++;
        if(rx_deadlock_count > DEADLOCK_TIMEOUT)
        {
            rx_phase = 0;
            rx_deadlock_count = 0;
            putsUart0("exiting rx due to dead lock",true);
        }
    }
    else
    {
        rx_deadlock_count = 0;
        old_rx_phase = rx_phase;
    }
}

//function to update Rx KED
void updateRxLed()
{
    RX_LED_TIMEOUT++;
    if(RX_LED_TIMEOUT>LED_TIMEOUT)
    {
        RX_LED = 0;
        RX_LED_TIMEOUT = 0;
    }
}

//function for receiving data bytes from RS485
void receive()
{
    bool parity_true = false;
    uint16_t n;
    while(!(UART1_FR_R & UART_FR_RXFE))
    {
        in_rx = true;
        n = UART1_DR_R;
        RX_LED = 1;
        if(current_eps)
        {
            if(n & 0x200)
                parity_true = true;
            else
                parity_true = false;
        }
        else
        {
            if(!(n & 0x200))
                parity_true = true;
            else
                parity_true = false;
        }
        if(parity_true)
        {
            rx_phase = 0;
            rx_data[rx_phase] = n & 0xFF;
            if(rx_data[0] != SRC_ADDR)
            {
                while(!(UART1_FR_R & UART_FR_RXFE))
                {
                    n = UART1_DR_R;
                }
            }
            if((rx_data[0] == SRC_ADDR) || (rx_data[0] == BROADCAST_ADDR))
            {
                rx_phase++;
                RX_LED = 1;
                updateRxLed();
            }
        }
        else if(rx_phase!= 0)
        {
            rx_data[rx_phase++] = n & 0xFF;
            RX_LED = 1;
            updateRxLed();
        }
        if(rx_phase == (rx_data[5]+PACKET_HEADER_SIZE))
        {
            in_rx = false;
            rx_phase = 0;
            processPacket();                                     //call process message
            return;
        }
        RX_LED = 0;
    }
}

//Timer 1 Interrupt Service Routine
void Timer1Isr(){
    uint8_t i;
    i = 0;

    if(!in_progress)
    {
        while((!valid[i])||(timeout[i] != 0))
        {
            i++;
            if(i > MAX_MESSAGES-1)
            {
                if(reset_flag && !D_EN)                                                                //check if reset command was issued
                    HWREG(NVIC_APINT) = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;         // if reset cmd received then reset if no commands are left to transmit
                goto receive;
            }
        }

        in_progress = true;
        in_tx = true;
        current_index = i;
        storeTxMessage(current_index);
    }

    transmit(current_index);
    receive:updateTxDeadlock();
    receive();
    updateRxDeadlock();
    updateTimeout();
    if(!(UART1_FR_R & UART_FR_BUSY))
        D_EN = 0;
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;                           //important to clear interrupt flag of timer 1
}

//timer 2 defined for generating signals like pulse square and sawtooth
void Timer2Isr()
{
    if(signal == 1)
    {
        if(cycle_count<cycles)
        {
            if(dwellcount<=dwell)
                dwellcount++;
            else
            {
                RED_LED = value[valptr++];
                dwellcount = 0;
                if(valptr > N)
                {
                    cycle_count++;
                    valptr = 0;
                }
            }
        }
        else
        {
            valptr = 0;
            cycle_count = 0;
            TIMER2_ICR_R = TIMER_ICR_TATOCINT;                           //important to clear interrupt flag of timer 2
            TIMER2_CTL_R &= ~TIMER_CTL_TAEN;
        }
    }

    if(signal == 2)
        {
            if(cycle_count<cycles)
            {
                RED_LED = value[0];
                if(dwellcount<=dwell)
                    dwellcount++;
                else
                {
                    RED_LED = 0;
                    cycle_count++;
                }
            }
            else
            {
                valptr = 0;
                cycle_count = 0;
                TIMER2_ICR_R = TIMER_ICR_TATOCINT;                           //important to clear interrupt flag of timer 2
                TIMER2_CTL_R &= ~TIMER_CTL_TAEN;
            }
        }
    if(signal == 3)
        {
            if(cycle_count<cycles)
            {
                if(dwellcount<t1)
                    RED_LED = value[0];
                else if(dwellcount>t1)
                    RED_LED = value[1];
                if(dwellcount <t1+t2)
                    dwellcount++;
                else
                {
                    RED_LED = 0;
                    cycle_count++;
                    dwellcount = 0;
                }
            }
            else
            {
                valptr = 0;
                cycle_count = 0;
                dwellcount = 0;
                TIMER2_ICR_R = TIMER_ICR_TATOCINT;                           //important to clear interrupt flag of timer 2
                TIMER2_CTL_R &= ~TIMER_CTL_TAEN;
            }
        }
    if(signal == 4)
        {
            if(cycle_count<cycles)
            {
                    if(dwellcount<=dwell)
                        dwellcount++;
                    else
                    {
                        if(delta>0)
                            RED_LED += delta;
                        else
                            RED_LED -= delta;
                        dwellcount = 0;
                        if(((RED_LED>(value[1]-delta))&&(delta>0))||((RED_LED<(value[1]+delta))&&(delta<0)))
                        {
                            cycle_count++;
                            RED_LED = value[0];
                        }
                    }
            }
            else
            {
                valptr = 0;
                cycle_count = 0;
                dwellcount = 0;
                RED_LED = 0;
                TIMER2_ICR_R = TIMER_ICR_TATOCINT;                           //important to clear interrupt flag of timer 2
                TIMER2_CTL_R &= ~TIMER_CTL_TAEN;
            }
        }
    TIMER2_ICR_R = TIMER_ICR_TATOCINT;                           //important to clear interrupt flag of timer 2
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    // Initialize hardware
    initHw();
    initTr1();
    initTr2();
    EEPROM_EEOFFSET_R = 0;
    if(EEPROM_EERDWR_R != 0xFFFFFFFF)
    {
        __asm("  NOP");
        __asm("  NOP");
        __asm("  NOP");
        __asm("  NOP");
    EEPROM_EEOFFSET_R = 0;
    SRC_ADDR = EEPROM_EERDWR_R;                           // obtain source address from EEPROM
    }
    putsUart0("SOURCE_ADDRESS is ", false);
    putnUart0(SRC_ADDR);
    putsUart0("", true);
////////////////////////////////////START SEQUENCE/////////////////////////////////////////////////
    // Turn on green LED
    TX_LED = 0;
    RX_LED = 0;
    RX_LED = 1;

    // Wait for 500ms
    waitMicrosecond(500000);

    // Turn off green LED
    RX_LED = 0;

    // Wait for 500ms
    waitMicrosecond(500000);
    putsUart0("READY", true);
//////////////////////////////////////////////////TEST/////////////////////////////////////////////

   while(1)
   {
     getsUart0();
     putsUart0(str, true);
     parseCommand(str);
     findCmd();
   }
}
