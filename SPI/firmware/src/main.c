
    unsigned char count = 0;    
    unsigned char chrs = 0;
    unsigned char SPIRX = 0;
#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>
#include <proc/p32mx795f512l.h>                     // Defines EXIT_FAILURE
#include "system/common/sys_module.h"   // SYS function prototypes



#define ENCa PORTDbits.RD2  // pin 6
#define ENCb PORTEbits.RE9  // pin 7

#define sw1 PORTBbits.RB0 //        A0
#define sw2 PORTBbits.RB3 //        A3



// SPI Master  SPI2    output bus
//#define MISO        LATGbits.LATG7              //PIN 29 not used
//#define MISOIN      PORTGbits.RG7       
//#define MISOTRIS    TRISGbits.TRISG7            //PIN 29 not used
// MISO pin Master IN   SPI2
#define MOSI     LATGbits.LATG8              //PIN 43 diode to 17
#define MOSIIN        PORTGbits.RG8
#define MOSITRIS    TRISGbits.TRISG8            //PIN 43
// Clock pin            
#define CLK         LATGbits.LATG6              //PIN 52 diode to 14
#define CLKIN       PORTGbits.RG6
#define CLKTRIS     TRISGbits.TRISG6            //PIN 52

// SPI Slave SPI4   input bus
//#define CLK       PORTF13bits.RF13            //PIN 14
//#define MOSI      LATFbits.LATF5              //PIN 16
#define MISO      PORTFbits.RF4                 //PIN 17 
#define MISOIN    PORTFbits.RF4
#define MISOTRIS  TRISFbits.TRISF4              // pin 17
// handshake pin
#define HS         LATAbits.LATA14             //PIN 21

//#define HSIN        PORTGbits.RG9
#define HSIN        PORTAbits.RA14              //PIN 21
//#define HSTRIS      TRISGbits.TRISG9
#define HSTRIS      TRISAbits.TRISA14           //PIN 21

// Handhaske Diretion pin // opamp on interposer board not needed for production debug use only.
#define HSDIR       PORTAbits.RA15              //PIN 20
#define HSDIRTRIS   TRISAbits.TRISA15           //PIN 20

#define ERROR_PIN0  LATBbits.LATB11             // PIN 40
#define ERROR_PIN1  LATBbits.LATB12             // PIN 42
#define ERROR_PIN2  LATAbits.LATA10             // PIN 44
#define ERROR_PIN3  LATFbits.LATF1              // PIN 46
#define ERROR_PIN4  LATDbits.LATD8              // PIN 48


#define baud96k 420
#define baud156k 255 //255
unsigned char bitcount = 9;     // when this is 9 all actions complete 0 means we need to do something with the data 8-1 capture bits
// used for SPI mode 0,1 and 2,3
unsigned char SPIINH = 0;    // stores the bits read when CLK is HIGH  
unsigned char SPIINL = 0;    // stores the bits read when CLK is LOW 

unsigned char CLKSTATE = 0;
unsigned char HSSTATE = 0;
unsigned char HSDIRSTATE = 0;
unsigned char speed = 0;
unsigned char temp = 0;
unsigned char tempb = 0;
unsigned char hextemp;
unsigned int DELAY = 0;
unsigned char TX_BUFFER[128];    // used to store the data to be transmitted
unsigned char GOTSPI = 0;
// ie the location to store the next byte
unsigned char bcount = 1;       // used to keep track of the number of bytes being transfered 
// ie the location of the next byte to be sent
unsigned char bpos = 1;         // current position in the buffer

unsigned int packet_count = 0;
#define trig_packet 24
unsigned int dx,dt;        // used to create delays
    
unsigned char encastop; // flag to stop sending enca signals
unsigned char encbstop; // flag to stop sending encb signals

unsigned char encastate;
unsigned char encbstate;

unsigned char sw2clear =0 ;

void check_n_send_data_source(void);
void send_hex(unsigned char inp);
void HS_EDGE(void);
void CLK_EDGE(void);
void dla100(void);
void dla1600(void);
unsigned char init156k(void);
unsigned char init96k(void);
void SPI_PIN_MASTER(void);
void SPI_PIN_SLAVE(void);
void scratchnsniff_SPI(void);           // service routine that reads spi and sends framed data to the usart
void send_SPI(unsigned char out);       // sends a byte out the SPI port bit bang style

unsigned char addtobuf(unsigned char ); // loads a byte on to the usart buffer
void dla1bit(void);
void init_spihw4(void);
void init_spihw2(void);
void dla5ms(unsigned int);
void dlaspi(void);
void enctest(void);
unsigned char read_SPI(unsigned char num);  // waits while we read x number of bytes from the bus.
unsigned char x = 0;
int main ( void )
{
  
    /* Initialize all MPLAB Harmony modules, including application(s). */
    SYS_Initialize ( NULL );    // sets up Open drain pins and usart0
    HSTRIS      = 1;
    CLKTRIS     = 1;
    MISOTRIS    = 1;
    HSTRIS      = 0;
    HS          = 0;
    CLK         = 1;
    MISO        = 1;

    TRISA = 0x00;   // all outputs
    PORTA = 0;  

    TRISBbits.TRISB11 = 0;      // error leds
    TRISBbits.TRISB12= 0;    
    TRISAbits.TRISA10 = 0;
    TRISFbits.TRISF1  = 0;
    TRISDbits.TRISD8 = 0;       // error leds
    
    TRISBbits.TRISB0 = 1;       //sw1 input active low
    TRISBbits.TRISB3 = 1;       //sw2 input active low

    HSDIRTRIS = 1;
    HSDIRSTATE = HSDIR;
    HSSTATE = HSIN;
    CLKSTATE = CLKIN;
    TX_BUFFER[0] = 0xff;      
    ERROR_PIN0 = 0;
    ERROR_PIN1 = 0;
    ERROR_PIN2 = 0;
    ERROR_PIN3 = 0;
    ERROR_PIN4 = 0;
    packet_count = 0;
    encastop = 5;
    encbstop = 5;

    addtobuf('S');
    addtobuf('t');
    addtobuf('a');
    addtobuf('r');
    addtobuf('t');
    addtobuf(0x0d);
    addtobuf(0x0a);
    
    x = 0xe0;

init_spihw4();               // sets up SPI2 & 4 (4 is ued to read & sniff) 2 used to send
init_spihw2();
      
    while ( true )
    {
        if (sw1 == 0)
        if (sw2clear == 1)
        {
          // init96k();              
           //send_SPI(0x28);
            SPI2BUF = 0x28;
           //while(1);
           //HSTRIS = 1;
           //HS = 1;       
           //read_SPI(44);           
            sw2clear = 0;
            }
        if (sw2 == 0)
        {          
            sw2clear = 1;
            ERROR_PIN2 = 0;            
        }
        else
        {
            ERROR_PIN2 = 1;                       
        }
      
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks ( );              // DONT think this does any thing was added by harmony config (we dont read the usart)
        //scratchnsniff_SPI();       // this will read spi if in slave mode and send in hex to usart with lens/cam framing    
        //HS_EDGE();
 
        // this code dumps the usart buffer to the usart non blocking
        if (bcount != bpos)  // if the counter is not equal to the current position we need to send the chars 
        {
        if( DRV_USART0_TransmitBufferIsFull() == 0)
            {
            DRV_USART0_WriteByte(TX_BUFFER[bpos]);
                bpos++;          
            }
        }
        if (bpos >= 120)
            {
                bpos = 1;
            }

    }
    /* Execution should not come here during normal operation */    
    return ( EXIT_FAILURE );
}
/*
unsigned char read_SPI(unsigned char num)
{  
    chrs = 0;           //chars received counter gets incremented in ISR's
//    HSTRIS = 1;
//    HS = 1;
while(SPI4STATbits.SPIBUSY == 1)
    {
        temp++;
    }
ERROR_PIN4 = 1;


    while (chrs < num)
    {       
      
        while(HSIN == 1); // wait for HS to go low pulled by lens if it wants to send a reply        
        //SPI4CONbits.DISSDO = 1; // might not be needed if we clock out zeros on the databus
        while(SPI4STATbits.SPIBUSY == 1);
        SPI4BUF = 0xff;         // clock out dummy data with SDO pin disabled to allow reading of the SDI pin        
        while(HSIN == 0); // wait until the line is released                 
       chrs++;
    }
     
    SPI4CONbits.DISSDO = 0;
    return count;
}
*/
void init_spihw4(void)
{
    SPI4BRG = baud96k;
    // SPI 4
    TRISFbits.TRISF13 = 1;  // clock3   J4-8 
    TRISFbits.TRISF4  = 1;  // SDI3     J4-5
    TRISFbits.TRISF5  = 1;  // SDO3     J4-6
    TRISAbits.TRISA14  = 1;  // HW handshake line
    // SPI HW 4 reads data from the bus.
    IPC8bits.SPI4IP   = 1;
    IPC8bits.SPI4IS   = 0;
    IFS1bits.SPI4RXIF = 0;
    IEC1bits.SPI4RXIE = 1;
    SPI4STAT = 0;
    SPI4CON = 0;
    SPI4CON = 0;
    SPI4CONbits.ENHBUF  = 0;        // hardware 8 byte buffer
    SPI4CONbits.CKP = 1;            // clock polarity should be 1 to read properly.
    SPI4CONbits.CKE = 0;            // should be 0 to read the camera properly
    SPI4CONbits.MSTEN = 1;          // slave mode
    SPI4CONbits.STXISEL = 00;       // TX interrupt 
    SPI4CONbits.SRXISEL = 01;       // RX interrupt when we have anything in the buffer
    SPI4CONbits.MODE16 = 0;
    SPI4CONbits.MODE32 = 0;    
    SPI4CONbits.ON = 1;         // ON/OFF          
}

void init_spihw2(void)
{
//    TRISGbits.TRISG8 = 0;   // MOSI2
//    TRISGbits.TRISG6 = 0;   // CLK2
//    LATGbits.LATG6 = 1;
//    LATGbits.LATG8 = 1;
    MOSITRIS = 0;
    CLKTRIS = 0;    
       // SPI HW 2 used to send data to the lens
    SPI2BRG = baud96k;      // 420= 96K2hz 255 = 156k    
    IPC7bits.SPI2IP   = 1;
    IPC7bits.SPI2IS   = 1;
    IFS1bits.SPI2RXIF = 0;
    IEC1bits.SPI2RXIE = 0;
    SPI2STAT = 0;
    SPI2CON = 0;
    SPI2CONbits.ENHBUF  = 0;        // hardware 8 byte buffer
    SPI2CONbits.CKP = 1;            // clock polarity
    SPI2CONbits.CKE = 0;            // clock edge
    SPI2CONbits.SMP = 0;
    SPI2CONbits.MSTEN = 1;          // master mode to send stuff
    SPI2CONbits.STXISEL = 00;       // TX interrupt 
    SPI2CONbits.SRXISEL = 00;       // RX interrupt when we have anything in the buffer
    SPI2CONbits.MODE16 = 0;
    SPI2CONbits.MODE32 = 0;    

    SPI2CONbits.ON = 1;         // ON/OFF    
    SPI2CONbits.DISSDO = 0;

    
}

void send_SPI(unsigned char out)
{   
    hextemp = 0;
    if ((out & 0x80) == 0x80)
                      hextemp = 0x01;
    if ((out & 0x40) == 0x40)
            hextemp = hextemp + 0x02;
    if ((out & 0x20) == 0x20)
            hextemp = hextemp + 0x04;
    if ((out & 0x10) == 0x10)
            hextemp = hextemp + 0x08;
    
    if ((out & 0x08) == 0x08)
            hextemp = hextemp + 0x10;
    if ((out & 0x04) == 0x04)
            hextemp = hextemp + 0x20;
    if ((out & 0x02) == 0x02)
            hextemp = hextemp + 0x40;
    if ((out & 0x01) == 0x01)
            hextemp = hextemp + 0x80;  

  
    SPI2CONbits.DISSDO  = 0;
    ERROR_PIN4 = 1;
    HS = 1;     // ensure HSline is high
    HSTRIS = 0; // make handshake line output
    HS = 0;     // pull HS line low
    SPI4BUF = hextemp;
    ERROR_PIN4 = 0;
//HSTRIS = 1;
    HS = 1;
    TRISAbits.TRISA14 = 1;

    temp = SPI4STATbits.SPIBUSY;

    while(temp == 1)
    {
        temp = SPI4STATbits.SPIBUSY;
    }
   ERROR_PIN4 = 1;
    temp = PORTAbits.RA14;
    while (temp == 0)
    {
        temp = PORTAbits.RA14;
    }
    temp =0;
}


void send_hex(unsigned char inp)    // formats a byte into hex ascii and sends out usart
{
encastop = 0;
encbstop = 0;       
    hextemp = 0;    
//inp = 0x12; 
    
    if ((inp & 0x80) == 0x80)
                      hextemp = 0x01;
    if ((inp & 0x40) == 0x40)
            hextemp = hextemp + 0x02;
    if ((inp & 0x20) == 0x20)
            hextemp = hextemp + 0x04;
    if ((inp & 0x10) == 0x10)
            hextemp = hextemp + 0x08;
    
    if ((inp & 0x08) == 0x08)
            hextemp = hextemp + 0x10;
    if ((inp & 0x04) == 0x04)
            hextemp = hextemp + 0x20;
    if ((inp & 0x02) == 0x02)
            hextemp = hextemp + 0x40;
    if ((inp & 0x01) == 0x01)
            hextemp = hextemp + 0x80;        
   
inp = hextemp;
// end of LSB > MSB flip.

    addtobuf(' ');
    addtobuf('0');
    addtobuf('x');
    hextemp = inp & 0xf0;
    hextemp = (hextemp >> 4);   
    if (hextemp <= 9)
    {
        hextemp = hextemp + 48;
    }
    else
    {
        hextemp = hextemp + 55;
    }
    addtobuf(hextemp);     // sends high nibble out usart 
    hextemp = inp & 0x0f;
    if (hextemp <= 9)
    {
        hextemp = hextemp + 48;
    }
    else
    {
        hextemp = hextemp + 55;
    }
    addtobuf(hextemp);      // sends lower nibble out the usart
    addtobuf(' ');
}


unsigned char init96k(void)
{
    SPI2BRG = baud96k;      // 420=96K2hz 255 = 156k  
    HSTRIS = 0;     // output
    HS = 0;
    dla1600();
    HSTRIS = 1;     // input
    HS = 1;         // set high so the other end can pull low if ready.
    dla5ms(4900);       // 
    return HSIN;    // will be 0 if all good otherwise 1 = dev not ready
}

unsigned char init156k(void)
{
    SPI2BRG = baud156k;      // 420=96K2hz 255 = 156k  
    HSTRIS = 0;     // output
    HS = 0;
    dla100();
    HSTRIS = 1;     // input
    HS = 1;     // release the line 
    dla5ms(100);    // give the lens a chance to NAK the packet/byte req
    return HSIN;
}

void dlaspi(void)
{
     DELAY = 0;
    for (dx = 0;dx <= 1000 ;dx++)
    {
        DELAY = 1;
    }   
}

void dla5ms(unsigned int t)
{  // routine to complete the 5ms delay, t is the number of us that has already passed.
   //
    temp = 5000 - t;        // t is micro seconds(us)
    for (dt = ((5000-t)/100);dt>0;dt--)
    for (dx = 0;dx <= 4000 ;dx++)
    {
        DELAY = 1;
    }
    
}

void dla100(void)       //confirmed 100us
{   // creates a 100us delay 

    DELAY = 0;
    for (dx = 0;dx <= 4000 ;dx++)
    {
        DELAY = 1;
    }
    // the PIC uP is executing 12M instructions per second
    // 118 for loops = 100us    
}

void dla1600(void)      // confirmed 1600us
{   // creates a 1600us delay 
    
    for (dt = 0;dt <= 64000 ;dt++)
    {
        DELAY = 1;
    }
}

void dla1bit(void)
{
     unsigned int dy;
    DELAY = 0;
    for (dy = 0;dy <= 30 ;dy++)
    {
        DELAY++;
    }   
}

// was used to sniff the spi bus bit banging style, not required any more
void scratchnsniff_SPI(void)
{
//  slave mode sniffer for spi bus. will dump sender and data to usart          

        HS_EDGE();          // do stuff on the HS transitions
                            // ie packet started and finished 
                               // send the packet on completion or reset counters on start
                               // also frame the data bytes with the sender
    //    CLK_EDGE();         // do stuff on the CLK transitions
                            // bits being prepared or ready

}

void enctest(void)
{
           // adds chars to the USART when the either of the encoder signals change.
        if (ENCa != encastate)
            if (encastop < 5)
        {
            addtobuf('a');
            addtobuf('a');
            addtobuf('a');            
            encastate = ENCa;
            encastop++;
        }
        if (ENCb != encbstate)
            if (encbstop < 5)
        {
            addtobuf('b');
            addtobuf('b');
            addtobuf('b');   
            encbstate = ENCb;
            encbstop++;
        }
}

unsigned char addtobuf(unsigned char byt)
{
        bcount++;
        TX_BUFFER[bcount] = byt;
    if (bcount >= 120)
    {
        bcount = 1;
    }
}

void HS_EDGE(void)      // checks the handshake line and starts reading bits  
{
    HSTRIS = 1; // make sure the pin is an input... 
        if (HSIN != HSSTATE)             // HS line goes low when data is on or will be on the bus.
        {
            check_n_send_data_source();
            // on the edged of the handshake line we send the level of the HSDIR pin 
            // once on each edge the data will be between these handles
            // eg CAM 0x0f CAM LENS 0x04 LENS == camera sending 0x0f the lens responding 0x04           

            if (HSIN == 0)
                {                        
                if (packet_count == trig_packet)
                {
                    ERROR_PIN3 = 1;  
                    addtobuf('X');
                    addtobuf('X');
                    addtobuf('X');
                    addtobuf(' ');
                }
            packet_count++;
            }                    
            HSSTATE = HSIN;      // store the last state of the pin so we only do this on the edges.
        } 
    HSTRIS = 0;
    HS = 1;
}

void check_n_send_data_source(void)
{
    if (HSDIRSTATE != HSDIR)      // remark this line to have LENS-DATA-LENS framing
    {                             // otherwise its only on one edge to reduce thruput on usart
        if (HSDIR == 0)            
            {
                addtobuf(0x0d);
                addtobuf(0x0a);
                addtobuf('C');
                addtobuf('A');
                addtobuf('M');
                addtobuf(0x0d);
                addtobuf(0x0a);     
            }
        if (HSDIR == 1)             
            {
                addtobuf(0x0d);
                addtobuf(0x0a);
                addtobuf('L');
                addtobuf('E');
                addtobuf('N');
                addtobuf('S');
                addtobuf(0x0d);
                addtobuf(0x0a);
             }
        HSDIRSTATE = HSDIR;
    }   
}
