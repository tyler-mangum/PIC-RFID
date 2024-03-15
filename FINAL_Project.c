/*
 * File:   RFIDPROJECT.c
 * Author: trman
 *
 * Created on April 10, 2023, 12:32 PM
 */


// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = OFF      // PLL Enable (4x PLL disabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = ON        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)


#define I2C_SLAVE_HI 0x27
#define I2C_WRITE 0
#define I2C_READ 1
#define BAUD 9600      
#define FOSC 4000000L
#define DIVIDER ((int)(FOSC/(16UL * BAUD) -1))
#define NINE_BITS 0
#define SPEED 0x4
#define RX_PIN TRISC5
#define TX_PIN TRISC4
#define _XTAL_FREQ 4000000.0    /*for 4mhz*/

#include <xc.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include  "i2c.h"
#include  "i2c_LCD.h"


/* Serial initialization */
void init_comms(void);
void putch(unsigned char);
unsigned char getch(void);
unsigned char getche(void);

//Prototypes
void I2C_LCD_Command(unsigned char,unsigned char);
void I2C_LCD_SWrite(unsigned char,unsigned char *, char);
void I2C_LCD_Init(unsigned char);
void I2C_LCD_Pos(unsigned char,unsigned char);
unsigned char I2C_LCD_Busy(unsigned char);

//Global Variables 
int PWM_freq ;//0 For left, 1 for right, 2 for center
unsigned char Dummy, Buff[10][4], Rxin;

void interrupt UART_Rx (){   
    Rxin = RCREG;
    return;
}

void output_pwm()
{
    if(PWM_freq==0){
        RA5=1;
        __delay_ms(0.5);
        RA5=0;
        __delay_ms(19.5);
    }
    else if (PWM_freq==1){
        RA5=1;
        __delay_ms(2.5);
        RA5=0;
        __delay_ms(17.5);
     
    }
    else if (PWM_freq==2){
        RA5=1;
        __delay_ms(18.5);
        RA5=0;
        __delay_ms(1.5);
     
    }
}


/*  ********************************************************************* */
void main(void) {

unsigned char Dummy, Treg, Rx, Sout[16],Sout2[16];
unsigned char * Sptr;
Sptr = Sout;  // This is a buffer for the Sprintf command to use
int count,i,j;
int Pos_1,Pos_2,Pos_3,Pos_4,Pos_5,Pos_6,Pos_7,Pos_8;
int case_var=0;

//Define Valid Tags
int tagA_1=56;
int tagA_2=49;
int tagA_3=65;
int tagA_4=48;
int tagA_5=66;
int tagA_6=53;
int tagA_7=50;
int tagA_8=52;
    OSCCON  = 0x68; // b6..4 = 1101 = 4MHz 
    APFCON0	= 0x84;  //makes RC4 & 5 TX & RX for USART 
    ANSELA   = 0x00;    // all ADC pins to digital I/O 
    ANSELC	= 0x80;
    TRISC = 0xFF; 
    TRISB = 0xF0;
    ANSELB = 0X00;
    TRISA = 0x10;	//make RA4 input for ping echo; note: RA2 is pulse
    LATA = 0x00;   
    LATC = 0x00;   
    TXCKSEL = 1;   
    RXDTSEL = 0;  //makes RC4 & RB5 TX & RX for USART (Allows ICSP)
    init_comms();
    RA2=1; 
    RA5=1;
    RC6=1;
    //Set up interrupts
    RCIE = 1;        //Sets interrupt for receive on UART
    INTCON |= 0X40;  //Sets GIE and PEIE  bits 6 and 7	
    printf("Start\r\n");
        //Now parse for found PN532 reader on power up of the Arduino @=ok *=No 	
	 
      Rx = getch();
      GIE = 1;  //Now turn on interrupts so we don't miss
      if(Rx == '*'){
          printf("No PN532\r\n");
          return;
      }
      else if (Rx == '@')printf ("PN532 found\r\n");
      else {
          printf("Error Halt\r\n");
          while(1)continue;
      }
        //wait here till ready for tags
          
    
      while(1) {   
    //Now parse for $ to save tag
         PWM_freq=0;
         if (case_var==1){
         i2c_Init();				// Start I2C as Master 100KH
         I2C_LCD_Init(I2C_SLAVE_HI); //pass I2C_SLAVE to the init function to create an instance
         I2C_LCD_Command(I2C_SLAVE_HI, 0x01);
         I2C_LCD_Pos(I2C_SLAVE_HI,0x00);
         sprintf(Sout, "Access Granted");
         I2C_LCD_SWrite(I2C_SLAVE_HI, Sout, 14);
         PWM_freq=0;
            for (int z=0;z<100;z++){
                output_pwm();
            }
         __delay_ms(3000);
         PWM_freq=1;
            for (int x=0;x<100;x++){
                output_pwm();
            }
         case_var=0;
         i2c_Init();				// Start I2C as Master 100KH
         I2C_LCD_Init(I2C_SLAVE_HI); //pass I2C_SLAVE to the init function to create an instance
         I2C_LCD_Command(I2C_SLAVE_HI, 0x01);
         I2C_LCD_Pos(I2C_SLAVE_HI,0x00);
         sprintf(Sout, "Scan RFID Tag");
         I2C_LCD_SWrite(I2C_SLAVE_HI, Sout, 13);
         }
         else {
         i2c_Init();				// Start I2C as Master 100KH
         I2C_LCD_Init(I2C_SLAVE_HI); //pass I2C_SLAVE to the init function to create an instance
         I2C_LCD_Command(I2C_SLAVE_HI, 0x01);
         I2C_LCD_Pos(I2C_SLAVE_HI,0x00);
         sprintf(Sout, "Invalid RFID Tag");
         I2C_LCD_SWrite(I2C_SLAVE_HI, Sout, 16);    
         __delay_ms(2000);
         i2c_Init();				// Start I2C as Master 100KH
         I2C_LCD_Init(I2C_SLAVE_HI); //pass I2C_SLAVE to the init function to create an instance
         I2C_LCD_Command(I2C_SLAVE_HI, 0x01);
         I2C_LCD_Pos(I2C_SLAVE_HI,0x00);
         sprintf(Sout, "Scan RFID Tag");
         I2C_LCD_SWrite(I2C_SLAVE_HI, Sout, 13);
         PWM_freq=1;
            for(int w=0;w<100;w++){
                output_pwm();
            }
         }
         
         
          while(Rxin != '$')continue;  //Stay here till tag detected
          GIE =0;  //Turn off interrupts use polling
          Dummy = getch(); //read the CR LF characters
          Dummy = getch();
	      for (i=0; i<4 ; i++){
              for(j=0;j<4;j++){
                  Buff[i][j] = getch();
              }
          } 
          Pos_1=Buff[0][2];
          Pos_2=Buff[0][3];
          Pos_3=Buff[1][2];
          Pos_4=Buff[1][3];
          Pos_5=Buff[2][2];
          Pos_6=Buff[2][3];
          Pos_7=Buff[3][2];
          Pos_8=Buff[3][3];
          if (Pos_1==tagA_1 && Pos_2==tagA_2 && Pos_3==tagA_3
                  && Pos_4==tagA_4 && Pos_5==tagA_5 && Pos_6==tagA_6
                  && Pos_7==tagA_7 && Pos_8==tagA_8
             )
          {
              case_var=1;
          }
          else { case_var=0;}
          GIE = 1;  //Turn on interrupts so we don't miss characters
          for (i=0;i<4;i++){
              for(j=0;j<4;j++) printf("%c",Buff[i][j]);
              printf(" ");
          }
          printf("\r\n");
          
  }    /* end of while loop  */
}		// end of main


void init_comms(void){
    CREN = 0;  //Clears any outstanding overflo error
	RX_PIN = 1;
	TX_PIN = 1;
	SPBRG = DIVIDER;
	RCSTA = (NINE_BITS|0x90);
	TXSTA = (SPEED|NINE_BITS|0x20);
        CREN=1;
    	TXEN=1;
    	SYNC=0;
    	SPEN=1;
    	BRGH=1;
}

void 
putch(unsigned char byte) 
{
	/* output one byte */
	while(!TXIF)	/* set when register is empty */
		continue;
	TXREG = byte;
}

unsigned char 
getch() {
	/* retrieve one byte */
	while(!RCIF)	/* set when register is not empty */
		continue;
    
	return RCREG;	
}

unsigned char
getche(void)
{
	unsigned char c;
	putch(c = getch());
	return c;
}
