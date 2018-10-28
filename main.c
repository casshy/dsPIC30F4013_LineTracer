/* 
 * File:   main.c
 */

#define FOSC (7370000*16) //FRC * PLL16 (7.37 * 16 MHz)
#define FCY (FOSC / 4) //dsPIC performs one instruction every four clock cycles
#define TCY (1/FCY)

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include <p30f4013.h>
#include <libpic30.h>
#include <outcompare.h>
#include "pwm.h"
#include "dsp.h"
#include "uart.h"
#include "timer.h"

// FOSC
#pragma config FOSFPR = FRC_PLL16       // Oscillator (FRC w/PLL 16x)
#pragma config FCKSMEN = CSW_FSCM_OFF   // Clock Switching and Monitor (Sw Disabled, Mon Disabled)

// FWDT
#pragma config FWPSB = WDTPSB_16        // WDT Prescaler B (1:16)
#pragma config FWPSA = WDTPSA_512       // WDT Prescaler A (1:512)
#pragma config WDT = WDT_OFF            // Watchdog Timer (Disabled)

// FBORPOR
#pragma config FPWRT = PWRT_64          // POR Timer Value (64ms)
#pragma config BODENV = BORV20          // Brown Out Voltage (Reserved)
#pragma config BOREN = PBOR_OFF         // PBOR Enable (Disabled)
#pragma config MCLRE = MCLR_EN          // Master Clear Enable (Enabled)

// FGS
#pragma config GWRP = GWRP_OFF          // General Code Segment Write Protect (Disabled)
#pragma config GCP = CODE_PROT_OFF      // General Segment Code Protection (Disabled)

// FICD
#pragma config ICS = ICS_PGD            // Comm Channel Select (Use PGC/EMUC and PGD/EMUD)

#define M1PIN1 LATBbits.LATB0
#define M1PIN2 LATBbits.LATB1
#define M2PIN1 LATBbits.LATB2
#define M2PIN2 LATBbits.LATB3
#define M3PIN1 LATBbits.LATB4
#define M3PIN2 LATBbits.LATB5
#define M4PIN1 LATBbits.LATB6
#define M4PIN2 LATBbits.LATB7
#define M1PWM OC1RS
#define M2PWM OC2RS
#define M3PWM OC3RS
#define M4PWM OC4RS
#define PHOTOR1 PORTBbits.RB9
#define PHOTOR2 PORTBbits.RB10
#define PHOTOR3 PORTBbits.RB11
#define PHOTOR4 PORTBbits.RB12
#define PHOTOR5 PORTDbits.RD9

unsigned char buffer[64];
unsigned char *pbuf;
unsigned int count;

void initUART();
void initTimer1();
void initTimer2();
void initPWM();
void SetPWM1Duty(int ratio);
void SetPWM2Duty(int ratio);
void SetPWM3Duty(int ratio);
void SetPWM4Duty(int ratio);
void Motor1(int pwm);
void Motor2(int pwm);
void Motor3(int pwm);
void Motor4(int pwm);
void Move(int Vx, int Vy, int Vtheta);

void __attribute__((__interrupt__, __shadow__)) _T1Interrupt(void){
    IFS0bits.T1IF = 0;
//    printf("Timer1 interrupt\r\n");
}

void __attribute__((__interrupt__)) _U1TXInterrupt(void)
{
    IFS0bits.U1TXIF = 0;
}
void __attribute__((__interrupt__)) _U1RXInterrupt(void)
{
    IFS0bits.U1RXIF = 0;
    while(DataRdyUART1()){
        char recv;
        recv = ReadUART1();
//        printf("recv: %c %x\r\n", recv, recv);
    }
}
void __attribute__((__interrupt__)) _U2RXInterrupt(void)
{
    IFS1bits.U2RXIF = 0;
    while(DataRdyUART2()){
        unsigned int recv;
        recv = ReadUART2();
        *pbuf = (unsigned char)(recv & 0xff);
        pbuf++;
        count++;
        if(count > 32){
            pbuf = buffer;
            count = 0;
        }
        //0x80を受信するたびにポインタをバッファの先頭に戻す
        if(recv == 0x80){
            printf("\r\n");
            pbuf = buffer;
            count = 0;
            int i, sum = 0;
            for(i = 0; i < 6; i++){
                sum += buffer[i];
            }
            //チェックサムが一致しない場合は処理をしない
            if((unsigned char)(sum&0x7f) == buffer[6]){
                int vx, vy, vtheta;
                vx = (int)buffer[2];
                vy = (int)buffer[3];
                vtheta = (int)buffer[4];
                vx = (4 * vx - 255);
                vy = 4 * vy - 255;
                vtheta = 4 * vtheta - 255;
                Move(vx, vy, vtheta);
                LATDbits.LATD9 ^= 1;
//                printf("data received: vx=%d, vy=%d, vtheta=%d\r\n", vx, vy, vtheta);
            }
            
        }
        printf("%02x ", recv);
    }
}

int main(){
    OSCCON = 0x0000;
    OSCCONbits.COSC = 0b001;
    OSCCONbits.NOSC = 0b001;
    OSCCONbits.POST = 0b00;
    TRISB = 0b1111100000000; //RB9 to RB12 are input
    TRISD = 0x00;
    TRISF = 0x00; //PortF is output.
    ADPCFG = 0xFFFF;
    
    LATB = 0x00;
    LATF = 0x00;
    
    pbuf = buffer;
    
    //Timer settings
    initTimer1();
    initTimer2();
    //PWM settings
    initPWM();
    //UART settings
    initUART();
    Move(0, 0, 0);
    __delay_ms(500);
    
    int i;
    for(i = 0; i < 3; i++){
        LATDbits.LATD9 = 1;
        __delay_ms(200);
        LATDbits.LATD9 = 0;
        __delay_ms(200);
    }
    Move(0, 0, 0);
    
//    printf("main loop.\r\n");
//    while(BusyUART1());
    while(1){
//        LATDbits.LATD9 ^= 1;
//        __delay_ms(500);
    }
}

void initUART(){
    unsigned int baud1, baud2;
    unsigned int UMODEvalue, U1STAvalue, U2STAvalue;
    UMODEvalue = UART_EN & UART_IDLE_CON &
                 UART_RX_TX & UART_DIS_WAKE &
                 UART_DIS_LOOPBACK & UART_DIS_ABAUD &
                 UART_NO_PAR_8BIT & UART_1STOPBIT;
    U1STAvalue = UART_INT_TX_BUF_EMPTY &
                 UART_TX_PIN_NORMAL &
                 UART_TX_ENABLE &
                 UART_INT_RX_CHAR &
                 UART_ADR_DETECT_DIS &
                 UART_RX_OVERRUN_CLEAR;
    U2STAvalue = UART_INT_TX_BUF_EMPTY &
                 UART_TX_PIN_LOW &
                 UART_TX_DISABLE &
                 UART_INT_RX_CHAR &
                 UART_ADR_DETECT_DIS &
                 UART_RX_OVERRUN_CLEAR;
    baud1 = 191; //baud rate 9600
    baud2 = 767; //baud rate 2400
    OpenUART1(UMODEvalue, U1STAvalue, baud1);
    OpenUART2(UMODEvalue, U2STAvalue, baud2);
    ConfigIntUART1(UART_RX_INT_EN & UART_RX_INT_PR5 & UART_TX_INT_DIS);
    ConfigIntUART2(UART_RX_INT_EN & UART_RX_INT_PR6 & UART_TX_INT_DIS);
}

void initTimer1(){
    OpenTimer1(T1_ON & T1_GATE_OFF & T1_PS_1_256 & T1_SYNC_EXT_OFF & T1_SOURCE_INT,10000);
    ConfigIntTimer1(T1_INT_PRIOR_5 & T1_INT_ON);
}

//Timer2 settings for Motor Control PWM
void initTimer2(void) {
    //Set PWM period to 0.8ms
    PR2 = 1473;
    T2CON = 0;	// Reset timer2 control bits - Prescale 1:1 (because T2CON<5:4> = 00)
    T2CONbits.TCS = 0; // the timer clock source set to internal (TCS = 0)
    T2CONbits.TON = 1;	// enable timer 2

    return;
}

void initPWM(){
    OC1CONbits.OCM = 0b110; // 110: PWM mode
    OC2CONbits.OCM = 0b110;
    OC3CONbits.OCM = 0b110;
    OC4CONbits.OCM = 0b110;
    OC1RS = 1178;	// Set output compare value (PWM duty cycle)
    OC2RS = 1178;	// Set output compare value (PWM duty cycle)
    OC3RS = 1178;	// Set output compare value (PWM duty cycle)
    OC4RS = 1178;	// Set output compare value (PWM duty cycle)
}


void SetPWM1Duty(int ratio){
    OC1RS = (int)(PR2 * ((float)ratio / 255));
}
void SetPWM2Duty(int ratio){
    OC2RS = (int)(PR2 * ((float)ratio / 255));
}
void SetPWM3Duty(int ratio){
    OC3RS = (int)(PR2 * ((float)ratio / 255));
}
void SetPWM4Duty(int ratio){
    OC4RS = (int)(PR2 * ((float)ratio / 255));
}


void Motor1(int value){
    if(value < 0){
        M1PIN1 = 0;
        M1PIN2 = 1;
        value = -value;
    }else{
        M1PIN1 = 1;
        M1PIN2 = 0;
    }
    if(value < 50){
        M1PIN1 = 1;
        M1PIN2 = 1;
    }
    SetPWM1Duty(value);
}
void Motor2(int value){
    if(value < 0){
        M2PIN1 = 0;
        M2PIN2 = 1;
        value = -value;
    }else{
        M2PIN1 = 1;
        M2PIN2 = 0;
    }
    if(value < 50){
        M2PIN1 = 1;
        M2PIN2 = 1;
    }
    SetPWM2Duty(value);
}
void Motor3(int value){
    if(value < 0){
        M3PIN1 = 0;
        M3PIN2 = 1;
        value = -value;
    }else{
        M3PIN1 = 1;
        M3PIN2 = 0;
    }
    if(value < 50){
        M3PIN1 = 1;
        M3PIN2 = 1;
    }
    SetPWM3Duty(value);
}
void Motor4(int value){
    if(value < 0){
        M4PIN1 = 0;
        M4PIN2 = 1;
        value = -value;
    }else{
        M4PIN1 = 1;
        M4PIN2 = 0;
    }
    if(value < 50){
        M4PIN1 = 1;
        M4PIN2 = 1;
    }
    SetPWM4Duty(value);
}
int L = 1;
void Move(int Vx, int Vy, int Vtheta){
    int V1, V2, V3, V4;
    V1 = Vx + L * Vtheta;
    V2 = Vy + L * Vtheta;
    V3 = -Vx + L * Vtheta;
    V4 = -Vy + L * Vtheta;
    Motor1(V1);
    Motor2(V2);
    Motor3(V3);
    Motor4(V4);
}