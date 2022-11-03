#define QRDEND ADC1BUF12 //pin15 or B12
#define QRDLEFT ADC1BUF4//pin6 or B2
#define QRDMIDDLE ADC1BUF13//pin7 or A2
//#define QRDRIGHT ADC1BUF14 //pin8 or A3 THIS one is the plan, but it is outputting pwm??
#define QRDRIGHT ADC1BUF11 //pin16 or b13
//#define QRDTASK ADC1BUF11 //pin 16 or B13
#define QRDTASK ADC1BUF0 // pin2 or A0
//#define QRDBALL ADC1BUF0 // pin2 or A0
#define QRDBALL ADC1BUF14 // pin8 or A3
#define RMSPEED OC1RS
#define LMSPEED OC2RS
#define RMFWDSPEED 313
#define LMFWDSPEED 313
#define TURNNINETY 900
#define SWITCHDIRCOUNT 1050
#define READJUST 50

#define LED _LATB1
#define LED2 _LATA4
#include "xc.h"

#pragma config ICS = PGx3
#pragma config FNOSC = LPFRC //500khz osc
#pragma config FWDTEN=OFF
#pragma config WINDIS=OFF
//#pragma config MCLRE = OFF //see useful tips to understand when to use this
#pragma config OSCIOFNC = OFF
#pragma config SOSCSRC = DIG

//Global variables
int steps = 0;
int isTimerUp = 0;

//Interrupt Functions -------------------------
void __attribute__((interrupt, no_auto_psv)) _OC1Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _OC2Interrupt(void);
void _ISR _T1Interrupt(void);

//Configure Functions --------------------------------
void config_ad(void);
void configPWM();
void configTimer();

//Action Functions--------------------------------------------------------------
void turnRight(int N){
    steps = 0;  //restart step count
    while(steps<N){
        LMSPEED = 0;
        _LATB9=1;
        _LATA1 = 0;
    }
}
void turnRight2(int N){
    steps = 0;  //restart step count
    while(steps<N){
        _LATB9=1;
        _LATA1 = 0;
    }
}
void turnLeft(int N){
    steps = 0;  //restart step count
    while(steps<N){
        _LATB1 = 1;
        LMSPEED = 0;
        _LATA1=1;
        _LATB9 = 0;
    }
}

void goBackwards(int N){
    steps = 0;
    while(steps<N){
        _LATB9 = 1;
        _LATA1 = 1;
        LMSPEED = LMFWDSPEED;
        RMSPEED = RMFWDSPEED;
    }
}

void forwardAdjust(int N){
        steps = 0;

    while (steps<N){
        LMSPEED = LMFWDSPEED;
        RMSPEED = RMFWDSPEED;
    }
}

void delay (int s) {
    int k = 0;
    while (k<s)
        k++;
}

void resetDefaultMotors(){
        LMSPEED = LMFWDSPEED;
        RMSPEED = RMFWDSPEED;
        _LATA1 = 0;
        _LATB9 = 0;
}

void theEnd(){
    //Change the direction of one of the motors
   // _LATB9=1;
//    //start count
    _OC1IE =1;
//    //Determining steps
//        //Determine the number of steps for turn
   
    N = 580;//half a square
    forwardAdjust(N);
    N = 1008;    //original number 126; value for 90 degrees and full square
    turnRight(N);
    goBackwards(N);
    LMSPEED = 0;
    RMSPEED = 0;
    while(1){}
   
}


int countLines(){
    //initialization
    RMSPEED = 375;
    LMSPEED = 375;
    
    int count = 1;
    int N = 600;//how far the bot goes to pass 3 lines, abt 1/3 a block
    steps = 0;//reset
    _OC1IE = 1;//start count
    int onBlack = 0;
//    LED = 0;
    //check 
    
    while(steps < N){
    if(QRDTASK > threshold ){//task sees black
        //bool = true
        onBlack = 1;
//        LED = 1;
        
    }
    if(onBlack == 1 && QRDTASK < threshold){
        //bool = false
        onBlack = 0;
        count ++;
//        LED = 0;
        //add to count
    }
    }
    
    return count;
    
}

//3 lines = sample return
    //2 line = sample collect
    //4 lines = canyon


// Main Function ------------------------------------------------------------------
int main(void) {
     //Configure the postscaler on the microchip
   _RCDIV = 0b000;
   
   //configure pins
   
//    ANSB = 00010001100;//b15-b0
//    ANSA = 001101; //A6-A0
//    TRISB = 01010011100; //B15-B0
//    TRISA = 001101;//A6-A0
   TRISA = 0;
   TRISB = 0;
   LATA = 0;
   LATB = 0;
   ANSA = 0;
   ANSB = 0;
   
  _ANSB2 = 1;
    _TRISB2 = 1;
    _ANSB13 = 1;
    _TRISB13 = 1;
     _ANSA2 = 1;
    _TRISA2 = 1;
    _ANSA3 = 1;
    _TRISA3 = 1;
    _ANSA0 = 1;
    _TRISA0 = 1;
    _ANSB12 = 1;
    _TRISB12 = 1;
    _TRISB7 = 1; // front proximity sensor
    _TRISB8 = 1; //right proximity sensor
    _TRISB15 = 1; // left proximity sensor
   
    _ANSB0 = 0;
    _TRISB0 = 0;
   
    _ANSB15 = 0;
    _TRISB15 = 0;
//    _ANSA4 = 0;
    _TRISA4 = 0;
   
    //Set motors to zero initially
    _LATA1 = 0;
    _LATB9 = 0;

 //initialize variables ---------------------------------------------------------
    int N = 0;//step counter
    int threshold = 1250; //QRD threshold
    int numLines = 0;
   

// Call Configurations -----------------------------------------------------------
    config_ad();
    configPWM();
    configTimer();
   
// States ----------------------------------------------------------------
    enum { LINE, CANYON} state;
    enum {FORWARD,TURNRIGHT} canyon_state;
    canyon_state = FORWARD;
    state = LINE;
    enum { FORWARD, LEFT90 , SECONDTIME , TURNAROUND, END, TASK} state;
    state = FORWARD;
   
// Set Initial Values ----------------------------------------------------------
//    _TON = 1;
    RMSPEED = RMFWDSPEED;
    LMSPEED = LMFWDSPEED; // ERROR, MAKE THIS 1500
    OC1R = LMFWDSPEED/2;
    OC2R = RMFWDSPEED/2;
   
//------------------------loop-------------------------------

    while(1){
        switch (state) {
            case LINE:
                _OC1IE = 0;
//                if(QRDEND > threshold){ //see black  
//                    theEnd(N);
//                }
//                else{
//                    _LATB9 = 0;
//                    _LATB1 = 0;
//                    _LATA1 = 0;
//                }
                if(QRDRIGHT > threshold && QRDLEFT < threshold){//see black
                    RMSPEED = 0;
                }
                else{
                    RMSPEED = RMFWDSPEED;
                }
                if(QRDLEFT > threshold && QRDRIGHT < threshold){//see black
                    LMSPEED = 0;
                }
                else{
                    LMSPEED = LMFWDSPEED;    
                }
                if (QRDLEFT < threshold && QRDRIGHT < threshold && QRDMIDDLE < threshold) {
                    canyon_state = FORWARD;
                    state = CANYON;
                }
                break;
            case CANYON:
                _OC1IE =1;
                switch(canyon_state) {
                    case FORWARD:
                      if (!_RB7){
                        goBackwards(READJUST);
                        resetDefaultMotors();
                        turnRight(TURNNINETY);
                        steps=0;
                        N = 1060;
                        resetDefaultMotors();
                        canyon_state = TURNRIGHT;
                        }
                      if (!_RB8) { // right side detects a wall
                        goBackwards(READJUST);
                        resetDefaultMotors();
                        turnLeft(READJUST);
                        resetDefaultMotors();
                        }
                      if (!_RB15) { // left side detects a wall
                        goBackwards(READJUST);
                        resetDefaultMotors();
                        turnRight(READJUST);
                        resetDefaultMotors();
                        }
                        break;
                    case TURNRIGHT:
                      if (!_RB7){
                          // full 180 turn and go back forward
                        turnRight2(SWITCHDIRCOUNT);
                        resetDefaultMotors();
                        canyon_state = FORWARD;
                        }
                      if (!_RB8) { // right side detects a wall
                        goBackwards(READJUST);
                        resetDefaultMotors();
                        turnLeft(READJUST);
                        resetDefaultMotors();
                        }
                      if (!_RB15) { // left side detects a wall
                        goBackwards(READJUST);
                        resetDefaultMotors();
                        turnRight(READJUST);
                        resetDefaultMotors();
                        }
                      if (steps > N) {
                          // return to forward
                        N = 0;
                        resetDefaultMotors();
                        canyon_state = FORWARD;
                      }
                      break;
        if(state == FORWARD){
            LED2 = 0;
            
            
            if(TMR1 > 800*numLines){
                LED = 0;
            }
             if(QRDEND > threshold){ //see black
                 TMR1 = 0;
                 state = END;
//                 theEnd();
        }
             else{
                 _LATB9 = 0;
                 _LATA1 = 0;
             }
        //start, check timer, if its too small, then DONT call function
            if(numLines == 3){
                RMSPEED = 0;
                LMSPEED = 0;
                while(1){
                
                }
                if (QRDLEFT > threshold || QRDRIGHT > threshold) {
                    forwardAdjust(145);
                    turnRight(TURNNINETY);
                    delay(2000);
                    state = LINE;
            } 
            if(QRDTASK > threshold){
                 //start timer
                 _TON = 1;
                 TMR1 = 0;
                 state = TASK;
                 LED2 = 1;
                 
             }else{
//                 LED = 0;
             }
            if(QRDRIGHT > threshold && QRDLEFT < threshold){//right see black
                RMSPEED = 0;    
            }
            else{
                RMSPEED = 375;
            }
            if(QRDLEFT > threshold && QRDRIGHT < threshold){//left see black
                LMSPEED = 0;
            }
            else{
                LMSPEED = 375;    
            }
        }
        
        if(state == TASK){
            if(QRDTASK < threshold){//task sees white
                if(TMR1 > 110){
                    LED = 1;
                    numLines = countLines();
                }
                break;
                state = FORWARD;
                TMR1 = 0;
//                _TON = 0;
            }
        }
        
        if(state == END){
            if(QRDEND < threshold){//task sees white
                if(TMR1 > 90){
                    theEnd();
                }
                state = FORWARD;
                TMR1 = 0;
//                _TON = 0;
            }
}
    }    
    return 0;
}

//Interrupt Functions -------------------------
void __attribute__((interrupt, no_auto_psv)) _OC1Interrupt(void){
    _OC1IF = 0; //take down flag
    steps=steps+1;
}
void __attribute__((interrupt, no_auto_psv)) _OC2Interrupt(void){
    _OC2IF = 0; //take down flag
    steps=steps+1;
}
void _ISR _T1Interrupt(void){
_T1IF = 0; // Clear interrupt flag
isTimerUp = 1;
// Do something here
}

//Configure Functions --------------------------------
void config_ad(void){
    //clear the registers
        AD1CON1 = 0;
    AD1CON2 = 0;
    AD1CON3 = 0;
    AD1CON5 = 0;
    AD1CSSL = 0;
    AD1CSSH = 0;
   
    _ADON = 0;    // AD1CON1<15> -- Turn off A/D during config
   
    // AD1CON1 register
    _ADSIDL = 0;  // AD1CON1<13> -- A/D continues in idle mode
    _MODE12 = 1;  // AD1CON1<10> -- 12-bit A/D operation
    _FORM = 0;    // AD1CON1<9:8> -- Unsigned integer output
    _SSRC = 7;    // AD1CON1<7:4> -- Auto conversion (internal counter)
    _ASAM = 1;    // AD1CON1<2> -- Auto sampling

    // AD1CON2 register
    _PVCFG = 0;   // AD1CON2<15:14> -- Use VDD as positive ref voltage
    _NVCFG = 0;   // AD1CON2<13> -- Use VSS as negative ref voltage
    _BUFREGEN = 1;// AD1CON2<11> -- Result appears in buffer location corresponding to channel
    _CSCNA = 1;   // AD1CON2<10> -- Scans inputs specified in AD1CSSx registers
    _SMPI = 4;  // AD1CON2<6:2> -- Every 4th conversion sent to buffer (if sampling 4 channels)
    _ALTS = 0;    // AD1CON2<0> -- Sample MUXA only

    // AD1CON3 register
    _ADRC = 0;    // AD1CON3<15> -- Use system clock
    _SAMC = 0;    // AD1CON3<12:8> -- Auto sample every A/D period TAD
    _ADCS = 0x3F; // AD1CON3<7:0> -- A/D period TAD = 64*TCY

    // AD1CSS registers
    // SET THE BITS CORRESPONDING TO CHANNELS THAT YOU WANT
    // TO SAMPLE
    _CSS4 = 1; // THE 4 corresponds to the AN# on the data sheet for the pin
    _CSS13 = 1;
    _CSS11 = 1;
    _CSS14 = 1;
    _CSS0 = 1;
    _CSS12 = 1;

    _ADON = 1;    // AD1CON1<15> -- Turn on A/D
}

void configPWM(){
    // Clear control bits
    OC1CON1 = 0;
    OC1CON2 = 0;
    OC2CON1 = 0;
    OC2CON2 = 0;
   
    // Set period
    RMSPEED = RMFWDSPEED;
    LMSPEED = LMFWDSPEED;
   
    //Number of counts for Duty Cycle. This is arbitrary for steppers
    OC1R = RMSPEED/2;
    OC2R = LMSPEED/2;
   
    // Configure OC1
    OC1CON1bits.OCTSEL = 0b111;
    OC1CON2bits.SYNCSEL = 0x1F;
    OC1CON2bits.OCTRIG = 0;    
    OC1CON1bits.OCM = 0b110;
    //Configure OC2
    OC2CON1bits.OCTSEL = 0b111;
    OC2CON2bits.SYNCSEL = 0x1F;
    OC2CON2bits.OCTRIG = 0;    
    OC2CON1bits.OCM = 0b110;
   
    _OC1IP = 4; // Select OCx interrupt priority
    _OC1IE = 0; // disable OCx interrupt to start
    _OC1IF = 0; // Clear OCx interrupt flag
   
    _OC2IP = 4; // Select OCx interrupt priority
    _OC2IE = 0; // disable OCx interrupt to start
    _OC2IF = 0; // Clear OCx interrupt flag
}

void configTimer(){
    T1CON = 0;
    _TCKPS = 0b11;  // 1:256
    _TCS = 0;       // Internal clock source 31khz
    TMR1 = 0;       // Reset Timer1
    _TON = 0;       // Turn Timer1 off
   
        // Configure Timer1 interrupt
    _T1IP = 4; // Select interrupt priority
    _T1IF = 0; // Clear interrupt flag
    _T1IE = 1; // Enable interrupt
    PR1 = 2929; // Timer period of 9688 or 5 sec
}