#define QRDMIDDLE ADC1BUF12 //pin15 or B12
#define QRDLEFT ADC1BUF4//pin6 or B2
#define QRDEND ADC1BUF13//pin7 or A2
//#define QRDRIGHT ADC1BUF14 //pin8 or A3 THIS one is the plan, but it is outputting pwm??
#define QRDRIGHT ADC1BUF11 //pin16 or b13
//#define QRDTASK ADC1BUF11 //pin 16 or B13
#define QRDTASK ADC1BUF0 // pin2 or A0
//#define QRDBALL ADC1BUF0 // pin2 or A0
#define QRDBALL ADC1BUF14 // pin8 or A3
#define SATDETECT ADC1BUF10 //pin 17 or B14
#define EQSERVICE ADC1BUF15 //pin 9 or B4 //SERVICING
#define RMSPEED OC1RS
#define LMSPEED OC2RS
#define RMFWDSPEED 175
#define LMFWDSPEED 175
#define TURNNINETY 900
#define SWITCHDIRCOUNT 1050
#define READJUST 50
#define PIVOTNINETY 450
#define SERVOPERIOD 5000
#define SERVO OC3R
#define LASER _LATB12

#define LED2 _LATB4
#define LED1 _LATA4
#include "xc.h"

#pragma config ICS = PGx3
#pragma config FNOSC = LPFRC //500khz osc
#pragma config FWDTEN=OFF
#pragma config WINDIS=OFF
//#pragma config MCLRE = OFF //see useful tips to understand when to use this
#pragma config OSCIOFNC = OFF
#pragma config SOSCSRC = DIG

//Global variables
int servoDC = 313;
int steps = 0;
int isTimerUp = 0;
int threshold = 800; //QRD threshold
int lineTime = 50;
int delayCount = 0;

//Interrupt Functions -------------------------
void __attribute__((interrupt, no_auto_psv)) _OC1Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _OC2Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _OC3Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void);

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

void turnLeft2(int N){
    steps = 0;
    while (steps<N){
        _LATB9 = 0;
        _LATA1 = 1;
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

void delay (long int s) {
    int k = 0;
    while (k<s){
        k++;
    }
}

void hesitate(int length){
    RMSPEED = 0;
    LMSPEED = 0;
    TMR1 = 0;
    _TON = 1;
    while(TMR1 < length){ }//lets waste time for a little over a second
    _TON = 0;

}
void resetDefaultMotors(){
        LMSPEED = LMFWDSPEED;
        RMSPEED = RMFWDSPEED;
        _LATA1 = 0;
        _LATB9 = 0;
}


void Satellite(){
    _OC3IE = 1;
    SERVO = 260;
    steps = 260;
    LED1 = 0;
//    delay(30);
    int keepServo = 0;
    int maxIR = 0;
    //to test success
//    while(1){
//        if(SATDETECT > 200){
//            LED1 = 1;
//        }else LED1 = 0;
//    }
    //increment servo and read in the IR values
    while(steps < 540){//what end val?
//        LED1 = 1;
        if(SATDETECT > maxIR){
            keepServo = SERVO;
            maxIR = SATDETECT;
        }
        //READin qrd and choose highest val
    }
    LED1 = 0;
    _OC3IE = 0;
    
    while(1){
        if(keepServo > 500 || keepServo < 300){
            SERVO = 390;
        }else{
        SERVO = keepServo;
        }
//        SERVO = keepServo;
        LASER = 0;//turn on laser
    }
    //if highest reading is high enough go to keepServo
    //else go to average value
}

void theEnd(){//NOTE: this must come after the satellite function
    //Change the direction of one of the motors
   // _LATB9=1;
//    //start count
    _OC1IE =1;
//    //Determining steps
//        //Determine the number of steps for turn
   
    forwardAdjust(400);
    turnRight2(PIVOTNINETY);
    goBackwards(2100);
    LMSPEED = 0;
    RMSPEED = 0;
    Satellite();
    while(1){}
   
}

void Sampledump()
{
//    int right = 1;
   //625 for max angle.
   //125 for min angle.
//read ball
SERVO=375;
 //   threshold=1250
//if (QRDBALL >= 800)
//    {
//        right=0;
//    }    
//move ball to either side depending on QRD value
if (QRDBALL >= threshold)
    {
 //   _LATA4=1;
     SERVO=205;  
    }
else
    {
//    _LATA4=0;
     SERVO=540;  
    }
}

int countLines(){
    //initialization
    RMSPEED = RMFWDSPEED;
    LMSPEED = LMFWDSPEED;
   
    int count = 1;
    int N = 600;//how far the bot goes to pass 3 lines, abt 1/3 a block
    steps = 0;//reset
    _OC1IE = 1;//start count
    int onBlack = 0;
   
    while(steps < N){
        if(QRDTASK > threshold ){//task sees black
            //bool = true
            onBlack = 1;  
            LED1 = 1;
        }
        if(onBlack == 1 && QRDTASK < threshold){//task sees white again
            //bool = false
            onBlack = 0;
            LED1 = 0;
            count ++;        //add to count
        }
    }
//    LED2 = 0;
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
    _ANSA3 = 1;//servo qrd
    _TRISA3 = 1;
    _ANSA0 = 1;
    _TRISA0 = 1;
    _ANSB12 = 1;
    _TRISB12 = 1;
    _TRISB7 = 1; // front proximity sensor
    _TRISB8 = 1; //right proximity sensor
    _TRISB15 = 1; // left proximity sensor
   _ANSB14 = 1;//LASER DETECTOR
    _TRISB14 = 1;
     _ANSB4 = 1;
    _TRISB4 = 1;
   
    _ANSB0 = 0;
    _TRISB0 = 0;
//    _ANSB4 = 0;
//    _TRISB4 = 0; //THESE need to be uncommented to use the second led
    _ANSB12 = 0;
    _TRISB12 = 0;
   
//    _ANSB15 = 0;
//    _TRISB15 = 0;
//    _ANSA4 = 0;
    _TRISA4 = 0;
   
    //Set motors to zero initially
    _LATA1 = 0;
    _LATB9 = 0;

 //initialize variables ---------------------------------------------------------
    int N = 0;//step counter
    int threshold = 1250; //QRD threshold
    int numLines = 0;
    int doCollect = 0;
    int doDrop = 0;
    LASER = 1; //turn off laser
   

// Call Configurations -----------------------------------------------------------
    config_ad();
    configPWM();
    configTimer();
   
// States ----------------------------------------------------------------
    enum {LINE, CANYON, END, TASK, CHECKLINE, COLLECTION, TESTSERVO} state;
    enum {FORWARD,TURNRIGHT} canyon_state;
    canyon_state = FORWARD;
    state = LINE;

// Set Initial Values ----------------------------------------------------------
//    _TON = 1;
    RMSPEED = RMFWDSPEED;
    LMSPEED = LMFWDSPEED; // ERROR, MAKE THIS 1500
    OC1R = LMFWDSPEED/2;
    OC2R = RMFWDSPEED/2;
   LED1 =0;
                LED2 = 0;
//------------------------loop-------------------------------
    //start
//    hesitate(800);
//    forwardAdjust(2000);
//    turnLeft2(PIVOTNINETY);
//    resetDefaultMotors();

//    theEnd();
    //running
    while(1){
        switch (state) {
           
            case TESTSERVO:
                
                RMSPEED = 0;
                LMSPEED = 0;
//                if(EQSERVICE > 400 ){//ir at threshold
//                    LED1 = 1;
//                    doCollect = 1;
//                    isTimerUp = 1;
//                }
//                SERVO = 1;
//                delay(10000);
//                SERVO = 200;
//                delay(10000);
//                SERVO = 500;
//                delay(10000);
                Satellite();

            break;    
           
            case LINE:
                _OC1IE = 0;
//                LED1 = 0;//signifies that we are back in the line function
                               
//                if(QRDEND > threshold){ //END sees black ERROR: THIS QRD SHOULD RESPOND TO THRESHOLD BUT IT ISNT
//                     TMR1 = 0;
//                     _TON = 1;
//                     LED2 = 1;
//                     state = END;
//                }
//                else{
//                     _LATB9 = 0;
//                     _LATA1 = 0;
//                 }
                if(isTimerUp == 1 && doDrop ==1){
//                    LED1 = 0;
                    T2CONbits.TON = 0;
                    TMR2 = 0;
                    doDrop = 0;
                    isTimerUp = 0;
//                    sampledump();
                    hesitate(800);
                    Sampledump();
                    hesitate(2000);
                    SERVO = 375;
                }
                if(isTimerUp == 1 && doCollect ==1){
//                    LED1 = 0;
                    T2CONbits.TON = 0;
                    TMR2 = 0;
                    doCollect = 0;
                    isTimerUp = 0;
                         _OC1IE = 1; //eRROR should this be in all of the functions?
//                        forwardAdjust(300);
                        turnLeft2(PIVOTNINETY);
                        goBackwards(820);
                        hesitate(800);
                        resetDefaultMotors();
                        forwardAdjust(700);
                        turnRight2(PIVOTNINETY);
                        resetDefaultMotors();
                        _OC1IE = 0;
                }
                switch (numLines){
                    case 2://collect ball
                       numLines = 0;
//                       LED1 = 1;
                       doCollect = 1;
                       TMR2 = 0;
                       T2CONbits.TON = 1;
                        break;
                    case 3://drop ball
                        numLines = 0;
                        doDrop = 1;
//                       LED1 = 1;
                       TMR2 = 0;
                       T2CONbits.TON = 1;
                    break;
                    case 4://canyon
                        state = CANYON;
                        numLines = 0;
                    break;
                }
               
                if(QRDTASK > threshold){//task sees black
                    //start timer
                    TMR1 = 0;
                    _TON = 1;
                    state = TASK;
//                    LED1 = 1;
                }
                
               //Equipment servicing 
//                if(EQSERVICE > 400 ){//ir at threshold
//                    LED1 = 1;
//                    doCollect = 1;
//                    isTimerUp = 1;
//                }
                
                //Line following -------------------------------------------------
                if(QRDRIGHT > threshold && QRDLEFT < threshold){//right see black
                    RMSPEED = 0; 
//                    LED1 = 1;
//                    LMSPEED = 413;  
                }
                else{
                    RMSPEED = RMFWDSPEED;
//                    LED1 = 0;
                }
                if(QRDLEFT > threshold && QRDRIGHT < threshold){//left see black
                    LMSPEED = 0;
                    LED2 = 1;
//                    RMSPEED = 413;
                }
                else{
                    LED2 = 0;
                    LMSPEED = LMFWDSPEED;
                }
               
                break;
                           
               
            // this makes sure that task is not incorrectly triggered
            case TASK:
                if(QRDTASK < threshold){//task sees white
                    _TON = 0;
                    if(TMR1 > lineTime){
//                        LED1 = 0;
//                        LED2 = 1;
                        numLines = countLines();
                    }
                    state = LINE;
                    TMR1 = 0;
    //                _TON = 0;
                }
                break;
               
            // this makes sure that end is not incorrectly triggered
            case END:
                if(QRDEND < threshold){//task sees white
                    _TON = 0;
                    if(TMR1 > 110){
//                                          LED1 = 1;

                        theEnd();
                    }
                    state = LINE;
                    TMR1 = 0;
                    
//                     LED2 = 0;
    //                _TON = 0;
                }
                break;
               
            case COLLECTION:
                if (QRDLEFT > threshold || QRDRIGHT > threshold) { //EITHER sees black
                        state = CHECKLINE;
                        TMR1 = 0;
                        _TON = 1;
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
                }
                if (QRDLEFT > threshold || QRDRIGHT > threshold) {
                    state = CHECKLINE;
                    TMR1 = 0;
                    _TON = 1;
                }
                break;  
            case CHECKLINE:
                if(QRDLEFT < threshold && QRDRIGHT < threshold){//both see white
                    _TON = 0;
                    if(TMR1 > 50){
                        forwardAdjust(145);
                        turnRight2(PIVOTNINETY);
                        resetDefaultMotors();
//                        delay(2000);
                        state = LINE;
                    }else{
                        state = CANYON;
                    }
                    TMR1 = 0;
                }
                break;
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
void __attribute__((interrupt, no_auto_psv)) _OC3Interrupt(void){
    _OC3IF = 0; //take down flag
//    delayCount++;
//    if(delayCount > 2){
        steps ++;
        SERVO = steps;
//        delayCount = 0;
//    }
}
void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void){
_T2IF = 0; // Clear interrupt flag
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
    _SMPI = 7;  // AD1CON2<6:2> -- Every 4th conversion sent to buffer (if sampling 4 channels)
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
    _CSS10 = 1;
    _CSS15 = 1;

    _ADON = 1;    // AD1CON1<15> -- Turn on A/D
}



void configPWM(){
    //Configure Values for Stepper motors
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
    _OC1IE = 1; // ENABLE OCx interrupt to start ERROR CHECK
    _OC1IF = 0; // Clear OCx interrupt flag
   
    _OC2IP = 4; // Select OCx interrupt priority
    _OC2IE = 0; // disable OCx interrupt to start
    _OC2IF = 0; // Clear OCx interrupt flag
   
   
    //CONFIGURE PWM FOR SERVOS
    OC3CON1 = 0;
    OC3CON2 = 0;
   
    _TRISB1= 0;//ERROR CHECK
             
                   
    OC3R = 375;
    //625 for max angle.
   //125 for min angle.       
    OC3RS= SERVOPERIOD;
   
    //Configure OC 3
    OC3CON1bits.OCTSEL = 0b111;
    OC3CON2bits.SYNCSEL = 0x1F;
    OC3CON2bits.OCTRIG = 0;    
    OC3CON1bits.OCM = 0b110;  
    
    _OC3IP = 4; // Select OCx interrupt priority
    _OC3IE = 0; // disable OCx interrupt to start
    _OC3IF = 0; // Clear OCx interrupt flag
   
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
    _T1IE = 0; // Enable interrupt
//    PR1 = 2929; // Timer period of 9688 or 5 sec
    
    
    //TIMER 2 -------------------------------
    T2CON = 0;
    T2CONbits.TCKPS = 0b11;  // 1:256
    T2CONbits.TCS = 0;       // Internal clock source 31khz
    TMR2 = 0;       // Reset Timer1
    T2CONbits.TON = 0;       // Turn Timer1 off
   
        // Configure Timer1 interrupt
    _T2IP = 4; // Select interrupt priority
    _T2IF = 0; // Clear interrupt flag
    _T2IE = 1; // Enable interrupt
    PR2 = 450; // Timer period of 9688 or 5 sec
}
