#define true 1
#define false 0

#define QRDMIDDLE ADC1BUF12 //pin15 or B12
#define QRDLEFT ADC1BUF4//pin6 or B2
#define QRDEND ADC1BUF13//pin7 or A2
#define QRDRIGHT ADC1BUF11 //pin16 or b13
#define QRDTASK ADC1BUF0 // pin2 or A0
#define QRDBALL ADC1BUF14 // pin8 or A3
#define SATDETECT ADC1BUF10 //pin 17 or B14
#define EQSERVICE ADC1BUF12 //pin 9 or B4 //SERVICING
#define RMSPEED OC1RS
#define LMSPEED OC2RS
#define SERVO OC3R
#define LASER _LATB12

int RMFWDSPEED = 1500;//4000000/2000;//clk speed divided by (steps per second*2) equals 120//have /800 for old 313
int LMFWDSPEED = 1500;//4000000/2000//this is for the old 120 //has been 175
#define TURNSPEED 4500//250
#define TURNNINETY 900//14400(Jared's val))
#define PIVOTNINETY 950//892//2400(jared's val))
#define SWITCHDIRCOUNT 1910//16800(Jared's val))
#define READJUST 100 //800(Jared's val))
#define SERVOPERIOD 80000//5000
#define QRDPSCALE 2.6//0.15
#define QRDISCALE .016//0.001
#define QRDERRORTHRESHOLD 400 //300
#define NEGERRORADJUST 35
#define TIMERSCALER 16 //16*256 or the difference between the oscillators times the postscalar
//BUT jared multipled by 11.66666 not 16
#define LED2 _LATB4
#define LED1 _LATA4
#include "xc.h"

#pragma config ICS = PGx3
#pragma config FNOSC = FRC //500khz osc
#pragma config FWDTEN=OFF
#pragma config WINDIS=OFF
//#pragma config MCLRE = OFF //see useful tips to understand when to use this
#pragma config OSCIOFNC = OFF
#pragma config SOSCSRC = DIG

//Global variables
int servoDC = 313;
int steps = 0;
int servosteps=0;
int isTimerUp = 0;
int qrdWhiteThreshold = 3000; // 12 bit adc so 4096 corresponds to 3.3 V. When it reads black it reads in about 1.5V
int qrdBlackThreshold = 2000; //QRD qrdBlackThreshold 1250?
int qrdLeftError = 0;
int qrdRightError = 0;
long int leftErrorTotal = 0;
long int rightErrorTotal = 0;
int lineTime = 300;//50;
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
void turnRight(int stepsThreshold){
    steps = 0;  //restart step count
    while(steps<stepsThreshold){
        LMSPEED = 0;
        _LATB9=1;
        _LATA1 = 0;
        RMSPEED = TURNSPEED;
    }
   
}
void rightPivot(int stepsThreshold){
 
   _LATB9=1;
   _LATA1 = 0;
    steps = 0;  //restart step count
    while(steps<stepsThreshold){
        LMSPEED = TURNSPEED;
        RMSPEED = TURNSPEED;
    }
      
}
void turnLeft(int stepsThreshold){

    steps = 0;  //restart step count
    while(steps<stepsThreshold){
    //    _LATB1 = 1;
        LMSPEED = 0;
        _LATA1=1;
        _LATB9 = 0;
       LMSPEED = TURNSPEED;
        RMSPEED = TURNSPEED;
    }

}

void leftPivot(int stepsThreshold){ 
    steps = 0;
    while (steps<stepsThreshold){
        LMSPEED = TURNSPEED;
        RMSPEED = TURNSPEED;
        _LATB9 = 0;
        _LATA1 = 1;
    }
}

void goBackwards(int stepsThreshold){
    steps = 0;
    while(steps<stepsThreshold){
        _LATB9 = 1;
        _LATA1 = 1;
        LMSPEED = TURNSPEED;
        RMSPEED = TURNSPEED;
    }  
}

void forwardAdjust(int stepsThreshold){
    steps = 0;
  _LATB9 = 0;
  _LATA1 = 0;
    while (steps<stepsThreshold){
        LMSPEED = TURNSPEED;
        RMSPEED = TURNSPEED;
    }
}

void delay (long int s) {
    int k = 0;
    while (k<s){
        k++;
    }
}

void hesitate(int length){
    LMSPEED = 0;
    RMSPEED = 0;
    TMR1 = 0;
    _TCKPS = 0b11;
    _TON = 1;
    while(TMR1 < length*TIMERSCALER){ }//lets waste time for a little over a second
    _TON = 0;

}
void resetDefaultMotors(){
        LMSPEED = TURNSPEED;
        RMSPEED = TURNSPEED;
        _LATA1 = 0;
        _LATB9 = 0;
}


void Satellite(){
    _OC3IE = 1;
    SERVO = 260;//I added the 16 because we switched the occilator(8Mhz))
    servosteps = 260;
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
    while(servosteps < 540){//what end val?
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
    hesitate(800);
    forwardAdjust(500);
    rightPivot(PIVOTNINETY);
    goBackwards(3000);
    LMSPEED = 0;
    RMSPEED = 0;
//    Satellite();
    while(1){}
  
}

void Sampledump()
{
//    int right = 1;
   //625 for max angle.
   //125 for min angle.
//read ball
SERVO=375*16;
 //   qrdBlackThreshold=1250
//if (QRDBALL >= 800)
//    {
//        right=0;
//    }    
//move ball to either side depending on QRD value
if (QRDBALL >= qrdBlackThreshold)
    {
 //   _LATA4=1;
     SERVO=205*16;  
    }
else
    {
//    _LATA4=0;
     SERVO=540*16;  
    }
}

int countLines(){
    //initialization
//    RMSPEED = 313;
//    LMSPEED = 313;
   
    int count = 1;
    int stepsThreshold = 800;//how far the bot goes to pass 3 lines, abt 1/3 a block
    steps = 0;//reset
    _OC1IE = 1;//start count
    int onBlack = 0;
   
    while(steps < stepsThreshold){
//        RMSPEED = 313;
//        LMSPEED = 313;
        if(QRDTASK > qrdBlackThreshold && onBlack == 0 ){//task sees black
            //bool = true
            onBlack = 1;  
            LED1 = 1;
        }
        if(onBlack == 1 && QRDTASK < qrdWhiteThreshold){//task sees white again
//        if(onBlack == 1 && QRDTASK < qrdWhiteThreshold){//task sees white again
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
//     _ANSB4 = 1;//theseneed to be un commented to use the ir sensor
//    _TRISB4 = 1;
   
    _ANSB0 = 0;
    _TRISB0 = 0;
    _ANSB4 = 0;
    _TRISB4 = 0; //THESE need to be uncommented to use the second led
    _ANSB12 = 0;
   _TRISB9 = 0;
//    _ANSB15 = 0;
//    _TRISB15 = 0;
//    _ANSA4 = 0;
    _TRISA4 = 0;
   
    //Set motors to zero initially
    _LATA1 = 0;
    _LATB9 = 0;

 //initialize variables ---------------------------------------------------------
    int stepsThreshold = 0;//step counter
    int numLines = 0;
    int doCollect = 0;
    int doDrop = 0;
    LASER = 1; //turn off laser
    int taskDetecting = false;
    int onBlack = 0;
    int inCanyon = 0;
   

// Call Configurations -----------------------------------------------------------
    config_ad();
    configPWM();
    configTimer();
   
// States ----------------------------------------------------------------
    enum {LINE, CANYON, END, TASK, CHECKLINE, COLLECTION, TESTSERVO} state;
    enum {FORWARD,TURNRIGHT} canyon_state;
    canyon_state = FORWARD;
    state = LINE;
//    state = TESTSERVO;

// Set Initial Values ----------------------------------------------------------
//    _TON = 1;
    RMSPEED = RMFWDSPEED;
    LMSPEED = LMFWDSPEED; // ERROR, MAKE THIS 1500
             //   RMFWDSPEED=0;
          //      LMFWDSPEED=0;
    OC1R = LMFWDSPEED/2;
    OC2R = RMFWDSPEED/2;
   LED1 =0;
   LED2 = 0;
   _OC1IE=1;
   _OC2IE=1;
//------------------------loop-------------------------------
    //start
//    hesitate(800);
    forwardAdjust(300);
//    leftPivot(PIVOTNINETY);
//    resetDefaultMotors();

//    theEnd();
    //running
    while(1){
        switch (state) {
           
            case TESTSERVO://used for testing purposes
                
//                qrdRightError = QRDRIGHT - (QRDERRORTHRESHOLD);
//                qrdLeftError = QRDLEFT - (QRDERRORTHRESHOLD);
//                leftErrorTotal += qrdLeftError;
//                rightErrorTotal += qrdRightError;
//                if (qrdLeftError < 0)  leftErrorTotal = 0;
//                if (qrdRightError < 0) rightErrorTotal = 0;
//               
//                RMSPEED = RMFWDSPEED + QRDRIGHT*QRDPSCALE + rightErrorTotal*QRDISCALE;
//                LMSPEED = LMFWDSPEED + QRDLEFT*QRDPSCALE + leftErrorTotal*QRDISCALE;
//                
//                if (QRDTASK >qrdBlackThreshold) {
////                _TCKPS = 0b11;
//                    _TON = 1;
//                    TMR1 = 0;
//                    while (TMR1 < lineTime){
//                        
//                    }
//                    hesitate(1000);
//                }
//                rightPivot(PIVOTNINETY);
//    OC1R = 0;
//    OC2R = 0;
//                delay(30000);
//                hesitate(8000);

                
//                if(EQSERVICE > 1500)//1500 was my val
//                {
////                    if(QRDRIGHT > qrdBlackThreshold && QRDLEFT < qrdBlackThreshold){
////                        while(QRDRIGHT > qrdBlackThreshold)
////                            {rightPivot(10);
////                            hesitate(100);
////                            }
////                    }
////                    else if(QRDLEFT > qrdBlackThreshold && QRDRIGHT<qrdBlackThreshold){
////                        while(QRDLEFT > qrdBlackThreshold)
////                            {leftPivot(10);
////                            hesitate(100);
////                            }
////                    }
////                    else{   
////                     hesitate(10000);   
////                    }
//               forwardAdjust(10000);
//                resetDefaultMotors();
//                    forwardAdjust(100);
//                    resetDefaultMotors();
//                    leftPivot(PIVOTNINETY);
//                    resetDefaultMotors();
//                    delay(10000);
//                    goBackwards(200);
//                    delay(10000);
//                   resetDefaultMotors();
//                    delay(10000);
//                    forwardAdjust(200);
//                    resetDefaultMotors();
                    rightPivot(PIVOTNINETY);
                    forwardAdjust(2000);
                    rightPivot(SWITCHDIRCOUNT);
                    forwardAdjust(2000);
//                    resetDefaultMotors();
//                    delay(65000);
//                    delay(65000);
//                    delay(65000);
//                    delay(65000);
//                    delay(65000);
//                    delay(65000);
//                    delay(65000);
//                    delay(65000);
//                    delay(65000);
         
//                if(!_RB7)
//                {
//                SERVO = 200*16;
//                }
   //            OC1R = 0;
//                     OC2R = 0;
//                    hesitate(8000);
//                    hesitate(8000);
//                    resetDefaultMotors();
//                }
//                else
//                {
//                SERVO=450*16;
//                }
//                if(QRDRIGHT > qrdBlackThreshold && QRDLEFT < qrdBlackThreshold){//right see black
//                    RMSPEED = 0;
//                    LMSPEED = LMFWDSPEED;
//                }
////                else{
////                    RMSPEED = RMFWDSPEED;
////                }
//                if(QRDLEFT > qrdBlackThreshold && QRDRIGHT < qrdBlackThreshold){//left see black
//                    LMSPEED = 0;
//                    RMSPEED = RMFWDSPEED;
//                }
////                else{
////                    LMSPEED = LMFWDSPEED;
////                }
//                if (QRDLEFT < qrdBlackThreshold && QRDRIGHT < qrdBlackThreshold) {
//                    LMSPEED = LMFWDSPEED;
//                    RMSPEED = RMFWDSPEED;
//                }

            break;    
           
            case LINE:
                               if(EQSERVICE > 1500)//1500 was my val
                {
//                    if(QRDRIGHT > qrdBlackThreshold && QRDLEFT < qrdBlackThreshold){
//                        while(QRDRIGHT > qrdBlackThreshold)
//                            {rightPivot(10);
//                            hesitate(100);
//                            }
//                    }
//                    else if(QRDLEFT > qrdBlackThreshold && QRDRIGHT<qrdBlackThreshold){
//                        while(QRDLEFT > qrdBlackThreshold)
//                            {leftPivot(10);
//                            hesitate(100);
//                            }
//                    }
//                    else{   
//                     hesitate(10000);   
//                    }
//                    forwardAdjust(100);
//                    resetDefaultMotors();
//                    leftPivot(PIVOTNINETY);
//                    resetDefaultMotors();
//                    goBackwards(200);
//                    resetDefaultMotors();
//                    forwardAdjust(200);
//                    resetDefaultMotors();
//                    rightPivot(PIVOTNINETY);
//                    resetDefaultMotors();
//                    SERVO = 200*16;
//                    hesitate(8000);
                     OC1R = 0;
                     OC2R = 0;
                     hesitate(800);
//                    resetDefaultMotors();
                }
                  // THE END ---------------------------------------------------  
//                if(QRDEND > qrdBlackThreshold){ //END sees black ERROR: THIS QRD SHOULD RESPOND TO THRESHOLD BUT IT ISNT
//                     TMR1 = 0;
//                    _TCKPS = 0b11;
//                     _TON = 1;
////                     LED2 = 1;
//                     state = END;
//                }
//                else{
//                     _LATB9 = 0;
//                     _LATA1 = 0;
//                 }
                
                if(isTimerUp == 1 && doDrop ==1){
                    T2CONbits.TON = 0;
                    TMR2 = 0;
                    T2CONbits.TCKPS = 0b11;
                    doDrop = 0;
                    isTimerUp = 0;
                    hesitate(800);
                    Sampledump();
                    hesitate(800);
                    SERVO = 375*16;// SEVERO STUFF WE'LL need to figure out
                }
                if(isTimerUp == 1 && doCollect ==1){
                    T2CONbits.TON = 0;
                    TMR2 = 0;
                    T2CONbits.TCKPS = 0b11;
                    doCollect = 0;
                    isTimerUp = 0;
                    _OC1IE = 1; //eRROR should this be in all of the functions?
                   leftPivot(PIVOTNINETY);
                   goBackwards(1800);
                   hesitate(800);
                   resetDefaultMotors();
                   forwardAdjust(1800);
                   rightPivot(PIVOTNINETY);
                   resetDefaultMotors();
                   _OC1IE = 0;//error
                }
               
               //Equipment servicing ----------------------------------------
//                if(EQSERVICE > 400 ){//ir at qrdBlackThreshold
//                    LED1 = 1;
//                    doCollect = 1;
//                    isTimerUp = 1;
//                }
               
                //Count lines ----------------------------------------------
                if(taskDetecting == false){
                        if(QRDTASK > qrdBlackThreshold){//task sees black
                                //start timer
                                TMR1 = 0;
                                _TCKPS = 0b11;
                                _TON = 1;
                                state = TASK;
                            }
                }else{
                        if(QRDTASK > qrdBlackThreshold && onBlack == false){//task sees black
                            onBlack = true;  
                        }
                        if(onBlack == true && QRDTASK < qrdBlackThreshold){//task sees white again
                            onBlack = false;
                            numLines ++;        //add to count
                        }

                        if(steps > stepsThreshold){
                            switch (numLines){
                                case 2://collect ball
                                   doCollect = 1;
                                   TMR2 = 0;
                                   T2CONbits.TCKPS = 0b11;
                                   T2CONbits.TON = 1;
                                    break;
                                case 3://drop ball
                                    doDrop = 1;
                                   TMR2 = 0;
                                   T2CONbits.TCKPS = 0b11;
                                   T2CONbits.TON = 1;
                                break;
                                case 4://canyon
                                    state = CANYON;
                                break;
                            }
                            numLines = 0;
                            taskDetecting = false;
                            _OC1IE = 0;//stop count
                            onBlack = false;
                            LMFWDSPEED = 1500;
                            RMFWDSPEED = 1500;//error
                            hesitate(100);
                            forwardAdjust(300);
                        }
                }
//                -----PI Controller for line following
                qrdRightError = QRDRIGHT - (QRDERRORTHRESHOLD);
                qrdLeftError = QRDLEFT - (QRDERRORTHRESHOLD);
                leftErrorTotal += qrdLeftError;
                rightErrorTotal += qrdRightError;
                if (qrdLeftError < 0)  leftErrorTotal = 0;
                if (qrdRightError < 0) rightErrorTotal = 0;
               
                RMSPEED = RMFWDSPEED + QRDRIGHT*QRDPSCALE + rightErrorTotal*QRDISCALE;
                LMSPEED = LMFWDSPEED + QRDLEFT*QRDPSCALE + leftErrorTotal*QRDISCALE;
                
                
                
//                //Old stuff that I replaced with the code in test servo
//                qrdRightError = QRDRIGHT - (QRDERRORTHRESHOLD);
//                qrdLeftError = QRDLEFT - (QRDERRORTHRESHOLD);
//                if (qrdLeftError < 0) qrdLeftError = qrdLeftError*NEGERRORADJUST;
//                if (qrdRightError < 0) qrdRightError = qrdRightError*NEGERRORADJUST;
//                leftErrorTotal += qrdLeftError;
//                rightErrorTotal += qrdRightError;
//                if (leftErrorTotal < 0) leftErrorTotal = 0;
//                if (rightErrorTotal < 0) rightErrorTotal = 0;
//               
//                RMSPEED = RMFWDSPEED + QRDRIGHT*QRDPSCALE + rightErrorTotal*QRDISCALE;
//                LMSPEED = LMFWDSPEED + QRDLEFT*QRDPSCALE + leftErrorTotal*QRDISCALE;
               
               
//                if (LMSPEED < (LMFWDSPEED + 50) && QRDLEFT > qrdBlackThreshold) leftErrorTotal += 100000;
//                if (RMSPEED < (RMFWDSPEED + 50) && QRDRIGHT > qrdBlackThreshold) rightErrorTotal += 100000;
               
                //Line following -------------------------------------------------
//                if(QRDRIGHT > qrdBlackThreshold && QRDLEFT < qrdBlackThreshold){//right see black
//                    RMSPEED = 0;
//                    LMSPEED = LMFWDSPEED;
//                }
////                else{
////                    RMSPEED = RMFWDSPEED;
////                }
//                if(QRDLEFT > qrdBlackThreshold && QRDRIGHT < qrdBlackThreshold){//left see black
//                    LMSPEED = 0;
//                    RMSPEED = RMFWDSPEED;
//                }
////                else{
////                    LMSPEED = LMFWDSPEED;
////                }
//                if (QRDLEFT < qrdBlackThreshold && QRDRIGHT < qrdBlackThreshold) {
//                    LMSPEED = LMFWDSPEED;
//                    RMSPEED = RMFWDSPEED;
//                }
                break;
                           
               
            // this makes sure that task is not incorrectly triggered
            case TASK:
                if(QRDTASK < qrdBlackThreshold){//task sees white
                    _TON = 0;
                    if(TMR1 > lineTime){
                        LMFWDSPEED = 313*16;
                        RMFWDSPEED = 313*16;//error
                        stepsThreshold = 1000;//how far the bot goes to pass 3 lines, abt 1/3 a block
                        steps = 0;//reset
                        _OC1IE = 1;//start count
                        taskDetecting = true;
                        numLines = 1;
//                        numLines = countLines(); //error delete
                    }
                    state = LINE;
                    TMR1 = 0;
                    _TCKPS = 0b11;
                }
                break;
               
            // this makes sure that end is not incorrectly triggered
            case END:
                if(QRDEND < qrdBlackThreshold){//task sees white
                    _TON = 0;
                    if(TMR1 > lineTime){
//                                          LED1 = 1;

                        theEnd();
                    }
                    state = LINE;
                    TMR1 = 0;
                    _TCKPS = 0b11;
                   
                   
//                     LED2 = 0;
    //                _TON = 0;
                }
                break;
               
            case COLLECTION:
                if (QRDLEFT > qrdBlackThreshold || QRDRIGHT > qrdBlackThreshold) { //EITHER sees black
                        state = CHECKLINE;
                        TMR1 = 0;
                        _TCKPS = 0b11;
                        _TON = 1;
                    }
                break;
               
            case CANYON:
                _OC1IE =1;
                LMSPEED = 200*16;
                RMSPEED = 200*16;
                switch(canyon_state) {
                    case FORWARD:
                      if (!_RB7){
                        rightPivot(PIVOTNINETY);
//                        goBackwards(READJUST);
//                        resetDefaultMotors();
//                        turnRight(TURNNINETY);
                        steps=0;
                        stepsThreshold = 1060;
                        resetDefaultMotors();
                        canyon_state = TURNRIGHT;
                        
                        }
//                      if (!_RB8) { // right side detects a wall
//                        goBackwards(READJUST);
//                        resetDefaultMotors();
//                        turnLeft(READJUST);
//                        resetDefaultMotors();
//                        }
//                      if (!_RB15) { // left side detects a wall
//                        goBackwards(READJUST);
//                        resetDefaultMotors();
//                        turnRight(READJUST);
//                        resetDefaultMotors();
//                        }
                        break;
                    case TURNRIGHT:
                      if (!_RB7){
                          goBackwards(READJUST);
                          // full 180 turn and go back forward
                        rightPivot(SWITCHDIRCOUNT);
                        resetDefaultMotors();
                        canyon_state = FORWARD;
                        inCanyon = 1;
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
                      if (steps > stepsThreshold) {
                          // return to forward
                        stepsThreshold = 0;
                        resetDefaultMotors();
                        canyon_state = FORWARD;
                      }
                      break;  
               
//                if (!_RB8) { // right side detects a wall
//                    goBackwards(READJUST);
//                    resetDefaultMotors();
//                    turnLeft(READJUST);
//                    resetDefaultMotors();
//                }
//                if (!_RB15) { // left side detects a wall
//                    goBackwards(READJUST);
//                    resetDefaultMotors();
//                    turnRight(READJUST);
//                    resetDefaultMotors();
//                }
//                      
                }
                if (QRDLEFT > qrdBlackThreshold|| QRDRIGHT > qrdBlackThreshold) {
//                    if(inCanyon == 1){
                        state = CHECKLINE;
                        TMR1 = 0;
                        _TCKPS = 0b11;
                        _TON = 1;
//                    }
                }
//                break;  
            case CHECKLINE:
                if(QRDLEFT < qrdBlackThreshold && QRDRIGHT < qrdBlackThreshold){//both see white
                    _TON = 0;
                    if(TMR1 > lineTime && inCanyon == 1 ){
                        forwardAdjust(500);
                        leftPivot(PIVOTNINETY);
                        RMSPEED = RMFWDSPEED;
                        LMSPEED = LMFWDSPEED;
//                        delay(2000);
                        state = LINE;
                    }else{
                        state = CANYON;
                    }
                    TMR1 = 0;
                    _TCKPS = 0b11;
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
        servosteps ++;
        SERVO = servosteps;
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
             
                   
    OC3R = 375*16;
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
    _TCS = 0;       // Internal clock source 8MHz
    TMR1 = 0;       // Reset Timer1
    _TCKPS = 0b11;  // 1:256
    _TON = 0;       // Turn Timer1 off
   
        // Configure Timer1 interrupt
    _T1IP = 4; // Select interrupt priority
    _T1IF = 0; // Clear interrupt flag
    _T1IE = 0; // Enable interrupt
//    PR1 = 2929; // Timer period of 9688 or 5 sec
   
   
    //TIMER 2 -------------------------------
    T2CON = 0;
    T2CONbits.TCS = 0;       // Internal clock source 31khz
    TMR2 = 0;       // Reset Timer1
    T2CONbits.TCKPS = 0b11;  // 1:256

    T2CONbits.TON = 0;       // Turn Timer1 off
   
        // Configure Timer1 interrupt
    _T2IP = 4; // Select interrupt priority
    _T2IF = 0; // Clear interrupt flag
    _T2IE = 1; // Enable interrupt
    PR2 = 450*TIMERSCALER; // Timer period of 9688 or 5 sec
}
