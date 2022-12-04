#define true 1
#define false 0

#define QRDLEFT ADC1BUF4//pin6 or B2
#define QRDEND ADC1BUF13//pin7 or A2
#define QRDRIGHT ADC1BUF11 //pin16 or b13
#define QRDTASK ADC1BUF0 // pin2 or A0
#define QRDBALL ADC1BUF14 // pin8 or A3
#define SATDETECT ADC1BUF10 //pin 17 or B14
//#define EQSERVICE ADC1BUF12 //pin 9 or B4 //SERVICING
#define EQSERVICE ADC1BUF15 //try two!
#define RMSPEED OC1RS
#define LMSPEED OC2RS
#define SERVO OC3R
#define LASER _LATA4
#define FRONTDISTANCE _RB7
#define LEFTDISTANCE _RB15
#define RIGHTDISTANCE _RB8

int RMFWDSPEED = 1500;//4000000/2000;//clk speed divided by (steps per second*2) equals 120//have /800 for old 313
int LMFWDSPEED = 1500;//4000000/2000//this is for the old 120 //has been 175
#define HIGHSPEED 1500
#define SLOWSPEED 2000
#define TURNSPEED 4500//250
#define PIVOTNINETY 500//892//2400(jared's val))
#define SWITCHDIRCOUNT 1700//16800(Jared's val))
#define READJUST 170 //800(Jared's val))
#define QRDPSCALE 2.6//0.15]
#define QRDISCALE .016//0.001
#define QRDERRORTHRESHOLD 400 //300
#define TIMERSCALER 16 //16*256 or the difference between the oscillators times the postscalar
//BUT jared multipled by 11.66666 not 16
const int SERVOPERIOD = 80000;//5000
#include "xc.h"

#pragma config ICS = PGx3
#pragma config FNOSC = FRC //500khz osc
#pragma config FWDTEN=OFF
#pragma config WINDIS=OFF
//#pragma config MCLRE = OFF //see useful tips to understand when to use this
#pragma config OSCIOFNC = OFF
#pragma config SOSCSRC = DIG

//Global variables
int steps = 0;
int servosteps=0;
int qrdWhiteThreshold = 3000; // 12 bit adc so 4096 corresponds to 3.3 V. When it reads black it reads in about 1.5V
int qrdBlackThreshold = 2000;
int qrdBallThreshold = 620; //QRD ball threshold
int qrdEndThreshold = 1000;
int qrdLeftError = 0;
int qrdRightError = 0;
long int leftErrorTotal = 0;
long int rightErrorTotal = 0;
int lineTime = 300;//50;
int Rcounter1= 0;
int Rcounter2= 0;
int Lcounter1= 0;
int Lcounter2=0;
//Interrupt Functions -------------------------
void __attribute__((interrupt, no_auto_psv)) _OC1Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _OC3Interrupt(void);

//Configure Functions --------------------------------
void config_ad(void);
void configPWM();
void configTimer();

//Action Functions--------------------------------------------------------------
void rightPivot(int stepsThreshold){
   _LATB9 = 1;
   _LATA1 = 0;
    steps = 0;  //restart step count
    while(steps<stepsThreshold){
        LMSPEED = TURNSPEED;
        RMSPEED = TURNSPEED;
    }
      
}

void leftPivot(int stepsThreshold){ 
    _LATB9 = 0;
    _LATA1 = 1;
    steps = 0;
    while (steps<stepsThreshold){
        LMSPEED = TURNSPEED;
        RMSPEED = TURNSPEED;
    }
}

void goBackwards(int stepsThreshold){
    steps = 0;
    _LATB9 = 1;
    _LATA1 = 1;
    while(steps<stepsThreshold){
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
    SERVO = 260*16;//I added the 16 because we switched the occilator(8Mhz))
    servosteps = 260*16;
    int keepServo = 0;
    int maxIR = 0;
    
    //increment servo and read in the IR values
    while(servosteps < 540*16){
        if(SATDETECT > maxIR){
            keepServo = SERVO;
            maxIR = SATDETECT;
        }
        //READin qrd and choose highest val
    }
    _OC3IE = 0;
   
    while(1){
        SERVO=260*16;
//        if(keepServo > 500*16 || keepServo < 300*16){
//            //SERVO = 390*16;
//           
//        }else
//       // SERVO = keepServo;
//           
//        }
//        delay(10);
        hesitate(500);
        LASER = 0;//turn on laser
    }
}

void theBeginning() {
    hesitate(100);
    forwardAdjust(1900);
    leftPivot(PIVOTNINETY);
    forwardAdjust(25);
    resetDefaultMotors();
}
void theEndlinefollow(){//NOTE: this must come after the satellite function
    hesitate(100);
    forwardAdjust(100);
    
     leftPivot(100); 
    while(QRDLEFT < 1500){
      leftPivot(2);
      hesitate(1);
      }
     resetDefaultMotors();
     hesitate(100);
      while(QRDRIGHT < 1500){//I added this second while loop so it is able to slowly move to this position and aline itself on the line.
      leftPivot(2);
     hesitate(1);
      }
     resetDefaultMotors();
     
     while(1){
            while(QRDLEFT > qrdBlackThreshold || QRDRIGHT > qrdBlackThreshold){
                            qrdRightError = QRDRIGHT - (QRDERRORTHRESHOLD);
                       qrdLeftError = QRDLEFT - (QRDERRORTHRESHOLD);
                       leftErrorTotal += qrdLeftError;
                       rightErrorTotal += qrdRightError;
                       if (qrdLeftError < 0)  leftErrorTotal = 0;
                       if (qrdRightError < 0) rightErrorTotal = 0;

                       RMSPEED = 2500 + QRDRIGHT*QRDPSCALE + rightErrorTotal*QRDISCALE;
                       LMSPEED = 2500 + QRDLEFT*QRDPSCALE + leftErrorTotal*QRDISCALE;   
            }
            
            while(QRDLEFT<qrdBlackThreshold && QRDRIGHT<qrdBlackThreshold && FRONTDISTANCE==0){
                RMSPEED = 2500;
                LMSPEED = 2500;
               _LATA1 = 0;
               _LATB9 = 0;       
            } 
            
            while(QRDLEFT<qrdBlackThreshold && QRDRIGHT<qrdBlackThreshold && FRONTDISTANCE==1){
                leftPivot(SWITCHDIRCOUNT);
                goBackwards(500);
                LMSPEED = 0;
                RMSPEED = 0;
                Satellite();
                while(1){}
            }              
       //                        forwardAdjust(1200);
//                        hesitate(1000);
//                            LMSPEED = 0;
//                            RMSPEED = 0;
//    rightPivot(675);
//    goBackwards(1800);
//    LMSPEED = 0;
//    RMSPEED = 0;
//    Satellite();
//    while(1){}
     }
}
void theEnd(){//NOTE: this must come after the satellite function
    hesitate(100);
    forwardAdjust(100);
    

     resetDefaultMotors();

    rightPivot(PIVOTNINETY);
    goBackwards(2000);
    LMSPEED = 0;
    RMSPEED = 0;
    Satellite();
    while(1){}
     
}

void Sampledump(){
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
if (QRDBALL < qrdBallThreshold){//if it sees white
     SERVO=540*16;  //white
    }else{
     SERVO=205*16;  //black
    }
}

// Main Function ------------------------------------------------------------------
int main(void) {
     //Configure the postscaler on the microchip
   _RCDIV = 0b000;
   
   //configure pins
   
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
    _TRISB7 = 1; // front proximity sensor
    _TRISB8 = 1; //right proximity sensor
    _TRISB15 = 1; // left proximity sensor
   _ANSB14 = 1;//LASER DETECTOR
    _TRISB14 = 1;   
    _TRISB4 = 1;
     _ANSB4 = 1;//theseneed to be un commented to use the ir sensor

 //initialize variables ---------------------------------------------------------
    int stepsThreshold = 0;//step counter
    int numLines = 0;
    LASER = 1; //turn off laser
    int taskDetecting = false;
    int onBlack = false;
    int inCanyon = false;
    int theStart = false;
    int EQdone = 0;

// Call Configurations -----------------------------------------------------------
    config_ad();
    configPWM();
    configTimer();
   
// States ----------------------------------------------------------------
    enum {LINE, CANYON, END, TASK, CHECKLINE, TESTSERVO, CHECKWALL} state;
    enum {FORWARD,TURNRIGHT} canyon_state;
    canyon_state = FORWARD;
    state = LINE;

// Set Initial Values ----------------------------------------------------------
    RMFWDSPEED = SLOWSPEED;
    LMFWDSPEED = SLOWSPEED;
    RMSPEED = RMFWDSPEED;
    LMSPEED = LMFWDSPEED; 
    OC1R = LMFWDSPEED/2;
    OC2R = RMFWDSPEED/2;
    _LATA1 = 0;
    _LATB9 = 0;
   _OC1IE=1; //error take this out
   LASER=1;//Our mosfet is backwards right now. 1 is off, 0 is on.
   //------------------------loop-------------------------------
    //start
//   
    hesitate(100);
    forwardAdjust(1900);
    leftPivot(PIVOTNINETY);
    forwardAdjust(25);
    resetDefaultMotors();
//                    
    while(1){
        switch (state) {
           
            case TESTSERVO://used for testing purposes
                    RMSPEED = 0;
                     LMSPEED = 0;
                     LASER = 1;
                SERVO=260*16;
//                hesitate(10);
////                Satellite();
//                LASER = 1;
               
//                if(QRDEND > qrdEndThreshold){ //END sees black ERROR: THIS QRD SHOULD RESPOND TO THRESHOLD BUT IT ISNT
//                    hesitate(200);
//                }else{resetDefaultMotors();}
            break;    
           
            case LINE:
//                -----PI Controller for line following
                qrdRightError = QRDRIGHT - (QRDERRORTHRESHOLD);
                qrdLeftError = QRDLEFT - (QRDERRORTHRESHOLD);
                leftErrorTotal += qrdLeftError;
                rightErrorTotal += qrdRightError;
                if (qrdLeftError < 0)  leftErrorTotal = 0;
                if (qrdRightError < 0) rightErrorTotal = 0;
               
                RMSPEED = RMFWDSPEED + QRDRIGHT*QRDPSCALE + rightErrorTotal*QRDISCALE;
                LMSPEED = LMFWDSPEED + QRDLEFT*QRDPSCALE + leftErrorTotal*QRDISCALE;
                              
                //Count lines ----------------------------------------------

                if(taskDetecting == false){
                        if(QRDTASK > qrdBlackThreshold){//task sees black
                                //start timer
                                TMR1 = 0;
                                _TCKPS = 0b11;
                                _TON = 1;
                                state = TASK;
                            }
                }
                else{
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
                                    leftPivot(PIVOTNINETY-50);
                                    goBackwards(900);
                                    hesitate(800);
                                    forwardAdjust(800);
                                    rightPivot(PIVOTNINETY);
                                    resetDefaultMotors();    
                                break;
                                case 3://drop ball
                                    forwardAdjust(50);
                                    hesitate(10);
                                    Sampledump();
                                    hesitate(800);
                                    SERVO = 375*16;// SEVERO STUFF WE'LL need to figure out
                                    hesitate(800);
                                    forwardAdjust(200);
                                break;
                                case 4://canyon
//                                    resetDefaultMotors();
                                    state = CANYON;
                                break;
                            }
                            numLines = 0;
                            taskDetecting = false;
                            onBlack = false;
                            LMFWDSPEED = 1500;
                            RMFWDSPEED = 1500; //error
                        }
                }
                
                
               //Equipment servicing ----------------------------------------
            //    if(EQSERVICE >  1500 && EQdone==0)
                if(EQSERVICE>1500){
                    EQdone = 1;
                    forwardAdjust(200);
                    hesitate(100);
                    goBackwards(100);
                    leftPivot(PIVOTNINETY-20);
                    goBackwards(860);
                    hesitate(600);
                    forwardAdjust(890);
                    rightPivot(PIVOTNINETY/2);
                        while(QRDRIGHT < 1500){
                        rightPivot(2);
                        hesitate(1);
                        }
                    hesitate(100);
                    forwardAdjust(200);
                }
//                
                
               //  THE END ---------------------------------------------------  
                if(QRDEND > qrdEndThreshold){ //END sees black ERROR: THIS QRD SHOULD RESPOND TO THRESHOLD BUT IT ISNT
                     TMR1 = 0;
                    _TCKPS = 0b11;
                     _TON = 1;
                     state = END;
                }
//                
            break;
                           
               
            // this makes sure that task is not incorrectly triggered
            case TASK:
//                if (QRDEND > qrdBlackThreshold){
//                    theStart = true;
//                }
                if(QRDTASK < qrdBlackThreshold){//task sees white
                    _TON = 0;
                    if(TMR1 > lineTime){
//                       if (theStart == true){
//                        theBeginning();
//                         theStart = false;
//                        state = LINE;
//                    }
                        LMFWDSPEED = 313*16;
                        RMFWDSPEED = 313*16;//error
                        stepsThreshold = 750;//how far the bot goes to pass 4 lines, abt 1/3 a block
                        steps = 0;//reset
                        taskDetecting = true;
                        numLines = 1;
                    }
                    state = LINE;
                    TMR1 = 0;
                    _TCKPS = 0b11;
                }
            break;
               
            // this makes sure that end is not incorrectly triggered
            case END:
//                start timer with linetime*2 as threshold
                // after timer expires check if QRDTASK has ever been high if so run beginning())
//                if (QRDTASK > qrdBlackThreshold){
//                    isStart = true;
//                }
                if(QRDEND < qrdEndThreshold){//task sees white
                    _TON = 0;
                    if(TMR1 > lineTime){
                        steps = 0;
//                        if (inCanyon == true){
                            theEnd();
//                        }
//                        else {
//                            theBeginning();
//                        }
                    }
                    TMR1 = 0;
                    _TCKPS = 0b11;
                    state = LINE;
                }
            break;
               
            case CANYON:
                LMSPEED = 200*16;
                RMSPEED = 200*16;
                switch(canyon_state) {
                    case FORWARD:
                      if (!FRONTDISTANCE){
                        hesitate(100);
                        goBackwards(200);
                        rightPivot(350);//ERROR shouldn't this be pivot 90?
                        steps=0;
                        stepsThreshold = 1060;
                        resetDefaultMotors();
                        Rcounter1=0;
                        Lcounter1=0;
                        canyon_state = TURNRIGHT;
                        }
                    if (!RIGHTDISTANCE && Rcounter1<5) { // right side detects a wall
                  //  goBackwards(READJUST);
                    hesitate(50);
                    goBackwards(READJUST);
                    resetDefaultMotors();
                    leftPivot(READJUST/2);
                    hesitate(10);
                    resetDefaultMotors();
                      Rcounter1++;
                }
                if (!LEFTDISTANCE && Lcounter1<5) { // left side detects a wall
                  //  goBackwards(READJUST);
                 //   hesitate(10);
                 //   resetDefaultMotors();
                    Lcounter1++;
                    hesitate(50);
                    goBackwards(READJUST);
                    rightPivot(READJUST/2);
                    hesitate(10);
                    resetDefaultMotors();
                }
                    break;
                    case TURNRIGHT:
                      if (!FRONTDISTANCE){
                          goBackwards(200);
                          // full 180 turn and go back forward
                        rightPivot(SWITCHDIRCOUNT/2);
                        resetDefaultMotors();
                        canyon_state = FORWARD;
                        inCanyon = true;
                        }
                    if (!RIGHTDISTANCE && Rcounter2 < 5) { // right side detects a wall
                  //  goBackwards(READJUST);
                   // hesitate(10);
                  //  resetDefaultMotors();
                        hesitate(50);
                        goBackwards(READJUST);
                        Rcounter2++;
                    leftPivot(READJUST/2);
                    hesitate(10);
                    resetDefaultMotors();
                }
                if (!LEFTDISTANCE && Lcounter2 < 5
                        ) { // left side detects a wall
                   // goBackwards(READJUST);
                    hesitate(50);
                    goBackwards(READJUST);
                    Lcounter2++;
                   // resetDefaultMotors();
                    rightPivot(READJUST/2);
                    hesitate(10);
                    resetDefaultMotors();
                }
                      if (steps > stepsThreshold) {
                          // return to forward
                        stepsThreshold = 0;//error delete
                        resetDefaultMotors();
                        canyon_state = FORWARD;
                        Rcounter2=0;
                        Lcounter2=0;
                      }
                      break;  
               
//                      
                }
                if (QRDLEFT > qrdBlackThreshold || QRDRIGHT > qrdBlackThreshold) {
//                    if(inCanyon == 1){
                        TMR1 = 0;
                        _TCKPS = 0b11;
                        _TON = 1;
                        state = CHECKLINE;
//                    }
                }
            break;  
                
            case CHECKLINE:
                if(QRDLEFT < qrdBlackThreshold && QRDRIGHT < qrdBlackThreshold){//both see white
                    _TON = 0;
                    if(TMR1 > lineTime && inCanyon == true ){
                        forwardAdjust(100);
                        leftPivot(200);//used to be 350, Changed after adding the while loop
                                while(QRDLEFT < 1500){
                              leftPivot(2);
                              hesitate(1);
                                  }
                        resetDefaultMotors();
                        hesitate(100);
                               while(QRDRIGHT < 1500){//I added this second while loop so it is able to slowly move to this position and aline itself on the line.
                              leftPivot(2);
                              hesitate(1);
                                  }
                        forwardAdjust(100);
                        
                        //JAREDS CODE That uses a timer---------------------------
//                        TMR1 = 0;
//                        _TCKPS = 0b11;
//                        _TON = 1;
                        //------------------------------------------
                        
//                        RMSPEED = RMFWDSPEED;
//                        LMSPEED = LMFWDSPEED;
//                        delay(2000);
                        stepsThreshold=700;
                        steps = 0;
                        state = CHECKWALL;
                    } 
                    else {
                        state = CANYON;
                    }
                    TMR1 = 0;
                    _TCKPS = 0b11;
                }
            break;
            case CHECKWALL:
                forwardAdjust(10);
                
                if(!FRONTDISTANCE){
                        goBackwards(300);
                        rightPivot(SWITCHDIRCOUNT/4);
                              while(QRDRIGHT < 1500){
                              rightPivot(2);
                              hesitate(1);
                                  }
                        forwardAdjust(50);
                        state=LINE;
                        EQdone=0;
                }
                if(steps>stepsThreshold){
                    state=LINE;
                    EQdone=0;
                }
                
                //JAREDS CODE THAT USES A TIMER------------------------------
//                if (TMR1 > 1000) {
//                    if (!FRONTDISTANCE){
//                        goBackwards(300);
//                        rightPivot(SWITCHDIRCOUNT/4);
//                              while(QRDRIGHT < 1500){
//                              rightPivot(2);
//                              hesitate(1);
//                                  }
//                        forwardAdjust(50);
//                        TMR1 = 0;
//                        _TON = 0;
//                    }
//                    state = LINE;
//                    EQdone = 0;
//                }
                //--------------------------------------------
              
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

void __attribute__((interrupt, no_auto_psv)) _OC3Interrupt(void){
    _OC3IF = 0; //take down flag
//    delayCount++;
//    if(delayCount > 2){
        servosteps ++;
        SERVO = servosteps;
//        delayCount = 0;
//    }
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
}
