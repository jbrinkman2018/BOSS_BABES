#include <xc.h>

#pragma config FNOSC = LPFRC     // 500kHz occiltor
 // I hope this works
// Global variables
int steps = 0;


void __attribute__((interrupt, no_auto_psv)) _OC1Interrupt(void)
{
       _OC1IF = 0; 
    steps=steps+1;
}


int main()
{

//----------------configure the Stepper/PWM-------------------
   //Configure the postscaler on the microchip
   _RCDIV = 000;
    // Set period and duty cycle
    OC1RS = 2000;
    OC2RS = 2000;
    //Number of counts for Duty Cycle
    OC1R = 750;
    OC2R = 750;
    // Clear control bits
    OC1CON1 = 0;
    OC1CON2 = 0;
    OC2CON1 = 0;
    OC2CON2 = 0;
    
    // Configure OC1
    OC1CON1bits.OCTSEL = 0b111;
    OC1CON2bits.SYNCSEL = 0x1F; 
    OC1CON2bits.OCTRIG = 0;     
    OC1CON1bits.OCM = 0b110; 
    //Configure OC2
    OC1CON1bits.OCTSEL = 0b111;
    OC2CON2bits.SYNCSEL = 0x1F; 
    OC2CON2bits.OCTRIG = 0;     
    OC2CON1bits.OCM = 0b110;

//-------------------Configure Timer--------------------------

// Configure Timer1
    _TON = 1;       // Turn Timer1 on
    _TCKPS = 0b11;  // 1:256
    _TCS = 0;       // Internal clock source (FOSC/2)
    TMR1 = 0;       // Reset Timer1
    _TON = 0;       // Turn Timer1 off


    int N = 0;              // Desired steps
 //-------------------Configure Pins-------------------------   
 //THESE ARE MY TWO PINS FOR THE DIRECTION
     _TRISA1 = 0;
     _LATA1 = 0;

    _TRISB9 = 0;
    _LATB9 = 0;
    
//------------------------loop-------------------------------
    // States
    enum { FORWARD, LEFT90 , SECONDTIME , TURNAROUND } state;
    
    _TON = 1;
    while(1)
    {
    if (state == FORWARD)
        {
    OC1RS = 2000;
    OC2RS = 2000;
    OC1R = 750;
    OC2R = 750;
            if(TMR1>2929)
            {
                //turn off the timer
                _TON =0;
                //Change the direction of one of the motors
                _LATB9=1;
                //start counting steps
                _OC1IE =1;
                //Determine the number of steps for turn
                N = 700;
                steps = 0;
                state = LEFT90;
            }
        }
    if (state == LEFT90)
        {
            if(steps>N)
            {
            //Reset timer
            TMR1=0;
            //turn timer on
            _TON=1;
            //same direction for both motors
            _LATB9=0;
            //Stop counting steps
            _OC1IE=0;
            N=0;
            state = SECONDTIME;
            
            
            }
        }  
    if (state == SECONDTIME)
    {
            if (TMR1>2929)
            {
            _TON=0;
            
            _LATB9 = 1;
            
            N=1400;
            
            steps=0;
            
            _OC1IE=1;
            state == TURNAROUND;
            }
            
    }
      if (state == TURNAROUND)
{
        if(steps>N)
        {
        _LATB9= 1;
        TMR1 =0;
        _TON=1;
state = FORWARD;
        }

}      
       
        }
         
    return 0;
    
    }
