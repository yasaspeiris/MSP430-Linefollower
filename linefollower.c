#include <msp430.h> 

void MotorSetup();
void SetLeftMotorSpeed();
void SetRightMotorSpeed();
void SetSpeeds(int lspeed,int rspeed);
void IRSensorSetup();
void SonarSetup();
void ReadSonar();
int readLine();
void lineFollow();
void SetBrakes();


/**
 * main.c
 */
unsigned int up_counter;
unsigned int distance_cm;


int val = 0;
int sensorpanelVal = 0;
int lastval = 0;



//PID Values
int error = 0;
int lasterror = 0;

#define BaseSpeed 160
#define Kp 18
#define Kd 30



int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    MotorSetup();
    IRSensorSetup();
    SonarSetup();

    while(1){
        lineFollow();
        if(sensorpanelVal ==15){
            SetBrakes();
            while(1){

            }
        }
    }

}


void SonarSetup(){

    /* set P1.2 (TRIG)to output direction */
    P1DIR |= BIT2;
    P1OUT &= ~BIT2;                 // keep trigger at low

    /* Set P1.1 to input direction (echo)
    P1.1 is an input for Timer A0 - Compare/Capture input */
    P1DIR &= ~BIT1;
    // Select P1.1 as timer trigger input select (echo from sensor)
    P1SEL = BIT1;

    /* Timer A0 configure to read echo signal:
    Timer A Capture/Compare Control 0 =>
    capture mode: 1 - both edges +
    capture sychronize +
    capture input select 0 => P1.1 (CCI1A) +
    capture mode +
    capture compare interrupt enable */
    CCTL0 |= CM_3 + SCS + CCIS_0 + CAP + CCIE;

    /* Timer A Control configuration =>
    Timer A clock source select: 1 - SMClock +
    Timer A mode control: 2 - Continous up +
    Timer A clock input divider 0 - No divider */
    TA0CTL |= TASSEL_2 + MC_2 + ID_0;

    // Global Interrupt Enable
    _BIS_SR(GIE);

}

#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
{
    if (CCTL0 & CCI)            // Raising edge
    {
        up_counter = CCR0;      // Copy counter to variable
    }
    else                        // Falling edge
    {
        // Formula: Distance in cm = (Time in uSec)/58
        distance_cm = (CCR0 - up_counter)/58;
    }
    TA0CTL &= ~TAIFG;           // Clear interrupt flag - handled
}

void ReadSonar(){
    P1OUT ^= BIT2;              // assert
    __delay_cycles(10);         // 10us wide
    P1OUT ^= BIT2;              // deassert
    __delay_cycles(60000);      // 60ms measurement cycle
}






void MotorSetup(){

    P2DIR |= BIT1+BIT5;//Pin 2.1 -> left motor speed Pin 2.5 -> Right Motor Speed
    P2SEL |= BIT1+BIT5;

    P1DIR |= BIT3+BIT4; // Set Pins 1.3 and 1.4 as outputs to control left motor direction
    P1DIR |= BIT5+BIT0; // Set Pins 1.5 and 1.6 as outputs to control right motor direction

    P1OUT &= ~(BIT1+BIT4+BIT5+BIT0); //set all pins to low



    /*** Timer1_A Set-Up ***/
    TA1CCR0 |= 200 - 1;
    TA1CCTL1 |= OUTMOD_7;
    TA1CCTL2 |= OUTMOD_7;
    TA1CCR1 |= 0;
    TA1CCR2 |= 0;
    TA1CTL |= TASSEL_2 + MC_1;
}

void SetLeftMotorSpeed(int speed){
    if (speed >0){
        P1OUT &= ~BIT3; // Pin 1.3 Low
        P1OUT |= BIT4; //Pin 1.4 High

        if(speed >199){
            speed = 199;//prevent CCR2 from being negative
        }
        TA1CCR1 = speed;
    }else if(speed <0){
        P1OUT |= BIT3; //1.3 High
        P1OUT &= ~BIT4; // Pin 1.4 Low
        speed = -speed;

        if(speed >199){
            speed = 199;//prevent CCR1 from being negative
        }

        TA1CCR1 = speed;
    }
}

void SetRightMotorSpeed(int speed){
    if (speed >0){
        P1OUT &= ~BIT5;//Pin 1.5 Low
        P1OUT |= BIT0; // Pin 1.3 Low

        if(speed >199){
            speed = 199;//prevent CCR2 from being negative
        }
        TA1CCR2 = speed;
    }else if(speed <0){
        P1OUT |= BIT0; //1.5 High
        P1OUT &= ~BIT6;//Pin 1.6 Low
        speed = -speed;

        if(speed >199){
            speed = 199; //prevent CCR2 from being negative
        }
        TA1CCR2 = speed;
    }
}

void SetSpeeds(int lspeed,int rspeed){
    SetLeftMotorSpeed(lspeed);SetRightMotorSpeed(rspeed);
}

void SetBrakes(){
    P1OUT &= ~BIT5;//Pin 1.5 Low
    P1OUT &= ~BIT0; // Pin 1.3 Low
    P1OUT &= ~BIT3; // Pin 1.3 Low
    P1OUT &= ~BIT4; //Pin 1.4 Low
    TA1CCR1 = 199;
    TA1CCR2 = 199;
}

void IRSensorSetup(){
    //Set IR sensor pins as inputs
    P2DIR &= ~(BIT0+BIT2+BIT3+BIT4); //Set pins 2.0,2.2,2.3,2.4 as inputs
    P1DIR &= ~(BIT6); //1.7 as inputs



}


int readLine()
{
    //from left to right. Sensor output is high when white space is detected. Since we're seeking a black line,inputs are inverted
    int sensor1 = !(P2IN&BIT0);
    int sensor2 = !(P2IN&BIT2);
    int sensor3 = !(P2IN&BIT3);
    int sensor4 =!(P1IN&BIT6);
    int sensor5 =!(P2IN&BIT4);



    int sum = 0;

    sensorpanelVal = (sensor1 * 1)+(sensor2* 2)+(sensor3 * 3)+(sensor4 *4)+(sensor5*5);
    sum = (sensor1+sensor2+sensor3+sensor4+sensor5);

    if (sum ==0){
        return lastval;
    }else{

    lastval = sensorpanelVal/sum;
    return lastval;
    }
}

void lineFollow(){
    val = readLine();
    error = 3-val;
    int delta = error-lasterror;
    int change = Kp*error + Kd*delta;
    lasterror = error;

    int leftMotorPWM = BaseSpeed -change;
    //constrain PWM
    if(leftMotorPWM >199){
        leftMotorPWM = 199;
    }else if(leftMotorPWM <0){
        leftMotorPWM = 0;
    }
    //constrain PWM
    int rightMotorPWM = BaseSpeed + change;
    if(rightMotorPWM >199){
            rightMotorPWM = 199;
    }else if(leftMotorPWM <0){
            rightMotorPWM = 0;
    }


    SetSpeeds(leftMotorPWM,rightMotorPWM);

}


