////////////////Library for i2c LCD//////////////

#include <Wire.h>
#include <LiquidCrystal_I2C.h>    

LiquidCrystal_I2C lcd(0x27,16,2); //address  0x27.

uint8_t arrow[8] = {0x0, 0x4 ,0x6, 0x3f, 0x6, 0x4, 0x0};
uint8_t ohm[8] = {0xE ,0x11, 0x11, 0x11, 0xA, 0xA, 0x1B};
uint8_t up[8] = {0x0 ,0x0, 0x4, 0xE , 0x1F, 0x4, 0x1C, 0x0};

////////////Library for ADS1115 ADC///////////////

#include <Adafruit_ADS1015.h>
Adafruit_ADS1115 ads(0x48);

/////////////Library for MCP4725 DAC////////////
#include <MCP4725.h>
  MCP4725 MCP(0x62);  // address 0x62 or 0x63

//////////////Temp and fan control///////////////////
LM35 sensA(A0);  
LM35 sensB(A1);

int temp1;
int temp2;
int temp; 
const byte OC1A_PIN = 5;   // //fan control pwm    

int tempMin = 0;                       //temperature at which to start the fan
int tempMax = 60;                     //maximum temperature when fan speed at 100%

const word PWM_FREQ_HZ = 16000; //Adjust this value to adjust the frequency
const word TCNT1_TOP = 16000000/(2*PWM_FREQ_HZ);

///////////////INPUTS/OUTPUTS////////////

const byte SW =A3 ;
const byte SW_red = A1;   

int SW_blue = 2;   // blue push button for menu
int Buzzer = 1;     //Buzzer connected on pin D3
/////////////////////////////////////////////





int Delay = 300;                    //This is the LCD refresh rate. Each 300ms.
unsigned long previousMillis = 0;   //Variables used for LCD refresh loop
unsigned long currentMillis = 0;    //Variables used for LCD refresh loop
int Rotary_counter = 0;             //Variable used to store the encoder position
int Rotary_counter_prev = 0;        //Variable used to store the previous value of encoder
bool clk_State;                     //State of the CLK pin from encoder (HIGH or LOW)
bool Last_State;                    //Last state of CLK pin from encoder (HIGH or LOW)
bool dt_State;                      //State of the DT pin from encoder (HIGH or LOW)
int Menu_level = 1;                 //Menu is strucured by levels
int Menu_row = 1;                   //Each level could have different rows
int push_count_ON = 0;              //Variable sued as counter to detect when a button is really pushed (debaunce)
int push_count_OFF = 0;             //Variable sued as counter to detect when a button is NOT pushed (debaunce)
String space_string = "______";     //used to print a line on LCD
String space_string_mA = "____";    //used to print a line on LCD
String pause_string = "";           //used to print something on LCD
bool SW_STATUS = false;             //Store the status of the rotary encoder push button (pressed or not)
bool SW_red_status = false;         //Store the status of the pause/resume button (pressed or not)
bool pause = false;                 //store the status of pasue (enabeled or disabled)

//Variables for storing each decimal for current, resistance and power. 
byte Ohms_0 = 0;
byte Ohms_1 = 0;
byte Ohms_2 = 0;
byte Ohms_3 = 0;
byte Ohms_4 = 0;
byte Ohms_5 = 0;
byte Ohms_6 = 0;

byte mA_0 = 0;
byte mA_1 = 0;
byte mA_2 = 0;
byte mA_3 = 0;
byte mA_4 = 0;

byte mW_0 = 0;
byte mW_1 = 0;
byte mW_2 = 0;
byte mW_3 = 0;
byte mW_4 = 0;

//Variables for ADC readings
float ohm_setpoint = 0;
float mA_setpoint = 0;
float mW_setpoint = 0;
int dac_value = 0;
float W_setpoint = 0;
float A_setpoint = 0;





void setup() {
  lcd.init();                 //Start i2c communication with the LCD
  lcd.backlight();            //Activate backlight
  
  lcd.createChar(0, arrow);   //create the arrow character
  lcd.createChar(1, ohm);     //create the ohm character
  lcd.createChar(2, up);      //create the up arrow character

  Serial.begin(9600);
   Wire.begin();
  
  lcd.clear();

startup();

 
  delay(3000);

  lcd.clear();  
  lcd.setCursor(2,0);                             
lcd.print("PROCESSING...");


  pinMode (OC1A_PIN, OUTPUT);
  
  PCICR |= (1 << PCIE0);      //enable PCMSK0 scan                                                 
  //PCMSK0 |= (1 << PCINT0);  //Set pin D8 trigger an interrupt on state change. 
  PCMSK0 |= (1 << PCINT1);    //Pin 9 (DT) interrupt. Set pin D9 to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);    //Pin 10 (CLK) interrupt. Set pin D10 to trigger an interrupt on state change.
  DDRB &= B11111001;          //Pins 8, 9, 10 as input  
  pinMode(Buzzer,OUTPUT);     //Buzzer pin set as OUTPUT
  digitalWrite(Buzzer, LOW);  //Buzzer turned OFF
  pinMode(SW,INPUT_PULLUP);       //Encoder button set as input with pullup
  pinMode(SW_blue,INPUT_PULLUP);  //Menu button set as input with pullup
  pinMode(SW_red,INPUT_PULLUP);   //Stop/resume button set as input with pullup
  delay(10);

ads.begin();            //  16 bit ADC, which will allow values between 0 and 65535
ads.setGain(GAIN_ONE);    //    +/- 4.096V  1 bit = 0.125mV
delay(30);
MCP.begin();

  delay(10);

  MCP.writeDAC(0,false); //Set DAC voltage output at 0V (MOSFET turned off)
 
  MCP.ready();
  delay(30);
   
  previousMillis = millis();
delay(4000);

}
//-----------------BEGIN LOOP----------------
void loop() {

  fanControl();  //fan control
  
  if(!digitalRead(SW_red) && !SW_red_status){
    push_count_OFF+=1;
    if(push_count_OFF > 10){  
      tone(Buzzer, 1000, 300);          
      pause = !pause;
      SW_red_status = true;
      push_count_OFF=0;
    }   
  }
  if(digitalRead(SW_red) && SW_red_status){
    SW_red_status = false;
  }

  

  
  if(Menu_level == 1)
  {
    if(!digitalRead(SW) && !SW_STATUS)    {
      
      Rotary_counter = 0;
      tone(Buzzer, 500, 20);
      if(Menu_row == 1){
        Menu_level = 2;
        Menu_row = 1;
      }
      else if(Menu_row == 2){
        Menu_level = 3;
        Menu_row = 1;
      }
      else if(Menu_row == 3){
        Menu_level = 4;
        Menu_row = 1;
      }
      
      SW_STATUS = true;
    }

    if(digitalRead(SW) && SW_STATUS)
    {      
      SW_STATUS = false;
    }


    
    
    if (Rotary_counter <= 4)
    {
      Menu_row = 1;
    }
    else if (Rotary_counter > 4 && Rotary_counter <= 8)
    {
      Menu_row = 2;
    }
    else if (Rotary_counter > 8 && Rotary_counter <= 12)
    {
      Menu_row = 3;
    }

    if(Rotary_counter < 0)
    {
      Rotary_counter = 0;
    }
    if(Rotary_counter > 12)
    {
      Rotary_counter = 12;
    }
    
    currentMillis = millis();
    if(currentMillis - previousMillis >= Delay){
      previousMillis += Delay;
      if(Menu_row == 1)
      {
        lcd.clear();
        lcd.setCursor(0,1);
        lcd.write(0); 
        lcd.print(" Cnt Load");
        lcd.setCursor(-4,2);
        lcd.print("  Cnt Current"); 
        lcd.setCursor(-4,3);
         lcd.print("  Cnt Power"); 
      }
    
      else if(Menu_row == 2)
      {
        lcd.clear();
        lcd.setCursor(0,1);     
        lcd.print("  Cnt Load");
        lcd.setCursor(-4,2);
        lcd.write(0);
        lcd.print(" Cnt Current"); 
        lcd.setCursor(-4,3);
         lcd.print("  Cnt Power"); 
      }
    
      else if(Menu_row == 3)
      {
        lcd.clear();
        lcd.setCursor(0,1);
         lcd.print("  Cnt Load"); 
         lcd.setCursor(-4,2);
         lcd.print("  Cnt Current"); 
         lcd.setCursor(-4,3);  
        lcd.write(0);   
        lcd.print(" Cnt Power");    
      }
    }
  }


  if(Menu_level == 2)
  {
    if(Rotary_counter < 0)
    {
      Rotary_counter = 0;
    }
    if(Rotary_counter > 9)
    {
      Rotary_counter = 9;
    }
    if(!digitalRead(SW) && !SW_STATUS)
    {
      tone(Buzzer, 500, 20);
      push_count_ON = push_count_ON + 1;
      push_count_OFF = 0;
      if(push_count_ON > 20)
      {
        Menu_row = Menu_row + 1;
        if(Menu_row > 7)
        {
          Menu_level = 5;
          pause = false;
          ohm_setpoint = Ohms_0*1000000 + Ohms_1*100000 + Ohms_2*10000 + Ohms_3*1000 + Ohms_4*100 + Ohms_5*10 + Ohms_6; 
          
        }
        Rotary_counter = 0;
        SW_STATUS = true;
        space_string = space_string + "_";
        push_count_ON = 0;
      }      
    }

    if(digitalRead(SW) && SW_STATUS)
    {      
      push_count_ON = 0; 
      push_count_OFF = push_count_OFF + 1; 
      if(push_count_OFF > 20){
        SW_STATUS = false;
        push_count_OFF = 0;
      }
        
    }
    

    if(Menu_row == 1)
    {
      Ohms_0 = Rotary_counter;      
    }
    if(Menu_row == 2)
    {
      Ohms_1 = Rotary_counter;      
    }
    if(Menu_row == 3)
    {
      Ohms_2 = Rotary_counter;
    }
    if(Menu_row == 4)
    {
      Ohms_3 = Rotary_counter;     
    }
    if(Menu_row == 5)
    {
      Ohms_4 = Rotary_counter;      
    }
    if(Menu_row == 6)
    {
      Ohms_5 = Rotary_counter;      
    }
    if(Menu_row == 7)
    {
      Ohms_6 = Rotary_counter;      
    }
    
    currentMillis = millis();
    if(currentMillis - previousMillis >= Delay){
      previousMillis += Delay;
      lcd.clear();
      lcd.setCursor(0,0);     
      lcd.print("Ohms: ");
      lcd.print(Ohms_0);
      lcd.print(Ohms_1);
      lcd.print(Ohms_2);
      lcd.print(Ohms_3);
      lcd.print(Ohms_4);
      lcd.print(Ohms_5);
      lcd.print(Ohms_6);
      lcd.setCursor(0,1);    
      lcd.print(space_string);
      lcd.write(2);
   
      // lcd.setCursor(-4,2);     
    // lcd.println("voltage :");
    //  lcd.setCursor(-4,3);     
     //lcd.println("Current :");
     
    }

    if(!digitalRead(SW_blue)){
      Menu_level = 1;
      Menu_row = 1;
      Rotary_counter = 0;
      Rotary_counter_prev = 0;
      MCP.setValue(0);
      previousMillis = millis();
      SW_STATUS = true;
      space_string = "______";    
      ohm_setpoint = 0;  
      Ohms_1 = 0;
      Ohms_2 = 0;
      Ohms_3 = 0;
      Ohms_4 = 0;
      Ohms_5 = 0;
      Ohms_6 = 0;
    }
    if(!digitalRead(SW_blue)){
      Menu_level = 1;
      Menu_row = 1;
      Rotary_counter = 0;
      Rotary_counter_prev = 0;
      MCP.setValue(0);
      previousMillis = millis();
      SW_STATUS = true;
      space_string = "______";    
      ohm_setpoint = 0;  
      Ohms_1 = 0;
      Ohms_2 = 0;
      Ohms_3 = 0;
      Ohms_4 = 0;
      Ohms_5 = 0;
      Ohms_6 = 0;
    }
  }







  if(Menu_level == 3)
  {
    if(Rotary_counter < 0)
    {
      Rotary_counter = 0;
    }
    if(Rotary_counter > 9)
    {
      Rotary_counter = 9;
    }
    
    if(!digitalRead(SW) && !SW_STATUS)
    {
      tone(Buzzer, 500, 20);
      push_count_ON = push_count_ON + 1;
      push_count_OFF = 0;
      if(push_count_ON > 20)
      {
        Menu_row = Menu_row + 1;
        if(Menu_row > 5)
        {
          Menu_level = 6;
          pause = false;
          mA_setpoint = mA_0*10000 + mA_1*1000 + mA_2*100 + mA_3*10 + mA_4; 
          
        }
        Rotary_counter = 0;
        SW_STATUS = true;
        space_string_mA = space_string_mA + "_";
        push_count_ON = 0;
      }      
    }

    if(digitalRead(SW) && SW_STATUS)
    {      
      push_count_ON = 0; 
      push_count_OFF = push_count_OFF + 1; 
      if(push_count_OFF > 20){
        SW_STATUS = false;
        push_count_OFF = 0;
      }
        
    }
    

    if(Menu_row == 1)
    {
      mA_0 = Rotary_counter;      
    }
    if(Menu_row == 2)
    {
      mA_1 = Rotary_counter;      
    }
    if(Menu_row == 3)
    {
      mA_2 = Rotary_counter;
    }
    if(Menu_row == 4)
    {
      mA_3 = Rotary_counter;     
    }
     if(Menu_row == 5)
    {
      mA_4 = Rotary_counter;     
    }
    
    
    currentMillis = millis();
    if(currentMillis - previousMillis >= Delay){
      previousMillis += Delay;
      lcd.clear();
      lcd.setCursor(0,0);     
      lcd.print("mA: ");
      lcd.print(mA_0);
      lcd.print(mA_1);
      lcd.print(mA_2);
      lcd.print(mA_3);  
      lcd.print(mA_4);  
         
      lcd.setCursor(0,1);    
      lcd.print(space_string_mA);
      lcd.write(2);
      

 lcd.setCursor(0,2);     
     lcd.println("voltage :");
     
    }
    if(!digitalRead(SW_blue)){
      Menu_level = 1;
      Menu_row = 1;
      Rotary_counter = 0;
      Rotary_counter_prev = 0;
      MCP.setValue(0);
      previousMillis = millis();
      SW_STATUS = true;
      space_string_mA = "____";  
      mA_setpoint = 0;   
      mA_0 = 0;
      mA_1 = 0;
      mA_2 = 0;  
      mA_3 = 0;  
       mA_4 = 0;  
              
    }     
  }



  if(Menu_level == 4)
  {
    if(Rotary_counter < 0)
    {
      Rotary_counter = 0;
    }
    if(Rotary_counter > 9)
    {
      Rotary_counter = 9;
    }
    
    if(!digitalRead(SW) && !SW_STATUS)
    {
      tone(Buzzer, 500, 20);
      push_count_ON = push_count_ON + 1;
      push_count_OFF = 0;
      if(push_count_ON > 20)
      {
        Menu_row = Menu_row + 1;
        if(Menu_row > 5)
        {
          Menu_level = 7;
          pause = false;
          mW_setpoint = mW_0*10000 + mW_1*1000 + mW_2*100 + mW_3*10 + mW_4; 
          
        }
        Rotary_counter = 0;
        SW_STATUS = true;
        space_string_mA = space_string_mA + "_";
        push_count_ON = 0;
      }      
    }

    if(digitalRead(SW) && SW_STATUS)
    {      
      push_count_ON = 0; 
      push_count_OFF = push_count_OFF + 1; 
      if(push_count_OFF > 20){
        SW_STATUS = false;
        push_count_OFF = 0;
      }
        
    }
    

    if(Menu_row == 1)
    {
      mW_0 = Rotary_counter;      
    }
    if(Menu_row == 2)
    {
      mW_1 = Rotary_counter;      
    }
    if(Menu_row == 3)
    {
      mW_2 = Rotary_counter;
    }
    if(Menu_row == 4)
    {
      mW_3 = Rotary_counter;
    }
    if(Menu_row == 5)
    {
      mW_4 = Rotary_counter;
    }
    
    
    
    currentMillis = millis();
    if(currentMillis - previousMillis >= Delay){
      previousMillis += Delay;
      lcd.clear();
      lcd.setCursor(0,0);     
      lcd.print("mW: ");
      lcd.print(mW_0);
      lcd.print(mW_1);
      lcd.print(mW_2);
      lcd.print(mW_3); 
      lcd.print(mW_4);            
      lcd.setCursor(0,1);    
      lcd.print(space_string_mA);
      lcd.write(2);

       lcd.setCursor(0,2);     
     lcd.println("voltage :");
    }
    if(!digitalRead(SW_blue)){
      Menu_level = 1;
      Menu_row = 1;
      Rotary_counter = 0;
      Rotary_counter_prev = 0;
      MCP.setValue(0);
      previousMillis = millis();
      SW_STATUS = true;
      space_string_mA = "____";
      mW_setpoint = 0;
      mW_0 = 0;
      mW_1 = 0;
      mW_2 = 0;  
      mW_3 = 0; 
      mW_4 = 0;     
    }
  }


//--------------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------


  //Constant Current Mode
  if(Menu_level == 6)
  {
    if(Rotary_counter > Rotary_counter_prev)
    {
      mA_setpoint = mA_setpoint + 1;
      Rotary_counter_prev = Rotary_counter;
    }

    if(Rotary_counter < Rotary_counter_prev)
    {
      mA_setpoint = mA_setpoint - 1;
      Rotary_counter_prev = Rotary_counter;
    }

    int16_t voltage_load, voltage_btread; 
    float voltage_on_load, sensosed_voltage, voltage_read, power_read;
    
       voltage_btread = ads.readADC_SingleEnded(1);  //(Read battery voltage sensor)
     voltage_load = ads.readADC_SingleEnded(0);      //Read  voltage shunt R, sensor(decide current)

     //Read  voltage shunt R----------
    voltage_on_load = (voltage_load * 0.125);    //ADS1115 has  gain. this is for it
    voltage_on_load = (voltage_on_load/4.7 )*0.03;

      //Read battery voltage-----
   voltage_read = (voltage_btread * 0.125)/1000;    // ADS1115 has  gain. this is for it
    voltage_read = (voltage_read/0.1039);
   
    
    power_read = voltage_on_load * voltage_read;

    A_setpoint = (mA_setpoint/1000)*0.03;
  dac_value = (( A_setpoint*4071.57)/5.03);  //set value to mcp4725 to drive fet

    Serial.println("ads convert value :");
    Serial.println(voltage_on_load,5);
    
    if(!pause){
      MCP.setValue(dac_value);
      pause_string = "";
    }
    else{
      MCP.setValue(0);
      pause_string = " PAUSE";
    }

    

    currentMillis = millis();
    if(currentMillis - previousMillis >= Delay){
      previousMillis += Delay;
      lcd.clear();
      lcd.setCursor(0,0); 
      lcd.print("I:");    
      lcd.print(mA_setpoint,0); lcd.print("mA  ");
      //lcd.print("  ");
      lcd.print(temp); lcd.print("C");
   lcd.setCursor(0,1); 
      lcd.print("----------------");    
      
      lcd.setCursor(-4,2);    
      lcd.print(voltage_on_load,3);  lcd.print("A"); lcd.print("  "); lcd.print(voltage_read);  lcd.print("V"); 
       lcd.setCursor(-4,3);    
      lcd.print(power_read,2);  lcd.print("W");
      lcd.print(pause_string);
    }
    if(!digitalRead(SW_blue)){
      Menu_level = 1;
      Menu_row = 1;
      Rotary_counter = 0;
      Rotary_counter_prev = 0;
      MCP.setValue(0);
      previousMillis = millis();
      SW_STATUS = true;
      space_string_mA = "____";  
      mA_setpoint = 0;   
      mA_0 = 0;
      mA_1 = 0;
      mA_2 = 0;      
    }      
  }



}//-------------------------//end void loop



ISR(PCINT0_vect){  
cli(); //stop interrupts happening before we read pin values
clk_State =   (PINB & B00000100); //pin 10 state? 
dt_State  =   (PINB & B00000010); 
if (clk_State != Last_State){
  // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
  if (dt_State != clk_State){ 
    Rotary_counter ++;    
    tone(Buzzer, 700, 5);
    Last_State = clk_State; // Updates the previous state of the outputA with the current state
    sei(); //restart interrupts
  }
  else {
    Rotary_counter --;  
    tone(Buzzer, 700, 5); 
    Last_State = clk_State; // Updates the previous state of the outputA with the current state    
    sei(); //restart interrupts
  } 
 }  
}
////////////////////fan and temp control//////////////////////

void fanControl (void) {
  temp1 = sensA.getTemp();
   temp2 = sensB.getTemp();
  int fanSpeed = setPwmDuty;
  temp = temp1 ;
  
  if (temp < tempMin) {      //is temperature lower than minimum setting
    fanSpeed = 0;            //fan turned off
    digitalWrite(OC1A_PIN, LOW);
  }
   if ((temp >= tempMin) && (temp <= tempMax)){
    fanSpeed = map(temp, tempMin, tempMax, 120, 255);
    //Serial.print("Fan Speed");  //Test only
    //Serial.println(fanSpeed);   //test only
    analogWrite(OC1A_PIN, fanSpeed);
  
   
  }
}

//===============================stsart up screen==============================
void startup(void){
  lcd.setCursor(0,0);
  lcd.print("----------------"); 
 
  lcd.setCursor(0,1);
  lcd.print("|"); 
  lcd.setCursor(16,1);
  lcd.print("|"); 

   lcd.setCursor(-4,2);
  lcd.print("|"); 
  lcd.setCursor(11,2);
  lcd.print("|"); 
  
   lcd.setCursor(-4,3);
  lcd.print("----------------"); 
  
  delay(500);
  lcd.setCursor(0,2);
  lcd.print("DC  LOAD");  
  delay(3000);
  
  
  }
//==============================================fan frequncy set=======================
void setPwmDuty(byte duty) {
  OCR1A = (word) (duty*TCNT1_TOP)/100;
}
