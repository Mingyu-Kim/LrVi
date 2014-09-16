#define Receiver_Pin 2      //Defines where the radio PPM SUM is connected to

#define Radio_Min 1125      //Minimum duration of raw radio pulse
#define Radio_Max 1860      //Maximum duration of raw radio pulse
#define Throt_Min 0         //Minimum value of parsed throttle
#define Throt_Max 1000      //Maximum value of parsed throttle
#define Parse_Min -500      //Minimum value of parsed radio
#define Parse_Max 500       //Maximum value of parsed radio

#define CH1 0    //defines CH number to numeric values for easy code reading
#define CH2 1
#define CH3 2
#define CH4 3
#define CH5 4
#define CH6 5

int radio[8];  //array for storing raw radio input data
int parsed_radio[6];  //array for storing parsed radio data



void setup()
{
  Serial.begin(115200);        //begins serial data stream
  Serial.println("ready");     //I am ready!
  radio_init();                 //Initializes radio input codes

}

void loop()
{
  parse();          //Parses raw radio data
  serial_radio();   //Shows serial data to PC
  delay(100);  //wait wait wait
}



void parse()  //takes data from raw radio data and parse them into usable form and saves them on parsed_radio
{
   parsed_radio[CH1] = map(radio[CH1], Radio_Min, Radio_Max, Parse_Min, Parse_Max);
   parsed_radio[CH2] = map(radio[CH2], Radio_Min, Radio_Max, Parse_Min, Parse_Max);
   parsed_radio[CH3] = map(radio[CH3], Radio_Min, Radio_Max, Throt_Min, Throt_Max);
   parsed_radio[CH4] = map(radio[CH4], Radio_Min, Radio_Max, Parse_Min, Parse_Max);
   parsed_radio[CH5] = map(radio[CH5], Radio_Min, Radio_Max, Parse_Min, Parse_Max);
   parsed_radio[CH6] = map(radio[CH6], Radio_Min, Radio_Max, Throt_Min, Throt_Max);
}

void serial_radio()     //Outputs Parsed Radio data to Serial
{
  for(int CH = 0; CH < 6; CH++)
  {
    Serial.print("CH");
    Serial.print(CH+1);
    Serial.print(": ");
    Serial.print(parsed_radio[CH]);
    Serial.print("  ");
  }
  Serial.println("");
}


/*********
Radio Receiving Code has been forked from
http://code.google.com/p/read-any-ppm/
*********/
void radio_init()
{
  pinMode(Receiver_Pin, INPUT);
  attachInterrupt(Receiver_Pin - 2, read_ppm, CHANGE);
  TCCR1A = 0;  //reset timer1
  TCCR1B = 0;
  TCCR1B |= (1 << CS11);  //set timer1 to increment every 0,5 us
}

void read_ppm(){  //leave this alone
  static unsigned int pulse;
  static unsigned long counter;
  static byte channel;

  counter = TCNT1;
  TCNT1 = 0;

  if(counter < 1020){  //must be a pulse if less than 510us
    pulse = counter;
  }
  else if(counter > 3820){  //sync pulses over 1910us
    channel = 0;
  }
  else{  //servo values between 510us and 2420us will end up here
    radio[channel] = (counter + pulse)/2;
    channel++;
  }
}


