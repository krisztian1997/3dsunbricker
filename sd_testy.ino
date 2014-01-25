#include <sd_raw_roland.h>
// arduinos already have this led on the board, so why not use it
/*int statusled = 3;*/
boolean initializated;
int red = 7;
int blue = 6;
void blink(int n){
  /*for(int i = 0; i < n; i++){
        digitalWrite(statusled, HIGH);   
        delay(1000);               
        digitalWrite(statusled, LOW);  
  }*/
}

int print_info()
{
    struct sd_raw_info sdinfo;
    if(!sd_raw_get_info(&sdinfo))
    {
        return 0;
    }
    Serial.println();
    Serial.print("revision: ");
    Serial.print(sdinfo.revision,HEX);
    Serial.println();
    Serial.print("Serial: 0x");
    Serial.print(sdinfo.serial,HEX);
    Serial.println();
    Serial.print("Manufacturing date (mm/yy): ");
    Serial.print(sdinfo.manufacturing_month,DEC);
    Serial.print("/");
    Serial.print(sdinfo.manufacturing_year,DEC);
    Serial.println();
    Serial.print("Size: ");
    Serial.print(sdinfo.capacity,DEC);
    Serial.println();
    Serial.print("Copied: ");
    Serial.print(sdinfo.flag_copy,DEC);
    Serial.println();
    Serial.print("Write protected: ");
    Serial.print(sdinfo.flag_write_protect_temp,DEC);
    Serial.println();
    Serial.print("Temporal write protected: ");
    Serial.print(sdinfo.flag_write_protect,DEC);
    Serial.println();
    Serial.print("Format: ");
    Serial.print(sdinfo.format,DEC);
    Serial.println();
    return 1;
}
// the setup routine runs once when you press reset:


void setup() { 
  Serial.begin(9600); 
  pinMode(red, OUTPUT); 
  pinMode(blue, OUTPUT); 
  initializated = false;
  // intialize the led pin as output
  /*pinMode(statusled, OUTPUT); */
  //turn the led on for 1s then off
  blink(1);
  if(sd_raw_init() == 0){
     Serial.print("Initialization failed\n");
     blink(2); // initialization error, blink two times
     digitalWrite(red, HIGH);
  } else {
     Serial.print("Card Initialized\n");
     blink(1);
     digitalWrite(blue, HIGH);
     initializated = true;
  }
  if(initializated){
         print_info();
  }
}

// unused loop
void loop() {

}
