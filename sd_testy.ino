#include <sd_raw_roland.h>
boolean initializated;
int red = 7;
int blue = 6;

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

void setup() { 
  Serial.begin(9600); 
  pinMode(red, OUTPUT); 
  pinMode(blue, OUTPUT); 
  initializated = false;
  int current = 0;
  int maxtries = 2;
  while(!initializated && current < maxtries){
     int r = sd_raw_init();
     if(r == 0){
       Serial.print("\nInitialization failed");
       Serial.print("\nRetrying...\n");
    } else if(r == 1){ 
       Serial.print("\nCard Initialized");       
       initializated = true;
    }
    current++;
  }
  if(initializated){
    digitalWrite(blue, HIGH);
    print_info();
  }else{
    digitalWrite(red, HIGH);
    Serial.print("\nInitialization failed after several retries");
  }
}

// unused loop
void loop() {

}
