3dsunbricker
============

3DS unbricker for Arduino/AVR written in C/C++
**Disclaimer:** I am not responsible for any damage created by not using my software correcly, or using the wrong hardware. You will need a NAND backup created before the bricking, and a hardware mod to can connect the 3DS to arduino and flash the NAND back.
**Instructions how to setup the arduino IDE (thanks khaalan @gbatemp):**
Okay so you first want to install the arduino development application in windows (I assume windows for most users) then start the development IDE (then close it immediately) and create the following folder structure (using Win8.1 here so if you are on XP replace Users with Documents and Settings) C:\Users\Michael\Documents\Arduino\libraries\sd_raw_roland and copy everything except sd_testy.ino into that folder. After you've done this you can double click on sd_testy.ino, follow its prompts about creating directories. You will want to step through configuring your Arduino IDE for your device (http://arduino.cc/en/Guide/Windows#toc7), then you can click File> Upload - this will upload your device to your arduino. Assuming you have all of your connections made the Arduino will reboot and execute the code.
**Short instruction of using the arduino with my code:**
0. If you didnt did it already, programm the Arduino with my code
1. Connect the 3DS to the arduino according to the following schematic (you will need an SD card shield, or a voltage divider):
	|      3DS      |    ARDUINO    |
	| :-------------: | :-------------: |
	|   CMD         |   MOSI        |
	|   DAT0        |   MISO        |
	|   DAT3        |   CS/SS       |
	|   CLK         |   CLK         |
	|   GND         |   GND         |
In case if you still dont understand, please take a look at those schematics: 
	http://arduinodiy.files.wordpress.com/2012/03/sd-card.jpg
	http://img441.imageshack.us/img441/2391/schematicuo7.jpg
	http://www.gammon.com.au/images/ArduinoUno_R3_Pinouts.png
	http://imageshack.us/a/img18/7048/tvj.png
2. Power up the arduino
3. Power up your 3DS (XL) console
4. Open the arduino IDE and watch the serial monitor, after the card is initialized choose the erase option then terminate the programs execution
5. Flash your NAND backup back and have fun.
	
	
