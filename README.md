3dsunbricker
============

3DS unbricker for Arduino/AVR written in C/C++

**Disclaimer:** I am not responsible for any damage created by not using my software correcly, or using the wrong hardware. You will need a NAND backup created before the bricking, and a hardware mod to can connect the 3DS to arduino and flash the NAND back. Never try soldering something so small like the 3DS mobo without some soldering knowledge, if you really want to do it, go to a local phone shop and maybe they will do it for you.

**Components needed:**
<ul>
  <li> A NAND BACKUP BEFORE THE CONSOLE GOT BRICKED </li>
  <li> Arduino UNO or any other Arduino model with SPI support </li>
  <li> A SD card shield, personally I recommand the ones made by LC Studio, they are around $1. If you cant get one then you will need to make a voltage divider, be very carefull because Arduino I/O pins are 5v and eMMC uses 3v3 </li>
  <li> Some way to connect the SD shield to the arduino (male-female dupont wires will do the job) </li>
  <li> Soldering knowledge to solder 5 wires to the points on the 3DS mobo </li>
  <li> An SD card adapter, best one is mini sd card adapter -> normal sd card </li>
  <li> Some ways to restore the NAND backup </li>
</ul>

**Instructions how to setup the arduino IDE (thanks khaalan @gbatemp):**
Okay so you first want to install the arduino development application in windows (I assume windows for most users) then start the development IDE (then close it immediately) and create the following folder structure (if you are on xp, its under document and settings) C:\Users\...\Documents\Arduino\libraries\sd\_raw\_roland (you will have to create the sdrawroland folder yourself) and copy everything except sd_testy.ino into that folder. After you've done this you can double click on sd\_testy.ino, follow its prompts about creating directories. You will want to step through configuring your Arduino IDE for your device (http://arduino.cc/en/Guide/Windows#toc7), then you can click File> Upload - this will upload your device to your arduino. Assuming you have all of your connections made the Arduino will reboot and execute the code.

**Short instruction of using the arduino with my code:**
<ol>
<li> If you didnt did it already, program the Arduino with my code </li>
<li>  Connect the 3DS to the arduino according to the following schematic (you will need an SD card shield, or a voltage divider):     
<table border="0">
<tr>
<th>3DS</th>
<th>ARDUINO</th>
</tr>
<tr>
<td>CMD</td>
<td>MOSI</td>
</tr>
<tr>
<td>DAT0</td>
<td>MISO</td>
</tr>
<tr>
<td>DAT3</td>
<td>CS/SS</td>
</tr>
<tr>
<td>CLK</td>
<td>CLK</td>
</tr>
<tr>
<td>GND</td>
<td>GND</td>
</tr>
</table>

In case if you still dont understand, please take a look at those schematics:         
<a href="http://arduinodiy.files.wordpress.com/2012/03/sd-card.jpg">SD card schematic with voltage divider</a>,     
<a href="http://img441.imageshack.us/img441/2391/schematicuo7.jpg">Another SD card schematic with a different voltage divider</a>,   
<a href="http://tinyurl.com/p4dyv8q">Arduino UNO pinout</a>, 
<a href="http://imageshack.us/a/img18/7048/tvj.png">3DS solder points</a>
<a href="https://dl.dropboxusercontent.com/u/33926727/3ds/duino.png"><img src="https://dl.dropboxusercontent.com/u/33926727/3ds/duino.png" /></a>
</li>
<li> Power up the arduino</li>
<li> Power up your 3DS (XL) console</li>
<li>  Open the arduino IDE and watch the serial monitor, after the card is initialized choose the erase option then terminate the programs execution</li>
<li> Flash your NAND backup back and have fun.</li>
</ol>


	
	
