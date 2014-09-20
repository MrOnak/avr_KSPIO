# AVR KSP-IO
This is a conversion from the Arduino plugin-code for Kerbal Space Program 
written by a developer I only know as 'Zitronen'. 

Zitronen's code is written for use with the Arduino GUI and was released under
Creative Commons Attribution (BY).

Kerbal Space Program is developed by SQUAD (http://kerbalspaceprogram.com/)

My attempt here is to merely port it to native AVR C which can be understood
by Atmel microprocessors, without the use of the Arduino GUI. 

The original plugin allows the development of custom game controllers by 
enabling the Arduino to read information from the game such as a flight and 
vessel information (altitude, speed, fuel remaining, ...) and also send 
steering inputs back to the game (rotation, translation, systems such as RCS, 
SAS).

I've tested the code on an Atmel ATMega328p without problems. It should also
work flawlessly on most other AVR chips though, provided you equip it with a
USB-to-serial adapter.

## USAGE
The Makefile provided was tested with Win-AVR32 on Windows7, 64bit. I'm using
USBTinyISP as programmer but you can easily change that in the Makefile itself.

For deeper information about how to use this code please see Zitronen's 
original posts in the Kerbal Space Program forum:
    http://forum.kerbalspaceprogram.com/threads/66393
	
## WHAT THIS CODE IS NOT
This code provides a working foundation to develop custom game controllers and
information panels for Kerbal Space Program. 

I understand that Zitronen's intention with the original code (KSPIODemo8) was
to provide a showcase of what can be done and how it is done. This intention
stays true with my port.

This code will not give you a working game controller. But it will give you a 
toolkit to develop just that yourself.

I have actively tried to avoid changing or restructuring the code in order to
make it relatively easy to keep it in sync with later versions of Zitronen's 
code. 

## CONTRIBUTION
My contribution here is limited to a translation of Zitronen's code to AVR C. 

This involved swapping the Arduino Serial library with a AVR UART library 
written by Peter Fleury and Tim Sharpe (see LICENSES below).

I've also provided an adaptation of the Arduino millis() function used in 
Zitronen's original code.

## LICENSES
./uart
The UART library in use is based on a library written by Peter Fleury 
    <pfleury@gmx.ch>   http://jump.to/fleury
    his code was published under GNU General Public License, version 2.

The library was further extended by Tim Sharpe to include additional functions 
    such as uart_available() and uart_flush(). (See uart.h for details).
    His work is released under the GNU General Public License.

Mr Sharpe's additions in turn are based on the Arduino HardwareSerial.h,
    released under LGPL, version 2.1.
	
The full text of the license can be found in the ./uart folder and online:
    http://opensource.org/licenses/GPL-2.0
	
./kspio
The code in the kspio folder is a direct 'translation' from the KSPIO 
plugin code, version 0.15.3, released under Creative Commons Attribution (BY)
by it's author, 'Zitronen'.

The version of the code in ./kspio remains licensed under Creative Commons
Attribution (BY).

The full text of the license can be found in the ./kspio folder and online:
    http://creativecommons.org/licenses/by/4.0/legalcode
	