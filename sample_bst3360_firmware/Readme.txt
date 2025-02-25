To load the firmware, go to this website:
https://www.pjrc.com/teensy/first_use.html
Then go to the next step: "Teensy Loader", choose your OS, and download the teensy loader program.

Load up Teensy.exe, connect the teensy, then press the button on the teensy to enter the bootloader mode.
Now drag and drop bst3360.hex to the teensy window, and press the Program button. Then, press the reset button. The teensy should now be loaded with the firmware.

To update the firmware, you can edit the main.c file. When you have compiled it, you can enter the bootloader mode by holding down the middle mouse button, whilst plugging the mouse in (useful since it means you don't have to open the mouse to press the teensy button every time you want to update the firmware).

Please see this forum post for more information:
http://www.overclock.net/forum/375-mice/1561041-reverse-engineering-3366-a-9.html#post26323358

The instructions there will work with this firmware. The only difference is that it doesn't mention the RGB LED pins, which are:
Red LED: B5

Green LED: B6

Blue LED: D7

--------
Instructions on how to operate the mouse
--------

The only thing you can do is to change the profile. 
By default it is set to 800 DPI with a red colour.
To change to a different profile, hold down one of the side buttons when plugging the mouse in:
Hold down the front side button for 1600 DPI with a purple colour.
Hold down the rear side button for 800 DPI with a pink colour.

--------
Instructions on how to change profile settings
--------

- To choose different colours, and change the brightness of the LED, edit this part of main.c. (Hint: p1 = profile 1):
uint8_t p1_led_colour = 1; // Red
 (DEFAULT PROFILE)	
uint8_t p2_led_colour = 2; // Pink
	
uint8_t p3_led_colour = 4; // Purple
	
uint8_t p1_led_brightness = 4; // 0 = 0, 1 = 0.25, 2 = 0.5, 3 = 0.75, 4 = 1
	
uint8_t p2_led_brightness = 4;
	
uint8_t p3_led_brightness = 4;

The colours and their respective numbers are:
White		0
Red		1
Pink		2
Magenta		3	
Violet		4
Blue		5	
Sky Blue	6
Cyan		7
Green		8
Toxic Green	9
Yellow		10
Orange		11

- To change the DPI is more complicated.

First, find the below part of the code in main.c
uint8_t dpis[] = {3, 7, 15};

Enter into the brackets which DPI you want available.
The numbers represent the DPI: for example (3+1)*100 = 400 DPI, (15+1)*100 = 1600 DPI.
So the below example can be read as: 400, 800, 1600.
uint8_t dpis[] = {3, 7, 15};

This one is 1000, 2000, 3000:
uint8_t dpis[] = {9, 19, 29}; 

The first number is for Profile 2 (rear side button)
The second number is for profile 1 (Default profile)
The third number is for profile 3 (front side button)

So if you use this setting:
uint8_t dpis[] = {3, 7, 15};

Then the DPIs will be assigned like this:
Profile 2: 400 DPI
Profile 1: 800 DPI
Profile 3: 1600 DPI

You can set any DPI value between 200-12,000.


--------
Disclaimer
--------
I am not responsible for any damage you cause to your property or yourself while following these instructions or using the firmware, these instructions are a rough guide, please take the time to learn from professional sources if you are unsure about anything.
