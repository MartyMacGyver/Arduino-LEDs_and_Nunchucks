Arduino-LEDs_and_Nunchucks
==========================

An Arduino project utilizing a Nyko Kama wireless Wii nunchuck to control a WS2811-based LED strip.

This ought to work with any Wii nunchuck, but I can't guarantee it. I'm curious to hear if anyone tries this out with a genuine (wired) Nintento nunchuck.

Technically the LED strip is optional... the room will just be darker without it. :-D Use verbose output and the serial monitor to see what the nunchuck is up to (it slows things down a bit though).

Sometimes after uploading modified code the nunchuck will fail to respond (it basically hangs while trying to initialize), even if you press the reset button on the Arduino. Solution: Power-cycle the wired controller or the wireless transmitter by momentarily disconnecting the 3.3V line. You might need to press reset on the Arduino as well. If you're using a wireless controller like this you'll need to re-sync it (for some reason controller won't re-sync with its transmitter until it times out... then I press the button and all is well again.)

