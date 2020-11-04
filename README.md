# Marvin_Lora_Sensor
This code can be used on Marvin, a microcontroller with LoRa. Made it for a school assignment. 
In this example I used a Humidity&Temperature sensor attached to D12, D4.

Outputs 2 values (float hexadecimal). Those two values are set after each other so it only takes one payload. 
Each float hexadecimal contains of 8 bits so in total the payload is 16 bits long.
If you split the payload you will get the two values in hexadecimal format.
