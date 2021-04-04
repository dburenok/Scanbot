# Scanbot


<img src="https://www.dburenok.com/img/scanbot_full2.jpg" alt="drawing" width="200"/>

https://www.youtube.com/watch?v=-Bxh61dTs7g

Scanbot is an Arduino-ESP32 based 360-degree point-cloud scanner, programmed in C++. It pairs to a computer over bluetooth and performs a full 3D scan of the environment.

Take a look at "Arduino_Project/main/main.ino" for the C++ source file that gets loaded onto the ESP-32.

And if you'd like to play around with the visualizer app, download Processing and load up app.pde in "Visualizer (open with Processing 3.5.4)". Press "o" once you've opened the app, and you will get a pop-up window asking you to select a file. Choose any of the .csv files in the scan folder to see the point-cloud scan!
