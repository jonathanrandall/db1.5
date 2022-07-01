# youtube video

https://youtu.be/8JkGv6YmDFk 


# db1.5
** The Robot chasis follows the build at dronebot workshop. 

See https://dronebotworkshop.com/build-a-real-robot/ for instructions on how I build the chasis.

This includes the navigation controls for the main esp32 board on the robot. This board is connected to:
* the two cytron MD10C motor controllers
* the oled screen which gives the IP of the board.

It also contains the javascript/html/css files which are needed to bring up the navigation controls in the webpage. You will need to load these into SPIFFS on esp32.
I have cut and pasted code from www.randomnerdtutorials.com for the implementation of the controls.

You will need to create in an ssid_stuff.h file with your network credentials to get it to connect.

There is also some code to connect with esp32-now to the peripheral board.

# esp32_cam_websocket
This has the controls for the esp32 cams. I have used websockets with a fixed IP, which is hardcoded in the javascript file include in the **db1.5**
This uses websocket protocol. I have cut and pasted code form http://www.iotsharing.com/2020/03/demo-48-using-websocket-for-camera-live.html for this implementation.

# peripheral_board_top_1

This board uses esp32-now to get the state of the robot and changes the LEDs colour depending on the state. Also controls the touchscreen.

