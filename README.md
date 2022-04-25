# This project

A two-wheeled self-balancing robot based on an Arduino micro controller and 12 V DC motors. A logic is implemented on top of a cascaded PID controller to obtain the best performance in speed and tilt control. For the user interface, a mobile application was developed using Ionic

## Main Code

The main file of the Arduino program is main.ino
which is in:

``` sh
./main/main.ino
```

Here we import the libraries used to interact with the MPU6050 inertial unit and import custom libraries for the Bluetooth communication task, control algorithms and measurement filtering.

<img src="https://user-images.githubusercontent.com/37028825/164944606-1e68961e-2881-43e3-976e-58d663b78416.png" width="600"/>

You can find those packages in the path:

``` sh
./USED_LIBRARIES
```

**if you don't know how to add libraries to the arduino IDE visit this link**:
[https://docs.arduino.cc/software/ide-v1/tutorials/installing-libraries]

## Block Diagram of Control Algorithms

![image](https://user-images.githubusercontent.com/37028825/164944911-e4f5bff5-50cd-4e1b-9f39-7eb1f345e617.png)

## Hardware connection

![image](https://user-images.githubusercontent.com/37028825/164944809-b879fc69-ef65-486a-b801-3ddeb7aeae59.png)

## Mobile Application Images

![image](https://user-images.githubusercontent.com/37028825/164945038-bf0f2db6-e50d-4aa4-b6c9-dd862b1f382e.png)

### The repo for the mobile application is at:

[https://github.com/josealejandro2928/ionic-react-mobile-app-self-balancing-robot]

## Robot results

### Watch videos of the robot in this links

[![main video](https://media-exp1.licdn.com/dms/image/sync/C4E27AQHoYDjRXYum1Q/articleshare-shrink_800/0/1650225441705?e=2147483647&v=beta&t=MVa15-GMcQ24pGtz-Pgde-uTOCKB2VxyYCMe7VwTT0s)](https://www.youtube.com/watch?v=NTQIz3hWsak)

[![main video 2](https://media-exp1.licdn.com/dms/image/sync/C4D27AQEmOZ0t977IvQ/articleshare-shrink_800/0/1650744012047?e=2147483647&v=beta&t=uszsP1K_WxqRwiHf1cKw5MvTG6UA9ftaZmSf5iL4E4k)](https://www.youtube.com/watch?v=mijxUfsSrdE)

[![main video 2](https://media-exp1.licdn.com/dms/image/sync/C4E27AQGTRxc4k5051w/articleshare-shrink_800/0/1648242738932?e=2147483647&v=beta&t=efO4rj6X8Rad44_dpQQ8JhWYrrs8W4pU4eZYuNL0jNU)](https://www.youtube.com/watch?v=O6BeBPsemHU)
