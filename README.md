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

<img src="https://user-images.githubusercontent.com/37028825/165102821-c934f39b-0220-4a4f-b4c4-c7a608f18580.png" width="800"  />

### The repo for the mobile application is at:

[https://github.com/josealejandro2928/ionic-react-mobile-app-self-balancing-robot]

## Robot results

### Watch videos of the robot in this links

[![main video](https://user-images.githubusercontent.com/37028825/171403987-cf097c4e-5f7c-4eaf-8e32-ab204e08aa4c.jpeg)](https://www.youtube.com/watch?v=NTQIz3hWsak)

[![main video 2](https://user-images.githubusercontent.com/37028825/171404073-87242390-c586-41d4-9e78-adff395a45c2.jpeg)](https://www.youtube.com/watch?v=mijxUfsSrdE)

[![main video 3](https://user-images.githubusercontent.com/37028825/171404132-7b037bd7-13df-4bab-a089-fac8ab39bd72.jpeg)](https://www.youtube.com/watch?v=O6BeBPsemHU)

