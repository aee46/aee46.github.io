+++
title = "Lab 8"
description = "Robot Stunts"
weight = 8

[extra]
remote_image = "projects/stunt.jpg"
+++

Robot Stunts - Flip
======
For this lab, I chose to implement a flip stunt. This involves sending the robot at full speed towards a wall, and when the robot is roughly 1 foot away from the wall, it executes a flip and moves in an opposite direction away from the wall. I used one TOF sensor to measure the distance between the robot and the wall. When the robot reached a certain distance, I reversed the motors' direction, causing the robot to flip backwards.

Setup
======
Implementing this stunt involved both updates to my robot's software and its physical structure. In order for the flip to execute successfully, the front of the robot needs to be weighted heavier than the back. If this is not the case, the robot simply reverses direction when its motors change direction without executing a flip. To weight the front of the robot, I used some spare laundry money (roughly 5 dollars in quarters). I placed the quarters in a plastic bag to condense them and then taped this weight to the front of my robot. This added about 125 grams of weight to the front of my robot. Luckily there was space in the front compartment of the robot so I placed the weight inside. This prevented th weight from dragging on the ground once the robot flips. Another important thing to note is that the weight should be on the top of the robot. If it is on the bottom (which is how I initially implemented it), the robot will not flip. Shown below is a picture of my weight system:

<br>
<center><img src="/projects/weightlab8.jpg" alt="sensors" width="500" height="400"></center>

With this setup, I had to drive my robot upside down from its usual state. As you'll see in my code below, this results in driving the car with negative PWM values to move it forward and positive PWM values to drive it backwards. For the software setup, I implemented a simple bluetooth command using much of the same code used in previous labs.

```Arduino
case LAB8_FLIP:{

    lab8_first_time = true;
    lab8_second_time = false;
    
    distanceSensor2.startRanging();
    float lab8_start_time = millis();
    int index = 0;

    while(millis() - lab8_start_time < 3000){ // Run stunt for maximum of 3 seconds
        lab8_current_time = millis();
        lab8_record_data();

        if(lab8_current_distance > 1000){
            lab8_current_pwm = -100;
        }
        else{
            lab8_current_pwm = 100;
        }
        driveMotors(lab8_current_pwm);
        time_stamps[index] = lab8_current_time;
        tof1_stamps[index] = lab8_current_distance;
        DATA_READY_ARRAY[index] = lab8_data_ready;
        motor_speeds[index] = lab8_current_pwm;
        index += 1;
    }
    stop_motors();
    for(int i = 0; i < index; i++){ // If array does not fill in alloted time, only send recorded values and not empty indicies
        tx_estring_value.clear();
        tx_estring_value.append(time_stamps[i]);
        tx_estring_value.append("|");
        tx_estring_value.append(tof1_stamps[i]);
        tx_estring_value.append("|");
        tx_estring_value.append(DATA_READY_ARRAY[i]);
        tx_estring_value.append("|");
        tx_estring_value.append(motor_speeds[i]);
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
    }
    break;
}
```

The code executes for a total of three seconds before disabling the robot's motors. During these three seconds, the robot reads data from the front TOF sensor to determine when it should execute the flip. Through experimental testing, I determined the best distance to begin the flip execution was about one meter away from the wall. This caused the robot to actually flip just before crashing into the wall, but it needed time for the motors to reverse direction. When the flip execution was set to begin when the robot was one foot from the wall, it did not have enough time to flip and would instead crash. Once the robot is within one meter from the wall, it reverses the direction of the motors, resulting in a flip being executed. I recorded PWM and TOF data for debugging purposes. 

Note that since I did not complete lab 7 (I was severely ill with the flu and had used up my slip weeks), my TOF recording function uses linear extrapolation instead of the Kalman filter. Shown below is the code I use to record TOF data:

```Arduino
void lab8_record_data(){
  if(!distanceSensor2.checkForDataReady()){
    if(lab8_first_time == false && lab8_second_time == false){
      lab8_current_distance = lab8_last_reading + ((lab8_last_reading - lab8_previous_reading) / (lab8_last_time - lab8_previous_time)) * (millis() - lab8_last_time); 
    }
    else{
      lab8_current_distance = lab8_last_measured_distance;
    }
    
    lab8_data_ready = false;
  }
  else{
    lab8_current_distance = distanceSensor2.getDistance();
    lab8_last_measured_distance = lab8_current_distance;
    distanceSensor2.clearInterrupt();
    distanceSensor2.stopRanging();
    distanceSensor2.startRanging();
    lab8_data_ready = true;
    
    lab8_previous_reading = lab8_last_reading;
    lab8_last_reading = lab8_last_measured_distance;
    lab8_previous_time = lab8_last_time;
    lab8_last_time = lab8_current_time;
    if(lab8_first_time){
      lab8_first_time = false;
      lab8_second_time = true;
    }
    if(lab8_second_time){
      lab8_second_time = false;
    }
  }
}
```

Hardware Bugs
======
It wouldn't be a proper Fast Robots lab if something didn't break at the worst possible time. Unfortunately, I encountered two hardware issues while completing this lab. Luckily, I was able to solve both of them and complete the lab on time!

First, I ran into issues with one of my TOF sensors recording inaccurate measurements when the robot was moving forward. I tested this several times by starting my robot 2-3 meters away from a wall and driving it forwards at half speed. Despite being over 2 meters away from the wall, the TOF sensor recorded that it was less than a meter away, as seen in the graph below. The sensor was set to its Long Distance Mode which has a maximum range of four meters. Additionally, the sensor recorded accurate measurements when the motors were off, and also while the motors were running but the robot was held stationary. I am not sure what caused this issue, but I fixed it by using my second TOF sensor in place of the broken one. Unfortunately, this TOF sensor was originally placed on the side of my robot, so its wire cable was quite short compared to the original front TOF sensor. This resulted in the new TOF sensor getting unplugged from the QWIIC connector frequently after the robot would crash or execute a flip. Luckily, I was able to (mostly) fix this issue by using lots of tape.

Shown here is a graph depiciting my inaccurate TOF sensor measurements. The blue line indiciates TOF measurements while the orange line indicates the motor PWM value scaled by 10 so that you can see when the motors are on and off.
<br>
<center><img src="/projects/lab8tofbug.PNG" alt="sensors" width="500" height="400"></center>

The second issue that I encountered was my Artemis' battery connector breaking. This was the second time this has happened, so I need to look into a permanent way to avoid this.

<br>
<center><img src="/projects/lab8battery.jpg" alt="sensors" width="500" height="400"></center>

I attempted to crimp a new pin onto the broken wire, but I was unable to do this successfully. I do not have access to crimping tools that work well with such small connector pins. Luckily, I was able to borrow a battery from another student, allowing me to continue the lab.

Successful Flips
======
With my hardware issues fixed (for now), I was able to consistently execute the flip stunt. Shown below are three proper successes where the robot returns across the starting line by moving in a relatively straight line. I was able to produce consistent results (>80%) with having the robot move in a straight line after executing the flip without PID orientation control. Implementing PID orientation control would result in extremely precise control of the robot's movement after the flip, but I found it unneccessary for this stunt.

<center><a href="https://www.youtube.com/watch?v=eGy7wUVAQAU" title="Scope"><img src="/projects/flip1.PNG" alt="Scope" width="400" height="300" /></a></center>
<br>
<center><img src="/projects/lab8tof1.PNG" alt="sensors" width="400" height="300"><img src="/projects/lab8pwm1.PNG" alt="sensors" width="400" height="300"></center>

<center><a href="https://www.youtube.com/watch?v=eBpJt5gkR8U" title="Scope"><img src="/projects/flip2.PNG" alt="Scope" width="400" height="300" /></a></center>
<center><img src="/projects/lab8tof2.PNG" alt="sensors" width="400" height="300"><img src="/projects/lab8pwm2.PNG" alt="sensors" width="400" height="300"></center>

<center><a href="https://www.youtube.com/shorts/dTykkAJNZHk" title="Scope"><img src="/projects/flip3.PNG" alt="Scope" width="400" height="300" /></a></center>
<center><img src="/projects/lab8tof3.PNG" alt="sensors" width="400" height="300"><img src="/projects/lab8pwm3.PNG" alt="sensors" width="400" height="300"></center>

As seen in the above videos, the robot is able to reliably execute the flip stunt multiple times. The robot executes the stunt roughly 1 second after it begins moving towards the wall. The flip takes about 1 second to execute, and the robot returns to its starting position in about 1 second.

Bloopers
======
While not particularly exciting, I had my fair share of bloopers and unexpected behavior while completing this lab. This was mostly caused by the robot randomly disconnecting from Bluetooth or the TOF sensor getting disconnected. One blooper shows the car reversing backwards at the start. This indicates a TOF sensor measurement where it records a value less than 1000 mm, despite being farther than 1000 mm from the wall. The second blooper shows the robot successfully complete a flip but then decide to try to execute a flip again, causing it to reverse directions a second time. This is also caused by a TOF measurement error.

<center><a href="https://www.youtube.com/watch?v=vj8wEg6iAl8" title="Scope"><img src="/projects/bloop2.PNG" alt="Scope" width="400" height="300" /></a></center>

<center><a href="https://youtu.be/yIC4FqVCr-w" title="Scope"><img src="/projects/bloop1.PNG" alt="Scope" width="400" height="300" /></a></center>

Overall, this lab was very fun to work on. It was cool to see the robot autonomously execute a stunt. I think that there are a lot of possibilities for more complex stunts that could be executed with different types of PID control and specific motor controls, and I may experiment with this if I have time in future weeks. Going forward, I definitely need to work on making my robot less prone to breaking. This will likely involved resoldering some of my sensors and creating a more robust wiring network.

Acknowledgements
======
I would like to thank Rachel Arena for letting me use her Artemis battery when mine broke. 