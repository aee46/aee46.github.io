+++
title = "Lab 5"
description = "PID Position Control"
weight = 5

[extra]
remote_image = "projects/PIDHeaderImage.PNG"
+++

Prelab
======
To begin, I implemented several bluetooth commands that would assist with debugging and recording data from the robot. This involved creating arrays on the Artemis that store different data readings (TOF distance, motor speed, etc.) with respect to time.

Each iteration of my PID control loop, I call the function record_data() to store current sensor readings into their corresponding arrays.

```Arduino
void record_data(
  int C_TIME,
  float C_DISTANCE,
  float C_L_SPEED,
  float C_R_SPEED,
  float C_KP,
  float C_KI,
  float C_KD
){
  if(INDEX < 2000){
    time_stamps[INDEX] = C_TIME;
    tof1_stamps[INDEX] = C_DISTANCE;
    left_motor_speeds[INDEX] = C_L_SPEED;
    right_motor_speeds[INDEX] = C_R_SPEED;
    KP_ARRAY[INDEX] = C_KP;
    KI_ARRAY[INDEX] = C_KI;
    KD_ARRAY[INDEX] = C_KD;
    INDEX += 1;
  }
}
```

When the PID control loop has finished executing after ten seconds, I can send this data to my laptop using the following bluetooth command:

```Arduino
case SEND_PID_DATA:{
    Serial.println("Sending Data!");
    for(int i = 0; i < TIME_ARRAY_SIZE; i++){
        tx_estring_value.clear();
        tx_estring_value.append(time_stamps[i]);
        tx_estring_value.append("|");
        tx_estring_value.append(tof1_stamps[i]);
        tx_estring_value.append("|");
        tx_estring_value.append(left_motor_speeds[i]);
        tx_estring_value.append("|");
        tx_estring_value.append(right_motor_speeds[i]);
        tx_estring_value.append("|");
        tx_estring_value.append(KP_ARRAY[i]);
        tx_estring_value.append("|");
        tx_estring_value.append(KI_ARRAY[i]);
        tx_estring_value.append("|");
        tx_estring_value.append(KD_ARRAY[i]);
        tx_estring_value.append("|");
        tx_estring_value.append("A");                               
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
    }
    INDEX = 0;
    break;
}
```

On the Python side, my notification handler processes the received string and divides the data into discrete lists.

```Python
def notification_handler(uuid, value):
    global i
    raw_value = ble.bytearray_to_string(value)
    if raw_value[-1] == 'A':
        PID_DATA = raw_value.split("|")
        time_list.append(float(PID_DATA[0]))
        tof1_list.append(float(PID_DATA[1]))
        left_motor_list.append(float(PID_DATA[2]))
        right_motor_list.append(float(PID_DATA[3]))
        kp_list.append(float(PID_DATA[4]))
        ki_list.append(float(PID_DATA[5]))
        kd_list.append(float(PID_DATA[6]))
```

Additionally, I implemented several bluetooth commands to remotely control the PID constants, motor speed, and setpoint distance. One example, which sets the PID constants, is shown below:

```Arduino
case SET_PID_CONSTANTS:{
    success = robot_cmd.get_next_value(val_a);
    if (!success)
        return;

    success = robot_cmd.get_next_value(val_b);
    if (!success)
        return;

    success = robot_cmd.get_next_value(val_c);
    if (!success)
        return;
    PIDPOS_KP_CONSTANT = val_a;
    PIDPOS_KI_CONSTANT = val_b;
    PIDPOS_KD_CONSTANT = val_c;
}
```

Time of Flight Sampling Speed
======
Additionally, I investigated the sampling speed of my time of flight (TOF) sensor prior to creating my PID loop. I also changed the operating mode of my sensor from SHORT to LONG so that I could accurately record further distances. While this resulted in a slower sampling speed, this was not very significant when compared to the performance gain of a larger measurement distance. I set the sensor's timing budget to 33 ms and the intermeasurement period to 33 seconds.
To conduct this test, I implemented a bluetooth command that loops for five seconds. Each loop iteration, the process checks if TOF sensor data is available and records this status to an array. When analyzing this data, I produced the following graphs:

<br>
<center><img src="/projects/lab5toffreqCORRECT.PNG" alt="sensors" width="600" height="400"></center>

Each stem indicates when data was ready from the sensor. Taking the mean time between each data point resulted in a value of 32.667 ms, or a frequency of 30.61 Hz.

PID Control - P Controller
======
With these tests and debugging steps out of the way, it was time to begin designing my PID controller. A PID Controlller is programmed with a value known as the setpoint. For this lab, this setpoint represents the distance between the front of the robot and a wall. Each time the PID control loop measures new data, it calculates the difference between the measured data and the setpoint. The controller then takes the value of this error, along with its integral and derivative, multiplies these values by their corresponding gains, and sums the three outputs. The controller sends this value to an acutator, such as the motors on the robot, to act in a direction that will work to reduce the measured error.
<br>
<center><img src="/projects/pidmath.jpg" alt="sensors" width="600" height="400"></center>

To begin, I implemented a controller that only used the proportional term, which is the measured error multiplied by a specific gain (Kp). One must be careful when setting the desired gain for the proportional term, as large gains can cause oscillations in the controller's response, and could potentially create an unstable system. A proportional gain will act directly upon the measured output, but by neglecting the Integral and Derivative terms, this controller will likely have a sizeable steady state error.

With a desired setpoint of 1 ft (304 mm), I used a controller with a gain of KP = 0.03