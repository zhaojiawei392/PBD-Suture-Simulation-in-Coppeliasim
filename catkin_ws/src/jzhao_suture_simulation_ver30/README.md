# Forceps operation check
1. The control box should be COMPLETELY turned off after use.
2. The control box should be COMPLETELY turned off before restarting the computer.

Failing to do so can cause unexpected motion which might damage the forceps modules and the forceps tips.

## Preliminaries

Check if the configuration file matches your system information.

In `cfg/launch`
1. Check if `DI_BOARD` and `AO_BOARD` match the ID of your board.
2. Check if `"ENC":` for each degree-of-freedom of each forceps match the ID of your board.

## Check cabling
1. Be sure that the control box is COMPLETELY turned off.
2. Be sure that all cables are connected. The cables between the control box and the PC, and the cables between the control box and the forceps modules.

## Run the motor control node
Open one terminal window and run

`roslaunch jaxa_mhi_control forceps.launch --screen`

You should have the following output. If there are any errors, fix the configuration and check your driver.

<p align="center">
  <img src="images/forceps_operation_check/step1.png" height="400px" width="400px">
</p>

## Turn on the control box

1. Turn on the control box power button.
2. Turn on, one by one, the switches for each degree-of-freedom of the forceps.

Check that there is no motion. 
If there is ANY motion, turn off the control box immediately. Restart the guide from step one because some mistake was done.

## Run the forceps operation check node
Open a new terminal window and run

`rosrun jaxa_mhi_control forceps_operation_check_node`

The following window should show up.
Note: the current value of "position" is meaningless.

<p align="center">
  <img src="images/forceps_operation_check/step3.png" height="200px" width="600px">
</p>

## Home each DOF

1. Click `Enable` on top of `Home` to set the mode to homing mode.
2. Click `DON'T HOME` for each DOF, one at a time. The button will change from `DON'T HOME` to `HOME` and the homing process will begin.
3. Verify that each motor moves and stops. When it stops, the `Position` for that motor should be `0`.
4. Verify that the `Sensor` value is `0` for a rotation motor and `1` for a slider motor.

The program should look like this:
<p align="center">
  <img src="images/forceps_operation_check/step4.png" height="200px" width="600px">
</p>

If something unexpected happens, restart the guide or fix the device.

Verify that each motor's home position matches the expected home position. If the home position is different from expected, fix the device.

## Attach the tips

Attach the tip for the forceps. If the forceps does not fit, fix the device.

## Evaluate the position control (Rotation DOF)

For the rotation DOF of each forceps:


### Move to 180

1. Click `Enable` on top of `Target Position` to set the mode to position control mode.
2. Click `+deg` for that DOF one time. Wait one second. Verify that `Position` is close to `Target Position`.
3. Do that 18 times until `Target Position` and `Position` reach `180`.

If at some point the `Position` stops changing and is different from `Target Position`, there is too much friction in the device. Fix the device.

The program should look like this:
<p align="center">
  <img src="images/forceps_operation_check/step5.png" height="200px" width="600px">
</p>

### Move to -180

4. Click the `-deg` for that DOF one time. Wait one second. Verify that `Position` is close to `Target Position`.
5. Do that 36 times until `Target Position` and `Position` reach `-180`.

If at some point the `Position` stops changing and is different from `Target Position`, there is too much friction in the device. Fix the device.

The program should look like this:
<p align="center">
  <img src="images/forceps_operation_check/step6.png" height="200px" width="600px">
</p>

### Move to 0

6. Click the `+deg` for that DOF one time. Wait one second. Verify that `Position` is close to `Target Position`.
7. Do that 18 times until `Target Position` and `Position` reach `0`.

If at some point the `Position` stops changing and is different from `Target Position`, there is too much friction in the device. Fix the device.

The program should look like this:
<p align="center">
  <img src="images/forceps_operation_check/step7.png" height="200px" width="600px">
</p>

If there are no problems, this DOF is most likely working correctly.

## Evaluate the position control (Slider DOF)

For the slider DOF of each forceps.

### Move to 3000
1. Click and hold `+deg` for that DOF until it reaches `3000`.

If at some point the `Position` stops changing and is different from `Target Position`, there is too much friction in the device or some part of the mechanism is blocked. Fix the device.

The program should look like this:
<p align="center">
  <img src="images/forceps_operation_check/step8.png" height="200px" width="600px">
</p>

### Move to 0
2. Clock and hold `-deg` for that DOF until it reaches `0`.

If at some point the `Position` stops changing and is different from `Target Position`, there is too much friction in the device or some part of the mechanism is blocked. Fix the device.

The program should look like this:
<p align="center">
  <img src="images/forceps_operation_check/step9.png" height="200px" width="600px">
</p>

## Turning off

1. Turn off the main power switch of the forceps control box.
2. Turn off the power switch of each DOF.
3. Close `forceps_operation_check_node` by pressing `CTRL+C` on that terminal window.
4. Close `forceps.launch` by pressing `CTRL+C` on that terminal window.
