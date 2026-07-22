# Seed Dropper

This project creates a drone mount for the S500 Frame, that can be used to drop seeds enclosed within sperical shells

# Components

- 3D printed mount and seeds(max 6)

- SG-90 servo

- S500 frame drone that is setup

- Flysky i6S transmitter

- Any flysky compatible receiver, connected to a pixhawk

- Pixhawk

# Setup

The mount is easy to setup, by just inserting the rods of the drone into the mount

## Setting up the controls

The SG-90 Servo has a 180 degree of possible rotation. By default the servo is positioned at 0 degree. Therefore, the servo can rotate 90 degrees in either side. To control such a movement, there are two possible controls on the transmitter.

- Variable Knob(VRA/VRB)
- 3 way switch

The following procedure uses the variable knob, but the 3 way switch can be easily incorporated

### Transmitter setup

1. Bind the transmitter to the receiver
2. On the transmitter, long press the lock icon for 2 seconds, the lock goes into unlocked state
3. At the top, there will be 2 options, 'function' and 'sys'. The function settigs shouch be selected
4. Scroll down, till 'Aux Channels'
5. Using the left and right arrows, go to any unoccupied channel(anything above channel 4 is good)
6. Click on the icon in the middle(It should be 'none' for unoccupied channel). Then to select vairable knov, click on 'VRx'.
7. By clicking on the right of the icon, either 'VRA' or 'VRB' can be selected
8. Now the variable knob is set to send values on the channel

### Ardupilot Setup

As we're using a pixhawk with Ardupilot software, the configuration must be done using 'Mission Planner'

1. Download the Mission Planner App on desktop
2. Connect the desktop to the pixhawk using a USB cable
3. Open mission planner, and on the top right, click on 'Connect'
> If its not connecting, try setting connection to auto
4. The on the top left, click on 'Setup'
5. Under 'Mandatory Hardware', select 'Radio Calibration'. Make sure the transmitter is also ON
6. Now, try to move the variable knob set. In mission planner that specified channel bar must also move. If the green bar is moving, transmitter is sending signals

### Servo Setup

The pixhawk has two output rails, titled 'Main Out' and 'Aux Out'. The servo must be connected to the 'Aux Out' as they send PWM in 50 Hz.

1. Select any 'Aux Out' rail, e.g. Aux Out 3. Connect your servo to those pins
2. In Mission Planner, goto 'Setup' > 'Mandatory Hardware' > 'Servo Outputs'
3. Select the position that matches with your selected Aux Out
> Positions 1 to 8 are for Main Out 1 to 8. Position 9 onwards correspond to Aux Out. e.g. Position 11 is Aux Out 3
4. Now to set its input as transmitter channel, drop down 'Function'
5. Select 'RCINx', where x is you channel number. e.g RCIN6 equals channel 6
6. Move the variable knob on transmitter, check if servo rotates
