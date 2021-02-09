# Control scheme

The ESHS 2020 FRC robot uses a _swerve drive_.  Like a shopping cart, all four
wheels on the drive can be rotated and driven independently.

## Proposal
### Driving
There are two modes that can be used to control the robot: **crab mode** and
**snake mode**.

Joysticks are currently only supported on port 0; additional joysticks on
higher ports will be ignored.

* **Crab mode**
  - **Joysticks**
    1. [x] One joystick controls *strafing*.  In the XBox controller, this is
       the left joystick.
       - [x] The magnitude of the joystick's vector controls speed.
       - [x] The orientation of the joystick's vector controls the pivot/swerve angle.
    2. [ ] One joystick controls *in-place rotation*.  In the XBox controller,
       this is the right joystick's left/right channel; with a single big
       joystick, this is controlled by twisting.
       * [ ] The magnitude of the left/right channel or twist determines the turning speed.
       * [ ] Turning requires the wheels to align so they are tangent to the robot's circle:
         <img src="./snake-rotation.png" style="width: 300px;" />
	3. [x] To handle *sneaking* (driving and turning at half-speed), we utilize a button
	   near or on the strafing joystick.  For the single joystick, this is button 2 (the
	   big thumb-accessible button on the left side of the trigger.)  In the XBox
	   controller, this is accomplished by pushing the left joystick down.
  - **Keyboard**
    1. [x] `WASD` keys control *strafing.*
       - Note that this means that we can only strafe in 8 directions using the
         keyboard (north = `W`, northeast = `W + D`, east = `D`, southeast = 
		 `D + S`, and so on.)  A driver can make up for this by tapping one
		 direction key while holding the other key down.
    2. [ ] Left and right arrow keys control *rotation*.
	3. [ ] The `Shift` key controls *sneaking*.
* **Snake mode**
  - **Joysticks**
    A single joystick controls all aspects of driving.
    * [ ] The magnitude of the up/down channel controls driving speed, forward
      or reverse.
    * [ ] The magnitude of the left/right channel controls the turn radius for
      swerving; a positive radius swerves right, and a negative radius swerves
      left.
  - **Keyboard**
    * [ ] The `W` key drives forward.
    * [ ] The `A` key swerves left (negative radius.)
    * [ ] The `S` key drives backward.
    * [ ] The `D` key swerves left (positive radius.)

### Other buttons
    _(Incomplete)_

## Discussion

* There are four swerve drive schemes discussed in FRC Team 1640's [excellent resource](https://team1640.com/wiki/images/8/85/Pivot-Wheel_Drive.pdf):
    1. **Crab mode**: All 4 wheels pivot in unison and drive at the same speed.
        * _Advantages_:
            - Simple to program
            - Analogous to the "strafing joystick" the ESHS VEX team uses for Mecanum drives
        * _Disadvantages_:
            - Purported orientation drift (may be possible to mitigate with a gyro)
    2. **Snake mode**: Robot has a turning center; inner and outer wheels align to be tangent to circles sharing that center
        * _Advantages_:
            - Robot moves with a natural gait, weaving around obstacles like a snake
        * _Disadvantages_:
            - Tough to program.  How are the inner and outer wheel tangents calculated?
    3. **Automobile mode**: The front wheels pivot as in snake mode, while the rear wheels remain fixed
        * _Advantages_:
            - Slightly easier to program than snake mode
        * _Disadvantages_:
            - Only slightly easier to program than snake mode.  The front wheels should _not_ be parallel when turning.
            - Not as easy to maneuver
    4. **Tank mode**: Turn by moving wheels on left/right sides in opposite directions
        * _Advantages_: Very easy to program
        * _Disadvantages_: With a 4-wheel drive, the longer the robot is lengthwise (**y-bias**), the more friction and wheel damage a tank drive turn will generate.

            Things slightly improve when the tank driving happens on the shortest edge, but for our robot, that would mean being oriented sideways.

* We also discussed two additional schemes:
    1. **Sidefacing tank mode**: Uses a button to change tank scheme from lengthwise (**y-bias**) to widthwise (**x-bias**).
        * _Advantages_:
            - The only realistic way to do tank mode on a rectangular bot is along the short side
        * _Disadvantages_:
            - May be awkward to drive with.  Which direction is currently the front?
            - X-biased tank driving may reduce friction, but it won't eliminate it completely
    2. **Rook style**: Robot travels lengthwise and width-wise straight, like a chess rook, but rotates by spinning in place.
        * _Advantages_:
            - Combines the best of crab mode (strafing without changing orientation) with the best of snake mode (turning with a radius of 0)
        * _Disadvantages_:
            - Robot must take time to pivot in place, and then more time to return to normal driving

* So far, **rook style** seems to offer the best balance.
