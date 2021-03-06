/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.nio.file.Path;

import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 *
 * <p>
 * QUICK REFERENCE: FL: 1, 5. BL: 2, 6. BR: 3, 7. FR: 4, 8 INTAKE: 4. BELTS:
 * 0,1. CLIMB: 2, 3. SHOOTER: TOP-10, BOT-9
 *
 */
public final class Constants {

  /**
   * When doing floating-point comparisons, values less than this are considered
   * to be zero for all intents and purposes.
   */
  public static final double EPSILON = 0.01;
  /**
   * joystick_epsilon is the controller joystick deadzone variable. This will help
   * with jittery movement while driving.
   */
  public static final double JOYSTICK_EPSILON = .15;
  /**
   * In meters. It is the horizontal component of the distance from the imaginary
   * center (dimensional center, not center of mass) to the center of any of the
   * wheels.
   */
  public static final double WHEEL_DRIVE_HORIZONTAL_WHEEL_TO_CENTER_DISTANCE = 0.435 * 0.5;

  /**
   * In meters. It is the vertical component of the distance from the imaginary
   * center (dimensional center, not center of mass) to the center of any of the
   * wheels.
   */
  public static final double WHEEL_DRIVE_VERTICAL_WHEEL_TO_CENTER_DISTANCE = 0.745 * 0.5;


  /**
   * This iS VERY IMPORTANT (see snake-rotation.png in docs folder for visual representation)
   * 
   * They represent angles that are tangent to a circle, so that when the robot's wheels are set to said angles, 
   * and the drive motors move in the same direction, the robot rotates in place.
   */
  public static final double[] crabRotationThetas = {
      (-Math.PI) + Math.atan(WHEEL_DRIVE_VERTICAL_WHEEL_TO_CENTER_DISTANCE / WHEEL_DRIVE_HORIZONTAL_WHEEL_TO_CENTER_DISTANCE),
      Math.PI - Math.atan(WHEEL_DRIVE_VERTICAL_WHEEL_TO_CENTER_DISTANCE / WHEEL_DRIVE_HORIZONTAL_WHEEL_TO_CENTER_DISTANCE),
      Math.atan(WHEEL_DRIVE_VERTICAL_WHEEL_TO_CENTER_DISTANCE / WHEEL_DRIVE_HORIZONTAL_WHEEL_TO_CENTER_DISTANCE),
      -Math.atan(WHEEL_DRIVE_VERTICAL_WHEEL_TO_CENTER_DISTANCE / WHEEL_DRIVE_HORIZONTAL_WHEEL_TO_CENTER_DISTANCE)
    };

  /**  
   * Is an assumption of the maximum drive train speed in meters per second. 
   * 
   * This is so we can calculate the motor power percentage. This number is guessed from thin air 
   * and should be replaced by actual drive speed train based upon trails. 
   */
  public static final double MAX_ROBOT_SPEED_MPS = 5;

  
  /**
   * Ratio for how many revolutions of the pivot motor correspond to one full
   * rotation of the wheel about the y-axis.
   *
   * The only way to get this value is to dismantle the swerve module and count
   * the teeth. So, we got this value from the tech specs at
   * https://www.swervedrivespecialties.com/products/mk3-swerve-module.
   */
  public static final double WHEEL_TURN_RATIO = 12.8 * (1.0);

  /**
   * If only one joystick or controller is plugged in, then the joystick port
   * defaults to 0. If multiple, it then depends on the order it was plugged in.
   * First joystick connected would 0, second one would be 1, etc.
   */
  public static final int JOYSTICK_PORT = 0;

  /**
   * The left joystick on the controller handles the direction of the movement,
   * all 360 degrees. The right joystick on the controller handles the magnitude
   * of the speed of the movement, Y-Value. The X-Value of the right joystick of
   * the controller handles turning in place.
   */
  public static final Hand TURNING_CONTROLLER_JOYSTICK = Hand.kLeft;
  public static final Hand POWER_CONTROLLER_JOYSTICK = Hand.kRight;

  /**
   * Any time the x or y channel of a joystick is less than this value, we assume the channel is zero and take no action.
   */
  public static final double DEADZONE = 0.25;

  ///////////////////
  // Motor indices //
  ///////////////////

  /**
   * Our swerve drive makes us use arrays of four quite often: four pivot motor
   * controllers, four driving motor controllers, and two sets of four PWM ports
   * for these. It helps to have a consistent order!
   *
   * These constants determine that order. You can think of them as being ordered
   * "counterclockwise around the car", starting with the "driver's seat."
   */
  public static final int FRONT_LEFT = 0;
  public static final int BACK_LEFT = 1;
  public static final int BACK_RIGHT = 2;
  public static final int FRONT_RIGHT = 3;

  ///////////////////
  // Driving ports //
  ///////////////////

  /**
   * The PWM port for the motor controller which drives the front left wheel.
   */
  public static final int FRONT_LEFT_DRIVE_MOTOR_CAN_ID = FRONT_LEFT + 1;// FRONT_LEFT + 0;

  /**
   * The PWM port for the motor controller which drives the back left wheel.
   */
  public static final int BACK_LEFT_DRIVE_MOTOR_CAN_ID = BACK_LEFT + 1;

  /**
   * The PWM port for the motor controller which drives the back right wheel.
   */
  public static final int BACK_RIGHT_DRIVE_MOTOR_CAN_ID = BACK_RIGHT + 1;

  /**
   * The PWM port for the motor controller which drives the front right wheel.
   */
  public static final int FRONT_RIGHT_DRIVE_MOTOR_CAN_ID = FRONT_RIGHT + 1;

  /////////////////
  // Pivot ports //
  /////////////////

  /**
   * The PWM port for the motor controller which pivots the front left wheel.
   */
  public static final int FRONT_LEFT_TURN_MOTOR_CAN_ID = FRONT_LEFT + 5;

  /**
   * The PWM port for the motor controller which pivots the back left wheel.
   */
  public static final int BACK_LEFT_TURN_MOTOR_CAN_ID = BACK_LEFT + 5;

  /**
   * The PWM port for the motor controller which pivots the back wheel.
   */
  public static final int BACK_RIGHT_TURN_MOTOR_CAN_ID = BACK_RIGHT + 5;

  /**
   * The PWM port for the motor controller which pivots the front right wheel.
   */
  public static final int FRONT_RIGHT_TURN_MOTOR_CAN_ID = FRONT_RIGHT + 5;

  /**
   * Easily controlls speed, we dont want 100 % all the time. This value should be
   * between 0.0 and 1.0.
   */
  public static final double DRIVE_SPEED_MULTIPLIER = 0.70;

  public static final double DRIVE_SNEAK_MODIFIER = 0.8;
  ///////////////////////////////
  // Intake and Shooter ports. //
  ///////////////////////////////
  // These take up ports 8 through 12.

  public static final int INTAKE_PORT = 4;
  public static final int LEFT_BELT_PORT = 0;
  public static final int RIGHT_BELT_PORT = 1;
  public static final int BOTTOM_SHOOTER_FLYWHEEL_CAN_ID = 9;
  public static final int TOP_SHOOTER_FLYWHEEL_CAN_ID = 10;
  public static final int RIGHT_CLIMBER_MOTOR_PWM = 2;
  public static final int LEFT_CLIMBER_MOTOR_PWM = 3;

  public static final double INTAKE_SPEED = 1.0;
  public static final double BELT_SPEED = 1.0;

  public static int CONTROLLER_SHOOT_TRIGGER_BUTTON = 8;

  public static int ROTATE_CLOCKWISE_BUTTON = 1;
  public static int ROTATE_COUNTERCLOCKWISE_BUTTON = 2;

  /**
   * You'd think that you could detect an XBox controller easily, perhaps with:
   *
   * DriverStation.getInstance()..getJoystickIsXbox(port) == true
   *
   * Or maybe just with:
   *
   * joystick.getType() == GenericHID.HIDType.kHIDGamePad
   *
   * but you'd be wrong. Both of those return the SAME RESULT for both our XBox
   * controller and our normal gaming joystick. Indeed, the only thing the system
   * _does_ seem to be able to distinguish is the joystick names, which we then
   * have to hard-code (only once, fortunately.)
   */
  public static String[] KNOWN_CONTROLLER_NAMES = { "Logitech Dual Action", "Controller (Xbox One For Windows)" };

  //////////////////////////////////////
  // NetworkTables button assignments //
  //////////////////////////////////////
  // Network Button key strings (for NetworkButton constructor):
  // - Alphanumeric : Uppercase of the key (i.e. A, B, C, ... or 0, 1, 2, ...)
  // - F Keys : F1, F2, ...
  // - Shift : "Shift"
  // - Control : "Ctrl"
  // - Alt : "Alt"
  // - Windows : "Windows"
  // - Enter : "Enter"
  // - Escape : "Esc"
  // - - : "Minus"
  // - = : "Equals"
  // - [ : "Open Bracket"
  // - ] : "Close Bracket"
  // - / : "Slash"
  // - ' : "Quote"
  // - ; : "Semicolon"
  // - Caps Lock : "Caps Lock"
  // - ` : "Back Quote"
  //
  // For the mouse right and left buttons, use "Right Mouse" and "Left Mouse"
  public static final String DRIVE_VECTOR_UP_KEY = "W";
  public static final String DRIVE_VECTOR_LEFT_KEY = "A";
  public static final String DRIVE_VECTOR_DOWN_KEY = "S";
  public static final String DRIVE_VECTOR_RIGHT_KEY = "D";
  public static final String DRIVE_AUXILIARY_LEFT_TURN_KEY = "Left";
  public static final String DRIVE_AUXILIARY_RIGHT_TURN_KEY = "Right";
  public static final String DRIVE_SNEAK_KEY = "Shift";

  /////////////////////////////////
  // Vision solutions constants. //
  /////////////////////////////////

  // Derived from limelight resolution.
  public static final double SCREEN_WIDTH_PIXELS = 960;
  // Derived from horizontal FOV of limelight.
  public static final double FOV_RADIANS = 59.6 * (2 * Math.PI / 360);

  // Height from the ground to the center of the limelight's camera.
  public static final double ROBOT_HEIGHT_METERS = 0.604166666;
  // Derived from vertical FOV of limelight.
  public static final double VERTICAL_FOV_RADIANS = 45.7 * (2 * Math.PI / 360);
  // Derived from limelight resolution.
  public static final double SCREEN_HEIGHT_PIXELS = 720;
  // Assumes that the bottom of the screen is the ground plane,
  // and that the distance from the ground plane to the center of the screen is
  // the robot height.
  public static final double SCREEN_HEIGHT_METERS = ROBOT_HEIGHT_METERS * 2;

  // Derived from meter and pixel values of screen height. This is true only at
  // the edge of the field of view
  public static final double METERS_PER_PIXEL = SCREEN_HEIGHT_METERS / SCREEN_HEIGHT_PIXELS;

  // Essentially the "P" in PID. This used to be .15 radians.
  // Used to tell the robot how far it needs to be from the goal angle to stop
  // rotating.
  //
  // NOTE: This is only used for the non-setReference()-based pivoting method in
  // {@link WheelDriveSubsystem.periodic}, which means it is not currently
  // being used.  It's also not final because it can be modified through the
  // SmartDashboard.
  public static double GOAL_ROTATION_EPSILON_RADIANS = 5.85 * 2 * Math.PI / 360;

  // Whenever our vision solution is this close to the center of the limelight, we
  // assume we've hit our target.
  public static final double DEVIATION_EPSILON_DEGREES = Math.E;

  // Assumes limelight returns a tx between zero and this.
  // It's ok if this isn't exactly accurate.
  public static final double MAX_HORIZONTAL_DEVIATION_DEGREES = 22.0;
 
  //////////////////////////////////////////
  // Autonomous and trajectory constants. //
  //////////////////////////////////////////
  
  /**
   * A list of the possible trajectories that a user can select with the
   * selection keys.
   *
   * Each represents an actual path on the RoboRio, meaning you must first
   * upload the file to the RoboRio (using FileZilla) prior to running it.
   */

  public static final Path[] trajectoryList = {
      Path.of("/home", "admin", "trajectories", "test_trajectory.json"), // "/home/admin/trajectories/test_trajectory.json"
      Path.of("/home", "admin", "trajectories", "test_time.json")        // "/home/admin/trajectories/test_time.json"
  };
}