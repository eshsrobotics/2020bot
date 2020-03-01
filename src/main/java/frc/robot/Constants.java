/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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
  public static final double joystick_epsilon = .10;
  /**
   * In meters. It is the horizontal component of the distance from the imaginary
   * center (dimensional center, not center of mass) to the center of any of the
   * wheels.
   */
  public static final double WHEEL_DRIVE_HORIZONTAL_WHEEL_TO_CENTER_DISTANCE = 0.5;

  /**
   * In meters. It is the vertical component of the distance from the imaginary
   * center (dimensional center, not center of mass) to the center of any of the
   * wheels.
   */
  public static final double WHEEL_DRIVE_VERTICAL_WHEEL_TO_CENTER_DISTANCE = 1.0;

  /*
   * Ratio for how many revolutions of the motor to a revolution of the wheel
   *
   * The gearbox for the planetary gear that our swerve controllers' NEO 550
   * motors are attached to is 10:1.
   *
   * The gearbox is, in turn, connected to a 12-tooth sprocket, and this is
   * attached by chain to a 26-tooth sprocket that drives the swerve wheel. _ Thus
   * the final gear ratio = (26 / 12) * (10 / 1) = 21.6.
   */
  public static final double WHEEL_TURN_RATIO = (26 / 12) * (10 / 1);

  /**
   * If only one joystick or controller is plugged in, then the joystick port
   * defaults to 0. If multiple, it then depends on the order it was plugged in.
   * First joystick connected would 0, second one would be 1, etc.
   */
  public static final int JOYSTICK_PORT = 0;

  /**
   * The left joystick on the controller handles the direction of the movement,
   * all 360 dergrees. The right joystick on the controller handles the magnitude
   * of the speed of the movement, Y-Value. The X-Value of the right joystick of
   * the controller handles turning in place.
   */
  public static final Hand TURNING_CONTROLLER_JOYSTICK = Hand.kLeft;
  public static final Hand POWER_CONTROLLER_JOYSTICK = Hand.kRight;

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
  ///////////////////////////////
  // Intake and Shooter ports. //
  ///////////////////////////////
  // These take up ports 8 through 12.

  public static final int INTAKE_PORT = 11;
  public static final int LEFT_BELT_PORT = 0;
  public static final int RIGHT_BELT_PORT = 1;
  public static final int BOTTOM_SHOOTER_FLYWHEEL_CAN_ID = 9;
  public static final int TOP_SHOOTER_FLYWHEEL_CAN_ID = 10;
  public static final int RIGHT_SHOOTER_MOTOR_PWM = 2;
  public static final int LEFT_SHOOTER_MOTOR_PWM = 3;

  public static final double INTAKE_SPEED = 1.0;
  public static final double BELT_SPEED = 1.0;

  public static int CONTROLLER_SHOOT_TRIGGER_BUTTON = 8;

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
  public static String[] KNOWN_CONTROLLER_NAMES = { "Logitech Dual Action" };

  //////////////////////////////////////
  // NetworkTables button assignments //
  //////////////////////////////////////
  // Network Button key strings (for NetworkButton constructor):
  // - Alphanumeric : Uppercase of the key (i.e. A, B, C, ... or 0, 1, 2, ...)
  // - F Keys    : F1, F2, ...
  // - Shift     : "Shift"
  // - Control   : "Ctrl"
  // - Alt       : "Alt"
  // - Windows   : "Windows"
  // - Enter     : "Enter"
  // - Escape    : "Esc"
  // - -         : "Minus"
  // - =         : "Equals"
  // - [         : "Open Bracket"
  // - ]         : "Close Bracket"
  // - /         : "Slash"
  // - '         : "Quote"
  // - ;         : "Semicolon"
  // - Caps Lock : "Caps Lock"
  // - `         : "Back Quote"
  //
  // For the mouse right and left buttons, use "Right Mouse" and "Left Mouse"
  public static final String DRIVE_VECTOR_UP_KEY = "W";
  public static final String DRIVE_VECTOR_LEFT_KEY = "A";
  public static final String DRIVE_VECTOR_DOWN_KEY = "S";
  public static final String DRIVE_VECTOR_RIGHT_KEY = "D";
  public static final String DRIVE_AUXILIARY_LEFT_TURN_KEY = "Left";
  public static final String DRIVE_AUXILIARY_RIGHT_TURN_KEY = "Right";
}
