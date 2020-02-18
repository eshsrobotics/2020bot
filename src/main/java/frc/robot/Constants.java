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
   * When doing floating-point comparisons, values less than this are considered to be
   * zero for all intents and purposes.
   */
  public static final double EPSILON = 0.0001;

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
   * The gearbox for the planetary gear that our swerve controllers' NEO 550 motors
   * are attached to is 10:1.
   * 
   * The gearbox is, in turn, connected to a 12-tooth sprocket, and this is attached by
   * chain to a 26-tooth sprocket that drives the swerve wheel.
   *                                                       _
   * Thus the final gear ratio = (26 / 12) * (10 / 1) = 21.6.
   */
  public static final double WHEEL_TURN_RATIO = (26 / 12) * (10 / 1);

  /**
   * If only one joystick or controller is plugged in, then the joystick port defaults to 0. If multiple, 
   * it then depends on the order it was plugged in. First joystick connected would 0, second one would be 1, etc. 
   */
  public static final int JOYSTICK_PORT = 0;

  /**
   * The left joystick on the controller handles the direction of the movement, all 360 dergrees. 
   * The right joystick on the controller handles the magnitude of the speed of the movement, Y-Value.
   * The X-Value of the right joystick of the controller handles turning in place. 
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
  public static final int FRONT_LEFT_DRIVE_MOTOR_PORT = FRONT_LEFT + 0;

  /**
   * The PWM port for the motor controller which drives the back left wheel.
   */
  public static final int BACK_LEFT_DRIVE_MOTOR_PORT = BACK_LEFT + 0;

  /**
   * The PWM port for the motor controller which drives the back right wheel.
   */
  public static final int BACK_RIGHT_DRIVE_MOTOR_PORT = BACK_RIGHT + 0;

  /**
   * The PWM port for the motor controller which drives the front right wheel.
   */
  public static final int FRONT_RIGHT_DRIVE_MOTOR_PORT = FRONT_RIGHT + 0;

  /////////////////
  // Pivot ports //
  /////////////////

  /**
   * The PWM port for the motor controller which pivots the front left wheel.
   */
  public static final int FRONT_LEFT_TURN_MOTOR_PORT = FRONT_LEFT + 4;

  /**
   * The PWM port for the motor controller which pivots the back left wheel.
   */
  public static final int BACK_LEFT_TURN_MOTOR_PORT = BACK_LEFT + 4;

  /**
   * The PWM port for the motor controller which pivots the back wheel.
   */
  public static final int BACK_RIGHT_TURN_MOTOR_PORT = BACK_RIGHT + 4;

  /**
   * The PWM port for the motor controller which pivots the front right wheel.
   */
  public static final int FRONT_RIGHT_TURN_MOTOR_PORT = FRONT_RIGHT + 4;
}
