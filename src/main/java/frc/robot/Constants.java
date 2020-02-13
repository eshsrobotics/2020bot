/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 *
 */
public final class Constants {
    /**
     * In meters. It is the horizontal component of the distance from the imaginary center
     * (dimensional center, not center of mass) to the center of any of the wheels.
     */
    public static final double WHEEL_DRIVE_HORIZONTAL_WHEEL_TO_CENTER_DISTANCE = 0.5;

    /**
     * In meters. It is the vertical component of the distance from the imaginary center
     * (dimensional center, not center of mass) to the center of any of the wheels.
     */
    public static final double WHEEL_DRIVE_VERTICAL_WHEEL_TO_CENTER_DISTANCE = 1.0;

    /**
     * Our swerve drive makes us use arrays of four quite often: four pivot
     * motor controllers, four driving motor controllers, and two sets of four
     * PWM ports for these.  It helps to have a consistent order!
     *
     * These constants determine that order.  You can think of them as being
     * ordered "counterclockwise around the car", starting with the "driver's
     * seat."
     */
    public static final int FRONT_LEFT = 0;
    public static final int BACK_LEFT = 1;
    public static final int BACK_RIGHT = 2;
    public static final int FRONT_RIGHT = 3;

    ///////////////////
    // Driving ports //
    ///////////////////

    /**
     * The PWM port for the motor controller which drives the front left
     * wheel.
     */
    public static final int FRONT_LEFT_DRIVE_MOTOR_PORT = FRONT_LEFT + 0;

    /**
     * The PWM port for the motor controller which drives the back left
     * wheel.
     */
    public static final int BACK_LEFT_DRIVE_MOTOR_PORT = BACK_LEFT + 0;

    /**
     * The PWM port for the motor controller which drives the back right
     * wheel.
     */
    public static final int BACK_RIGHT_DRIVE_MOTOR_PORT = BACK_RIGHT + 0;

    /**
     * The PWM port for the motor controller which drives the front right
     * wheel.
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
