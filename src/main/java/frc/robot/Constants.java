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
     * The front left motor that drives the front left wheel
     */
    public static final int FRONT_LEFT_DRIVE_MOTOR = 0;

    /**
     * The back left motor that drives the back left wheel
     */
    public static final int BACK_LEFT_DRIVE_MOTOR = 1;    
    /**
     * The front right motor that drives the front right wheel
     */
    public static final int FRONT_RIGHT_DRIVE_MOTOR = 2;

    /**
     * The back right motor that drives the back right wheel
     */
    public static final int BACK_RIGHT_DRIVE_MOTOR = 3;
    
    /**
     * The front left motor that turns the front left wheel
     */
    public static final int FRONT_LEFT_TURN_MOTOR = 0;
    /**
     * The back left motor that turns the back left wheel
     */
    public static final int BACK_LEFT_TURN_MOTOR = 1;
    /**
     * The front right motor that turns the front right wheel
     */
    public static final int FRONT_RIGHT_TURN_MOTOR = 2;
    /**
     * The back right motor that turns the back wheel
     */
    public static final int BACK_RIGHT_TURN_MOTOR = 3;

}
