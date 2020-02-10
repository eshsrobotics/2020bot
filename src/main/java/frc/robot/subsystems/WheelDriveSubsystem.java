package frc.robot.subsystems;

/*----------------------------------------------------------------------------*/

/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMax;

import frc.robot.Constants;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.controller.PIDController;

/**
 * This class is designed to take inputs from OI and JoystickInput, and control
 * the driving motors for driving based on those inputs. It will handle all
 * modes of driving, such as CLASSIC and SNAKE.
 * 
 * For all arrays of 4 in this class, the indices within the arrays are
 * indicated by Constants.FRONT_RIGHT, Constants.FRONT_LEFT,
 * Constants.BACK_LEFT, and Constants.BACK_RIGHT.
 */
public class WheelDriveSubsystem extends SubsystemBase {
    public PWMSparkMax angleMotor;
    public PWMSparkMax speedMotor;
    public PIDController pidController;

    /**
     * This array contains four elements, one for each wheel.  It maintains the
     * last values passed into setGoalAngles().
     * 
     * These are directional angles, with 0 radians indicating straight forward and
     * positive angles turning counterclockwise.
     */
    private double[] goalThetas;

    /**
     * This array contains four elements, one for each wheel.  It stores the
     * encoder values we learned from calibrate().
     */
    private double[] initialEncoderValues;

    public WheelDriveSubsystem(int angleMotor, int speedMotor, int encoder) {
        this.angleMotor = new PWMSparkMax(angleMotor);
        this.speedMotor = new PWMSparkMax(speedMotor);
        this.pidController = new PIDController(1, 0, 0);

        this.goalThetas = new double[4];
        this.initialEncoderValues = new double[4];

        pidController.setIntegratorRange(-1.0, 1.0);
        pidController.enableContinuousInput(-1.0, 1.0);
        // pidController.enable();
    }

    private final double MAX_VOLTS = 12;

    public void drive(double Speed, double Angle) {
        speedMotor.set(Speed);

        double setpoint = Angle * (MAX_VOLTS * 0.5) + (MAX_VOLTS * 0.5); // Optimization offset can be calculated here.
        if (setpoint < 0) {
            setpoint = MAX_VOLTS + setpoint;
        }
        if (setpoint > MAX_VOLTS) {
            setpoint = setpoint - MAX_VOLTS;
        }

        pidController.setSetpoint(setpoint);
    }

    /**
     * Sets the desired directional angles for the drives.  "Directional angles" are relative to a bearing of 0
     * degrees, which for each wheel of the robot is assumed to point directly forward at the start of robotInit().
     * 
     * Ultimately, the goal of the input system is to set the appropriate goal angles for each drive mode,
     * and the goal of the WheelDriveSubsystem itself is to move the wheels until they align with these goals.
     * 
     * All angles are in radians.
     * 
     * @param thetas An array of exactly 4 desired direction values.  The 
     *               indices are as specified in 
     *               Constants.{FRONT,BACK}_{LEFT,RIGHT}.
     */
    public void setGoalAngles(double[] thetas) {
        this.goalThetas[Constants.FRONT_LEFT] = thetas[Constants.FRONT_LEFT];
        this.goalThetas[Constants.FRONT_RIGHT] = thetas[Constants.FRONT_RIGHT];
        this.goalThetas[Constants.BACK_LEFT] = thetas[Constants.BACK_LEFT];
        this.goalThetas[Constants.BACK_RIGHT] = thetas[Constants.BACK_RIGHT];
    }

    /**
     * Measures the four existing encoder values for the four wheels and
     * presumes that they are all pointing directly to the front.
     * 
     * Thereafter, all pivoting angles will be relative to these initial measurements.
     * 
     * It is recommended to call calibrate() once at the very start of the match, during robotInit().
     */
    public void calibrate() {

        // TODO: For each CANSparkMax, call getEncoder() to get the encoder, and then call getPosition() to get the encoder
        // positions in units of rotations.  Store these into this.initialEncoderValues[].
    }

    /**
     * exists to constantly run with the scheduler
     */
    @Override
    public void periodic() {
        /*
        here we would set all of the wheels to the required angles and speeds based on goalThetas and etc
        */
    }

    /**
     * 
     * @param turningRadius which is in meters.   
     * @return An array of four directional angles.
     * 
     *         All angles are given in radians, in respect to the starting position of the wheels. The starting angle is pi/2 radians. 
     *         To find the absolutel angle of the wheel, it would be pi/2 + the directional angle. 
     *         The convention is that counterclockwise pivot angles are positive, while clockwise pivot angles are negative.
     */
    public static double[] snakeDriveGetAngle(double turningRadius) {
        double angles[] = new double[4];

        double innerTheta = Math.atan(Constants.WHEEL_DRIVE_VERTICAL_WHEEL_TO_CENTER_DISTANCE / (turningRadius - Constants.WHEEL_DRIVE_HORIZONTAL_WHEEL_TO_CENTER_DISTANCE));
        double outerTheta = Math.atan(Constants.WHEEL_DRIVE_VERTICAL_WHEEL_TO_CENTER_DISTANCE / (turningRadius + Constants.WHEEL_DRIVE_HORIZONTAL_WHEEL_TO_CENTER_DISTANCE));

        angles[Constants.FRONT_LEFT] = innerTheta;
        angles[Constants.BACK_LEFT] = -innerTheta;
        angles[Constants.FRONT_RIGHT] = outerTheta;
        angles[Constants.BACK_RIGHT] = -outerTheta;

        return angles;
    }

}
