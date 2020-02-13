package frc.robot.subsystems;

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import java.lang.Math;
import java.util.List;
import java.util.Collections;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;


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

    /**
     * This array contains four PWM-driven SpeedControllers, one for each
     * wheel.
     *
     * We use this abstract class on purpose: any motor controllers than can
     * be driven by PWM signals can drive the wheels forward, not just the
     * Spark MAX.  Normal Sparks and CIM motors will work just as well!
     */
    private List<PWMSpeedController> speedMotors;

    /**
     * This array contains four CANSparkMax controllers, one for each wheel.
     *
     * They are documented here:
     * https://www.revrobotics.com/content/sw/max/sw-docs/java/com/revrobotics/CANSparkMax.html
     *
     * These are the only speed controllers that are condoned for use with the
     * brushless NEO motors for FRC, so no, you don't get a choice of
     * SpeedController here.
     */
    private List<CANSparkMax> pivotMotors;

    /**
     * PID constants.  Each of these appears on the SmartDashboard.
     * We're going to have to experiment with these.
     *
     * See
     * https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/smartdashboard/index.html
     * for more information.
     */
    double kP = 0.1, kI = 1e-4, kD = 1, kIz = 0, kFF = 0, kMaxOutput = 1, kMinOutput = -1;

    public WheelDriveSubsystem() {

        this.goalThetas = new double[4];
        this.initialEncoderValues = new double[4];

        // Fill the speedMotors list with 4 nulls and then overwrite them.
        //
        // By doing things this way, we make this code immune to changes in
        // the values of the indexing constants (Constants.FRONT_LEFT, Constants.FRONT_RIGHT, and
        // so on.)
        this.speedMotors = new ArrayList<PWMSpeedController>();
        Collections.addAll(this.speedMotors, new PWMSpeedController[] { null, null, null, null });
        this.speedMotors.set(Constants.FRONT_LEFT,  new PWMSparkMax(FRONT_LEFT_DRIVE_MOTOR_PORT));
        this.speedMotors.set(Constants.FRONT_RIGHT, new PWMSparkMax(FRONT_RIGHT_DRIVE_MOTOR_PORT));
        this.speedMotors.set(Constants.BACK_LEFT,   new PWMSparkMax(BACK_LEFT_DRIVE_MOTOR_PORT));
        this.speedMotors.set(Constants.BACK_RIGHT,  new PWMSparkMax(BACK_RIGHT_DRIVE_MOTOR_PORT));

        // Fill the pivotMotors list with 4 nulls and then overwrite them.
        //
        // For _ease of brain_, the IDs for the CANSparkMax controllers are
        // the same as the index constants.
        this.pivotMotors = new ArrayList<CANSparkMax>();
        Collections.addAll(this.pivotMotors, new CANSparkMax[] { null, null, null, null });
        this.pivotMotors.set(Constants.FRONT_LEFT,  new CANSparkMax(FRONT_LEFT_TURN_MOTOR_PORT,   MotorType.kBrushless));
        this.pivotMotors.set(Constants.FRONT_RIGHT, new CANSparkMax(FRONT_RIGHT_TURN_MOTOR_PORT,  MotorType.kBrushless));
        this.pivotMotors.set(Constants.BACK_LEFT,   new CANSparkMax(BACK_LEFT_TURN_MOTOR_PORT,    MotorType.kBrushless));
        this.pivotMotors.set(Constants.BACK_RIGHT,  new CANSparkMax(BACK_RIGHT_TURN_MOTOR_PORT,   MotorType.kBrushless));

        this.pivotMotors.forEach(m ->
                                 {
                                     // TODO: Read the required PID constants
                                     // for the SmartDashBoard.  The user may
                                     // change the values at runtime.

                                     // Reset to the default state (at least
                                     // until the next power cycle.)
                                     m.restoreFactoryDefaults();

                                     // When we cut power, the motors should
                                     // stop pivoting immediately; otherwise,
                                     // the slop will throw us off.
                                     m.setIdleMode(CANSparkMax.IdleMode.kBrake);

                                     // TODO: call
                                     // m.getEncoder().setPositionConversionFactor()
                                     // with a constant that converts motor
                                     // rotations into pivot wheel rotations,
                                     // then multiple that by 2π to use radians.

                                     CANPIDController pidController = m.getPIDController();
                                     pidController.setP(kP);
                                     pidController.setI(kI);
                                     pidController.setD(kD);
                                     pidController.setIZone(kIz);
                                     pidController.setFF(kFF);
                                     pidController.setOutputRange(kMinOutput, kMaxOutput);

                                 }); // end (for each pivot motor)

        // Create SmartDashboard variables for the pivot PID constants.  These
        // are read-write, so they can be adjusted on the fly from the
        // SmartDashboard itself.
        SmartDashboard.putNumber("Pivot PID: P Gain", kP);
        SmartDashboard.putNumber("Pivot PID: I Gain", kI);
        SmartDashboard.putNumber("Pivot PID: D Gain", kD);
        SmartDashboard.putNumber("Pivot PID: I Zone", kIz);
        SmartDashboard.putNumber("Pivot PID: Feed Forward", kFF);
        SmartDashboard.putNumber("Pivot PID: Max Output", kMaxOutput);
        SmartDashboard.putNumber("Pivot PID: Min Output", kMinOutput);
    }

    /**
     * Retrieves the pivot motors directly for debugging purposes.
     */
    List<CANSparkMax> getPivotMotors() { return this.pivotMotors; }

    /**
     * Retrieves the speed motors directly for debugging purposes.
     */
    List<PWMSpeedController> getSpeedMotors() { return this.speedMotors; }

    /**
     * Sets the desired directional angles for the drives.  "Directional
     * angles" are relative to a bearing of 0 degrees, which for each wheel of
     * the robot is assumed to point directly forward at the start of
     * robotInit().
     *
     * Ultimately, the goal of the input system is to set the appropriate goal
     * angles for each drive mode, and the goal of the WheelDriveSubsystem
     * itself is to move the wheels until they align with these goals.
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

        SmartDashboard.putNumber("FL goal θ", this.goalThetas[Constants.FRONT_LEFT]);
        SmartDashboard.putNumber("FR goal θ", this.goalThetas[Constants.FRONT_RIGHT]);
        SmartDashboard.putNumber("BL goal θ", this.goalThetas[Constants.BACK_LEFT]);
        SmartDashboard.putNumber("BR goal θ", this.goalThetas[Constants.BACK_RIGHT]);
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
        this.pivotMotors.forEach(m ->
                                 {
                                     int index = m.getDeviceId();
                                     double rotations = m.getEncoder().getPosition();
                                     this.initialEncoderValues[index] = rotations;
                                 });

        SmartDashboard.putNumber("FL pivot encoder (initial)", this.initialEncoderValues[Constants.FRONT_LEFT]);
        SmartDashboard.putNumber("FR pivot encoder (initial)", this.initialEncoderValues[Constants.FRONT_RIGHT]);
        SmartDashboard.putNumber("BL pivot encoder (initial)", this.initialEncoderValues[Constants.BACK_LEFT]);
        SmartDashboard.putNumber("BR pivot encoder (initial)", this.initialEncoderValues[Constants.BACK_RIGHT]);
    }

    /**
     * Adjusts our pivoting angles so they get closer to the desired pivoting
     * angles.
     *
     * This function is run continuously, thousands of times a second, by the
     * scheduler.
     */
    @Override
    public void periodic() {
        /*
        here we would set all of the wheels to the required angles and speeds based on goalThetas and etc
        */

        // This will *NOT* work correctly until we have called
        // setPositionConversionFactor() on each of the pivotMotor encoders!
        // The rotation values we get out of the encoders are unusable until
        // then.
        this.pivotMotors.forEach(m ->
                                 {
                                     CANPIDController pidController = m.getPIDController();
                                     CANEncoder encoder = m.getEncoder();

                                     final double wheelPositionInRotations = encoder.getPosition();
                                     final double wheelPositionInRadians = wheelPositionInRotations * 2 * Math.PI;

                                     var errorCode = pidController.setReference(wheelPositionInRadians, ControlType.kPosition);

                                     if (errorCode != CANError.kOk) {
                                         // Something went wrong.
                                         System.err.printf("pivot motor #%d: pidController.setReference() returned %d (%s)\n",
                                                           m.getDeviceId(),
                                                           errorCode,
                                                           errorCode.name());
                                     }
                                 }); // end (for each pivot motor

        SmartDashboard.putNumber("FL pivot encoder", this.pivotMotors.get(Constants.FRONT_LEFT).getEncoder().getPosition());
        SmartDashboard.putNumber("FR pivot encoder", this.pivotMotors.get(Constants.FRONT_RIGHT).getEncoder().getPosition());
        SmartDashboard.putNumber("BL pivot encoder", this.pivotMotors.get(Constants.BACK_LEFT).getEncoder().getPosition());
        SmartDashboard.putNumber("BR pivot encoder", this.pivotMotors.get(Constants.BACK_RIGHT).getEncoder().getPosition());
    }

    /**
     * Given a desired turn radius, which can be positive or negative, this
     * function finds the necessary orientation for all 4 pivot wheels so that
     * the center is turningRadius units away from the turning center, and
     * the inner and outer wheels are aligned with circles that are concentric
     * with the turning center.
     *
     * Note that when turningRadius == 0, this degenerates into an in-place
     * rotation (which is supported behavior.)
     *
     * The values returned here can be passed directly into setGoalAngles().
     *
     * @param turningRadius The turning radius, in the same units as
     *                      Constants.WHEEL_DRIVE_VERTICAL_WHEEL_TO_CENTER_DISTANCE
     *                      and
     *                      Constants.WHEEL_DRIVE_HORIZONTAL_WHEEL_TO_CENTER_DISTANCE
     *                      (that is, meters.)
     * @return An array of four directional angles.  Directional angles are
     *         relative to the starting position of the wheels (see
     *         calibrate()).  Clockwise directional angles -- right turns --
     *         are positive, and counterclockwise directional angles -- left
     *         turns -- are negative.
     *
     *         All angles are given in radians.
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
