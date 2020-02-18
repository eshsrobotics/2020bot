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
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.*;

/**
 * This class is designed to take inputs from OI and JoystickInput, and control
 * the driving motors for driving based on those inputs. It will handle all
 * modes of driving, such as CLASSIC and SNAKE.
 *
 * For all arrays of 4 in this class, the indices within the arrays are
 * indicated by FRONT_RIGHT, FRONT_LEFT, BACK_LEFT, and BACK_RIGHT.
 */
public class WheelDriveSubsystem extends SubsystemBase {

    /**
     * At any given time, the drive can be controlled in one of two ways, carefully
     * chosen at the beginning of the season:
     * 
     * - Crab mode, which is the equivalent of a permanet "strafing" mode, and -
     * Snake mode, which turns the chassis in a manner not dissimilar from a snake
     * with four wheels.
     */
    public enum DriveMode {
        CRAB_MODE, SNAKE_MODE
    }

    /**
     * What's our current mode right now?
     */
    private DriveMode driveMode;

    /**
     * This array contains four elements, one for each wheel. It maintains the last
     * values passed into setGoalAngles().
     *
     * These are directional angles, with 0 radians indicating straight forward and
     * positive angles turning counterclockwise.
     */
    private double[] goalThetas;

    /**
     * This array contains four elements, one for each wheel. It stores the encoder
     * values we learned from calibrate().
     */
    private double[] initialEncoderValues;

    /**
     * This array contains four PWM-driven SpeedControllers, one for each wheel.
     *
     * We use this abstract class on purpose: any motor controllers than can be
     * driven by PWM signals can drive the wheels forward, not just the Spark MAX.
     * Normal Sparks and CIM motors will work just as well!
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
     * PID constants. Each of these appears on the SmartDashboard. We're going to
     * have to experiment with these.
     *
     * See
     * https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/smartdashboard/index.html
     * for more information.
     */
    double kP = 1, kI = 1e-4, kD = 1, kIz = 0, kFF = 0, kMaxOutput = 1, kMinOutput = -1;

    public WheelDriveSubsystem() {


        // By default, we're in crab mode.
        driveMode = DriveMode.CRAB_MODE;

        this.goalThetas = new double[4];
        this.initialEncoderValues = new double[4];

        // Fill the speedMotors list with 4 nulls and then overwrite them.
        //
        // By doing things this way, we make this code immune to changes in
        // the values of the indexing constants (FRONT_LEFT, FRONT_RIGHT, and
        // so on.)
        this.speedMotors = new ArrayList<PWMSpeedController>();
        Collections.addAll(this.speedMotors, new PWMSpeedController[] { null, null, null, null });
        this.speedMotors.set(FRONT_LEFT, new PWMSparkMax(FRONT_LEFT_DRIVE_MOTOR_PORT));
        this.speedMotors.set(FRONT_RIGHT, new PWMSparkMax(FRONT_RIGHT_DRIVE_MOTOR_PORT));
        this.speedMotors.set(BACK_LEFT, new PWMSparkMax(BACK_LEFT_DRIVE_MOTOR_PORT));
        this.speedMotors.set(BACK_RIGHT, new PWMSparkMax(BACK_RIGHT_DRIVE_MOTOR_PORT));

        // Fill the pivotMotors list with 4 nulls and then overwrite them.
        //
        // For _ease of brain_, the IDs for the CANSparkMax controllers are
        // the same as the index constants.
        this.pivotMotors = new ArrayList<CANSparkMax>();
        Collections.addAll(this.pivotMotors, new CANSparkMax[] { null, null, null, null });
        this.pivotMotors.set(FRONT_LEFT, new CANSparkMax(FRONT_LEFT_TURN_MOTOR_PORT, MotorType.kBrushless));
        this.pivotMotors.set(FRONT_RIGHT, new CANSparkMax(FRONT_RIGHT_TURN_MOTOR_PORT, MotorType.kBrushless));
        this.pivotMotors.set(BACK_LEFT, new CANSparkMax(BACK_LEFT_TURN_MOTOR_PORT, MotorType.kBrushless));
        this.pivotMotors.set(BACK_RIGHT, new CANSparkMax(BACK_RIGHT_TURN_MOTOR_PORT, MotorType.kBrushless));

        this.pivotMotors.forEach(m -> {
            // TODO: Read the required PID constants
            // for the SmartDashBoard. The user may
            // change the values at runtime.

            // Reset to the default state (at least
            // until the next power cycle.)
            m.restoreFactoryDefaults();

            // When we cut power, the motors should
            // stop pivoting immediately; otherwise,
            // the slop will throw us off.
            m.setIdleMode(CANSparkMax.IdleMode.kBrake);

            // setting the conversion factor between rotations of the motor to rotations of
            // the wheel

            m.getEncoder().setPositionConversionFactor(1 / WHEEL_TURN_RATIO);

            CANPIDController pidController = m.getPIDController();
            pidController.setP(kP);
            pidController.setI(kI);
            pidController.setD(kD);
            pidController.setIZone(kIz);
            pidController.setFF(kFF);
            pidController.setOutputRange(kMinOutput, kMaxOutput);

        }); // end (for each pivot motor)

        // Create SmartDashboard variables for the pivot PID constants. These
        // are read-write, so they can be adjusted on the fly from the
        // SmartDashboard itself.
        SmartDashboard.putNumber("Pivot PID: P Gain", kP);
        SmartDashboard.putNumber("Pivot PID: I Gain", kI);
        SmartDashboard.putNumber("Pivot PID: D Gain", kD);
        SmartDashboard.putNumber("Pivot PID: I Zone", kIz);
        SmartDashboard.putNumber("Pivot PID: Feed Forward", kFF);
        SmartDashboard.putNumber("Pivot PID: Max Output", kMaxOutput);
        SmartDashboard.putNumber("Pivot PID: Min Output", kMinOutput);
        SmartDashboard.putNumber("Set Rotations", 0);
    }

    /**
     * Obtains the current drive mode for the benefit of commands that need to know
     * this.
     * 
     * @return The current drive mode.
     */
    public DriveMode getDriveMode() {
        return driveMode;
    }

    /**
     * Changes the current drive mode to the given one.
     * 
     * @param driveMode The new mode.
     */
    public void setDriveMode(DriveMode driveMode) {
        this.driveMode = driveMode;
    }

    /**
     * Retrieves the pivot motors directly for debugging purposes.
     */
    List<CANSparkMax> getPivotMotors() {
        return this.pivotMotors;
    }

    /**
     * Retrieves the speed motors directly for debugging purposes.
     */
    List<PWMSpeedController> getSpeedMotors() {
        return this.speedMotors;
    }

    /**
     * Sets the desired directional angles for the drives. "Directional angles" are
     * relative to a bearing of 0 degrees, which for each wheel of the robot is
     * assumed to point directly forward at the start of robotInit().
     *
     * Ultimately, the goal of the input system is to set the appropriate goal
     * angles for each drive mode, and the goal of the WheelDriveSubsystem itself is
     * to move the wheels until they align with these goals.
     *
     * All angles are in radians.
     *
     * @param thetas An array of exactly 4 desired direction values. The indices are
     *               as specified in {FRONT,BACK}_{LEFT,RIGHT}.
     */
    public void setGoalAngles(double[] thetas) {
        this.goalThetas[FRONT_LEFT] = thetas[FRONT_LEFT];
        this.goalThetas[FRONT_RIGHT] = thetas[FRONT_RIGHT];
        this.goalThetas[BACK_LEFT] = thetas[BACK_LEFT];
        this.goalThetas[BACK_RIGHT] = thetas[BACK_RIGHT];

        SmartDashboard.putNumber("FL goal theta", this.goalThetas[FRONT_LEFT]);
        SmartDashboard.putNumber("FR goal theta", this.goalThetas[FRONT_RIGHT]);
        SmartDashboard.putNumber("BL goal theta", this.goalThetas[BACK_LEFT]);
        SmartDashboard.putNumber("BR goal theta", this.goalThetas[BACK_RIGHT]);
    }

    /**
     * Measures the four existing encoder values for the four wheels and presumes
     * that they are all pointing directly to the front.
     *
     * Thereafter, all pivoting angles will be relative to these initial
     * measurements.
     *
     * It is recommended to call calibrate() once at the very start of the match,
     * during robotInit().
     */
    public void calibrate() {
        for (int index = 0; index < this.pivotMotors.size(); index++) {
            var m = this.pivotMotors.get(index);
            double rotations = m.getEncoder().getPosition();
            this.initialEncoderValues[index] = rotations;
        }

        SmartDashboard.putNumber("FL pivot encoder (initial)", this.initialEncoderValues[FRONT_LEFT]);
        SmartDashboard.putNumber("FR pivot encoder (initial)", this.initialEncoderValues[FRONT_RIGHT]);
        SmartDashboard.putNumber("BL pivot encoder (initial)", this.initialEncoderValues[BACK_LEFT]);
        SmartDashboard.putNumber("BR pivot encoder (initial)", this.initialEncoderValues[BACK_RIGHT]);
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

        // Read the PID values that the user set, dynamically.
        kP = SmartDashboard.getNumber("Pivot PID: P Gain", kP);
        kI = SmartDashboard.getNumber("Pivot PID: I Gain", kI);
        kD = SmartDashboard.getNumber("Pivot PID: D Gain", kD);
        kIz = SmartDashboard.getNumber("Pivot PID: I Zone", kIz);
        kFF = SmartDashboard.getNumber("Pivot PID: Feed Forward", kFF);
        kMaxOutput = SmartDashboard.getNumber("Pivot PID: Max Output", kMaxOutput);
        kMinOutput = SmartDashboard.getNumber("Pivot PID: Min Output", kMinOutput);
        
        /*
         * here we would set all of the wheels to the required angles and speeds based
         * on goalThetas and etc
         */


        for (int i = 0; i< this.pivotMotors.size(); i++) {
            var m = this.pivotMotors.get(i);

            CANPIDController pidController = m.getPIDController();
            
            pidController.setP(kP);
            pidController.setI(kI);
            pidController.setD(kD);
            pidController.setIZone(kIz);
            pidController.setFF(kFF);
            pidController.setOutputRange(kMinOutput, kMaxOutput);

            CANEncoder encoder = m.getEncoder();

            //final double wheelPositionInRotations = encoder.getPosition();
            //final double wheelPositionInRadians = wheelPositionInRotations * 2 * Math.PI;

            final double goalRotations = this.goalThetas[i] / (2 * Math.PI);

            // var errorCode = pidController.setReference(wheelPositionInRotations, ControlType.kPosition);
            var errorCode = pidController.setReference(goalRotations, ControlType.kPosition);

            if (errorCode != CANError.kOk) {
                // Something went wrong.
                System.err.printf("pivot motor #%d: pidController.setReference() returned %d (%s)\n", m.getDeviceId(),
                        errorCode, errorCode.name());
            }
        }
        this.pivotMotors.forEach(m -> {

        }); // end (for each pivot motor

        SmartDashboard.putNumber("FL pivot encoder", this.pivotMotors.get(FRONT_LEFT).getEncoder().getPosition());
        SmartDashboard.putNumber("FR pivot encoder", this.pivotMotors.get(FRONT_RIGHT).getEncoder().getPosition());
        SmartDashboard.putNumber("BL pivot encoder", this.pivotMotors.get(BACK_LEFT).getEncoder().getPosition());
        SmartDashboard.putNumber("BR pivot encoder", this.pivotMotors.get(BACK_RIGHT).getEncoder().getPosition());
    }

    /**
     * Given a desired turn radius, which can be positive or negative, this function
     * finds the necessary orientation for all 4 pivot wheels so that the center is
     * turningRadius units away from the turning center, and the inner and outer
     * wheels are aligned with circles that are concentric with the turning center.
     *
     * Note that when turningRadius == 0, this degenerates into an in-place rotation
     * (which is supported behavior.)
     *
     * The values returned here can be passed directly into setGoalAngles().
     *
     * @param turningRadius The turning radius, in the same units as
     *                      WHEEL_DRIVE_VERTICAL_WHEEL_TO_CENTER_DISTANCE and
     *                      WHEEL_DRIVE_HORIZONTAL_WHEEL_TO_CENTER_DISTANCE (that
     *                      is, meters.)
     * @return An array of four directional angles. Directional angles are relative
     *         to the starting position of the wheels (see calibrate()). Clockwise
     *         directional angles -- right turns -- are positive, and
     *         counterclockwise directional angles -- left turns -- are negative.
     *
     *         All angles are given in radians.
     */
    public static double[] snakeDriveGetAngle(double turningRadius) {
        double angles[] = new double[4];

        double innerTheta = Math.atan(WHEEL_DRIVE_VERTICAL_WHEEL_TO_CENTER_DISTANCE
                / (turningRadius - WHEEL_DRIVE_HORIZONTAL_WHEEL_TO_CENTER_DISTANCE));
        double outerTheta = Math.atan(WHEEL_DRIVE_VERTICAL_WHEEL_TO_CENTER_DISTANCE
                / (turningRadius + WHEEL_DRIVE_HORIZONTAL_WHEEL_TO_CENTER_DISTANCE));

        angles[FRONT_LEFT] = innerTheta;
        angles[BACK_LEFT] = -innerTheta;
        angles[FRONT_RIGHT] = outerTheta;
        angles[BACK_RIGHT] = -outerTheta;

        return angles;
    }

    /**
     * 
     * @param vector2 A vector based on axis input
     * @return An array of four directional angles; see snakeDriveGetAngle() for
     *         more information.
     */
    public double[] crabDriveGetAngle(Vector2d joystickVector) {
        double angles[] = new double[4];

        // The reference vector points straight forward.
        // Its length is 1.
        final Vector2d referenceVector = new Vector2d(0, -1);

        // The joystick vector comes from our args.
        // We need to normalize it so its value is 1.
        final double joystickVectorLength = joystickVector.magnitude();
        if (joystickVectorLength < EPSILON) {
            // The joystick has barely moved. There's nothing to rotate toward.
            angles[FRONT_LEFT] = 0;
            angles[BACK_LEFT] = 0;
            angles[BACK_RIGHT] = 0;
            angles[FRONT_RIGHT] = 0;
            return angles;
        }

        final Vector2d normalizedVector = new Vector2d(joystickVector.x / joystickVectorLength,
                joystickVector.y / joystickVectorLength);
        final double dotProduct = normalizedVector.dot(referenceVector);
        double theta = Math.acos(dotProduct);
        SmartDashboard.putNumber("Atan2 theta", 
          Math.atan2(normalizedVector.y, normalizedVector.x) + Math.PI);
        theta = Math.atan2(normalizedVector.y, normalizedVector.x) + Math.PI;

        // TODO: If the difference between the current angle and the new angle is 2*PI or more,
        // then set the current angle to the difference minus 2*PI.
        //
        // - The current angle must come from this.pivotMotors.get(i).getEncoder().getPosition().
        // - The new angle is theta, and I guess it will come from atan2() rather than the dot product.

        angles[FRONT_LEFT] = theta;
        angles[BACK_LEFT] = theta;
        angles[BACK_RIGHT] = theta;
        angles[FRONT_RIGHT] = theta;
        return angles;
    }
}
