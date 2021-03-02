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

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.InputSubsystem.*;

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
     * A state variable showing whether the joystick is in the the deadzone for driving
     */
    private boolean manualDriving = true;

    static final double TWO_PI = 2 * Math.PI;

    private boolean crabCenterRotationMode = false;

    private double crabCenterRotationSpeed = 0.0;

    private boolean sneakMode = false;

    private boolean turn_enabled = false;

    private PIDController[] pidControllers;

    private double[] pidErrors;

    private double[] pidIntegrals;

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
     * An array of speeds that the motors will gradually accelerate to.
     */
    private double[] goalSpeeds;

    /**
     * This array contains four elements, one for each wheel. It stores the encoder
     * values we learned from calibrate().
     */
    private double[] initialEncoderValues;

    /**
     * This array contains four PWM-driven SpeedControllers, one for each wheel. We
     * use this abstract class on purpose: any motor controllers than can be driven
     * by PWM signals can drive the wheels forward, not just the Spark MAX. Normal
     * Sparks and CIM motors will work just as well!
     */
    private List<CANSparkMax> speedMotors;

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
        pidControllers = new PIDController[4];
        this.pidErrors = new double[] { 0, 0, 0, 0};
        this.pidIntegrals = new double[] {0, 0, 0, 0};
        for (int i = 0; i < 4; i++) {
            pidControllers[i] = new PIDController(0.075, 0, 0);
            pidControllers[i].setTolerance(GOAL_ROTATION_EPSILON_RADIANS / TWO_PI);
            pidControllers[i].enableContinuousInput(0, 1);
        }

        // By default, we're in crab mode.
        driveMode = DriveMode.CRAB_MODE;

        this.goalThetas = new double[4];
        this.setGoalAngles(new double[] { 0, 0, 0, 0 });
        this.initialEncoderValues = new double[4];
        this.goalSpeeds = new double[4];

        // Fill the speedMotors list with 4 nulls and then overwrite them.
        //
        // By doing things this way, we make this code immune to changes in
        // the values of the indexing constants (FRONT_LEFT, FRONT_RIGHT, and
        // so on.)
        this.speedMotors = new ArrayList<CANSparkMax>();
        Collections.addAll(this.speedMotors, new CANSparkMax[] { null, null, null, null });
        this.speedMotors.set(FRONT_LEFT, new CANSparkMax(FRONT_LEFT_DRIVE_MOTOR_CAN_ID, MotorType.kBrushless));
        this.speedMotors.set(FRONT_RIGHT, new CANSparkMax(FRONT_RIGHT_DRIVE_MOTOR_CAN_ID, MotorType.kBrushless));
        this.speedMotors.set(BACK_LEFT, new CANSparkMax(BACK_LEFT_DRIVE_MOTOR_CAN_ID, MotorType.kBrushless));
        this.speedMotors.set(BACK_RIGHT, new CANSparkMax(BACK_RIGHT_DRIVE_MOTOR_CAN_ID, MotorType.kBrushless));
        for (int i = 0; i < 4; i++) {
            this.speedMotors.get(i).stopMotor();
            this.speedMotors.get(i).set(0);
        }

        // Fill the pivotMotors list with 4 nulls and then overwrite them.
        //
        // For _ease of brain_, the IDs for the CANSparkMax controllers are
        // the same as the index constants.
        this.pivotMotors = new ArrayList<CANSparkMax>();
        Collections.addAll(this.pivotMotors, new CANSparkMax[] { null, null, null, null });
        this.pivotMotors.set(FRONT_LEFT, new CANSparkMax(FRONT_LEFT_TURN_MOTOR_CAN_ID, MotorType.kBrushless));
        this.pivotMotors.set(FRONT_RIGHT, new CANSparkMax(FRONT_RIGHT_TURN_MOTOR_CAN_ID, MotorType.kBrushless));
        this.pivotMotors.set(BACK_LEFT, new CANSparkMax(BACK_LEFT_TURN_MOTOR_CAN_ID, MotorType.kBrushless));
        this.pivotMotors.set(BACK_RIGHT, new CANSparkMax(BACK_RIGHT_TURN_MOTOR_CAN_ID, MotorType.kBrushless));

        this.pivotMotors.forEach(m -> {
            // TODO: Read the required PID constants
            // for the SmartDashBoard. The user may
            // change the values at runtime.

            // Reset to the default state (at least
            // until the next power cycle.)
            // m.restoreFactoryDefaults();

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

    public void setManualDriving(boolean a) {
        if (this.manualDriving && !a) {
            //transitioning for first time from manual to non-manual
            this.setDriveSpeeds(new double[] {0,0,0,0});
        }
        this.manualDriving = a;
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
    List<CANSparkMax> getSpeedMotors() {
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
        this.goalThetas[FRONT_LEFT] = modulus(thetas[FRONT_LEFT], TWO_PI);
        this.goalThetas[FRONT_RIGHT] = modulus(thetas[FRONT_RIGHT], TWO_PI);
        this.goalThetas[BACK_LEFT] = modulus(thetas[BACK_LEFT], TWO_PI);
        this.goalThetas[BACK_RIGHT] = modulus(thetas[BACK_RIGHT], TWO_PI);

        // NB: We're not using this.pidControllers anymore.
        for (int i = 0; i < 4; i++) {
            this.pidControllers[i].reset();
            this.pidControllers[i].setSetpoint(thetas[i]);
        }

        SmartDashboard.putNumber("FL goal theta", this.goalThetas[FRONT_LEFT] * 360 / TWO_PI);
        SmartDashboard.putNumber("FR goal theta", this.goalThetas[FRONT_RIGHT] * 360 / TWO_PI);
        SmartDashboard.putNumber("BL goal theta", this.goalThetas[BACK_LEFT] * 360 / TWO_PI);
        SmartDashboard.putNumber("BR goal theta", this.goalThetas[BACK_RIGHT] * 360 / TWO_PI);
    }

    public void setDriveSpeeds(double[] speeds) {
        //if (!this.crabCenterRotationMode) {
            this.goalSpeeds[FRONT_RIGHT] = speeds[FRONT_RIGHT];
            this.goalSpeeds[FRONT_LEFT] = speeds[FRONT_LEFT];
            this.goalSpeeds[BACK_LEFT] = speeds[BACK_LEFT];
            this.goalSpeeds[BACK_RIGHT] = speeds[BACK_RIGHT];
         /*} else {
            this.goalSpeeds[FRONT_LEFT] = .5;
            this.goalSpeeds[FRONT_RIGHT] = .5;
            this.goalSpeeds[BACK_LEFT] = .5;
            this.goalSpeeds[BACK_RIGHT] = .5;

        }*/

        /*
         * } else { this.goalSpeeds[FRONT_LEFT] = this.crabCenterRotationSpeed;
         * this.goalSpeeds[FRONT_RIGHT] = this.crabCenterRotationSpeed;
         * this.goalSpeeds[BACK_LEFT] = this.crabCenterRotationSpeed;
         * this.goalSpeeds[BACK_RIGHT] = this.crabCenterRotationSpeed; }
         */

        // SmartDashboard.putNumber("FL goal speeds", this.goalSpeeds[FRONT_LEFT]);
        // SmartDashboard.putNumber("FR goal speeds", this.goalSpeeds[FRONT_RIGHT]);
        // SmartDashboard.putNumber("BL goal speeds", this.goalSpeeds[BACK_LEFT]);
        // SmartDashboard.putNumber("BR goal speeds", this.goalSpeeds[BACK_RIGHT]);

    }

    /**
     * In Crab Drive mode, the only way to accurately rotate the frame is to
     * position the wheels such that they are tangent to a common circle, then drive
     * them so as to rotate in place.
     *
     * There are two scenarios when we wish to do this, each with their own exit
     * condition:
     *
     * - MANUAL: Driver is in crab drive mode and uses either joystick 2 (on the
     * XBox controller) or Z-rotation (on the ordinary joystick) to turn. When they
     * release, we stop. - VISION: We rotate until vision finds a vision solution
     * and our horizontal deviation in degrees is minimized. Then we stop.
     *
     * This function controls both starting and stopping that rotation mode. As long
     * as the rotation mode is on, the goal angles coming from
     * {@link crabDriveGetAngle} will point toward that ideal tangent alignment.
     * When the mode is off, the goal angles will be tied to the joystick again.
     *
     * @param speed Non-zero to turn the rotation mode ON; zero to turn it OFF.
     */
    public void setCrabDriveCenterRotation(double speed) {
        if (Math.abs(speed) > 0.05) {
            this.crabCenterRotationMode = true;
            this.crabCenterRotationSpeed = speed;
        } else {
            this.crabCenterRotationMode = false;
            this.crabCenterRotationSpeed = 0.0;
        }

    }

    public void enableSneak() {
        this.sneakMode = true;

    }

    public void disableSneak() {
        this.sneakMode = false;

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
            m.getEncoder().setPosition(0);
            this.initialEncoderValues[index] = rotations;
        }
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
         * here we would set all of the wheels to the required angles and speeds based
         * on goalThetas and etc
         */

        for (int i = 0; i < this.pivotMotors.size(); i++) {
            var m = this.pivotMotors.get(i);

            CANPIDController pidController = m.getPIDController();

            pidController.setP(kP);
            pidController.setI(kI);
            pidController.setD(kD);
            pidController.setIZone(kIz);
            pidController.setFF(kFF);
            pidController.setOutputRange(kMinOutput, kMaxOutput);

            CANEncoder encoder = m.getEncoder();

            // final double wheelPositionInRotations = encoder.getPosition();
            // final double wheelPositionInRadians = wheelPositionInRotations * 2 * Math.PI;

            final double goalRotations = this.goalThetas[i] / (2 * Math.PI);

            final boolean oldSetReferenceMethod = true;//false;
            final boolean oldManualPIDMethod = false;//true;

            if (oldSetReferenceMethod) {
                final double wheelPositionInRotations = encoder.getPosition();
                var errorCode = pidController.setReference(wheelPositionInRotations,
                    ControlType.kPosition);

                errorCode = pidController.setReference(goalRotations,
                    ControlType.kPosition);
            } else if (oldManualPIDMethod) {
                double crabDriveCurrentGoalTheta = (modulus(goalThetas[i], TWO_PI));
                double trueCurrentPosition = modulus(encoder.getPosition() * TWO_PI, TWO_PI);
                double diffSign = Math.signum(crabDriveCurrentGoalTheta - trueCurrentPosition);
                double trueDiff = crabDriveCurrentGoalTheta - trueCurrentPosition;

                // SmartDashboard.putNumber("trueGoalTheta", crabDriveCurrentGoalTheta);
                // SmartDashboard.putNumber("trueCurrentPosition", trueCurrentPosition);
                // SmartDashboard.putNumber("trueDiff", trueDiff);

                // Rotate the pivot motors toward their goal values.
                // Note that we do _NOT_ use PIDController.setReference() here.
                // That is what REV Robotics provided, and it does work, but it
                // has issues when crossing angle boundaries between 0 and 360 degrees.
                //
                // This code below overcomes that flaw.
                final double goalEpsilonDegrees = SmartDashboard.getNumber("goal epsilon", 0);
                if (goalEpsilonDegrees > 0) {
                    GOAL_ROTATION_EPSILON_RADIANS = goalEpsilonDegrees;
                }
                final double goalEpsilonRadians = goalEpsilonDegrees * TWO_PI / 360;

                double differenceModifier = 1.0;

                double approachingMinSpeed = 0.07;

                double slowingThreshold = 1.7;

                double turningSpeed = 0.3;

                if (Math.abs(trueDiff) > Math.PI) {
                    trueDiff = (-Math.signum(trueDiff)*TWO_PI) - trueDiff;
                }

                if (Math.abs(trueDiff) < slowingThreshold) {
                    differenceModifier = Math.abs(trueDiff)/(slowingThreshold-approachingMinSpeed) + approachingMinSpeed;
                }

                if ((Math.abs(trueDiff) > goalEpsilonRadians)) {
                    if (Math.abs(trueDiff) > Math.PI) {
                        m.set(-turningSpeed * diffSign * differenceModifier);
                    } else {
                        m.set(turningSpeed * diffSign * differenceModifier);
                    }
                } else {
                    m.set(0);
                    m.stopMotor();
                }
                SmartDashboard.putNumber("goal epsilon", GOAL_ROTATION_EPSILON_RADIANS);

            }
            // COMMENTED OUT TO TEST POSSIBLE ROLLOVER FIX

            // var errorCode = pidController.setReference(goalRotations,
            // ControlType.kPosition);

            // if (errorCode != CANError.kOk) { // Something went wrong.
            // System.err.printf("pivot motor #%d: pidController.setReference() returned %d
            // (%s)\n",
            // m.getDeviceId(), errorCode, errorCode.name()); }

        }

        SmartDashboard.putNumber("FL pivot",
                modulus(this.pivotMotors.get(FRONT_LEFT).getEncoder().getPosition() * 360, 360));
        SmartDashboard.putNumber("FR pivot",
                modulus(this.pivotMotors.get(FRONT_RIGHT).getEncoder().getPosition() * 360, 360));
        SmartDashboard.putNumber("BL pivot",
                modulus(this.pivotMotors.get(BACK_LEFT).getEncoder().getPosition() * 360, 360));
        SmartDashboard.putNumber("BR pivot",
                modulus(this.pivotMotors.get(BACK_RIGHT).getEncoder().getPosition() * 360, 360));

        // if (!this.crabCenterRotationMode) {
        for (int i = 0; i < this.speedMotors.size(); i++) {
            var m = this.speedMotors.get(i);
            double currentSpeed = m.get();
            SmartDashboard.putNumber(String.format("currentSpeed[%d]", i), currentSpeed);
            final double DRIVE_SPEED_EPSILON = 0.01;
            double deltaSpeed = goalSpeeds[i] - currentSpeed;

            if (Math.abs(deltaSpeed) > DRIVE_SPEED_EPSILON) {

                // We haven't reached our speed yet.  Accelerate *toward* that speed.
                double sign = Math.signum(deltaSpeed);
                final double ACCELERATION = 0.04;

                double newSpeed = currentSpeed + sign * ACCELERATION;
                if (this.sneakMode) {
                    newSpeed *= DRIVE_SNEAK_MODIFIER;
                }

                // The goalSpeed should always be between -1 and 1.  But just in case...
                final double MIN_SPEED = -1.0;
                final double MAX_SPEED = 1.0;
                SmartDashboard.putNumber(String.format("goalSpeeds[%d]", i), this.goalSpeeds[i]);

                // Clamp to the desired range.
                newSpeed = Math.min(MAX_SPEED, Math.max(MIN_SPEED, newSpeed));

                // Actually set our speed.
                m.set(newSpeed);
            } else {
                // Target speed attained, nothing to do.
            }
        } // end (for each speedMotor)
        // } else {
        /*
         * for (int i = 0; i < this.speedMotors.size(); i++) { var m =
         * this.speedMotors.get(i); if (!this.sneakMode) {
         * m.set(this.crabCenterRotationSpeed); } else {
         * m.set(this.crabCenterRotationSpeed * DRIVE_SNEAK_MODIFIER); } }
         */
        // }
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
     * This is a version of the modulus function (fmod() in libc) that behaves
     * correctly in the presence of negative numbers. For instance, while -10 % 360
     * returns -10, modulus(-10, 360) returns 350.
     *
     * The sign of the result will always match the sign of b.
     *
     * Credit goes to https://stackoverflow.com/a/4412200 for the elegantly simple
     * insight.
     */
    private static final double modulus(double a, double b) {
        // - a % b is always lower than b, regardless of whether b is positive
        // or negative.
        //
        // - If a is negative, then a % b is always between 0 and b, so adding
        // b makes a % b positive.
        //
        // Of course, if a was positive to begin with, then adding b will
        // make the result greater than b, so a final modulus operation
        // takes care of that.
        return ((a % b) + b) % b;
    }

    /**
     * Calculates were we need to rotate a swerve drive wheel to match the alignment
     * that the human driver has specified. This is used during crab drive mode on
     * C3-PO-TATO.
     *
     * @param currentAngle  The current position of the swerve wheel as determined
     *                      by the encoder, in radians.
     * @param joystickAngle The current angle of the joystick, in radians.
     * @return A new angle that is, at most, π radians away from the current
     *         position, and that is equivalent to the joystickAngle (modulo 2π.)
     */
    private static final double getNewAngle(double currentAngle, double joystickAngle) {

        double theta = modulus(currentAngle + modulus(joystickAngle - currentAngle, TWO_PI), TWO_PI);

        // if (theta > modulus(Math.PI + currentAngle, TWO_PI)) {
        // // The calculation would have had us turning more than 180
        // // degrees. Rotate to the same position, but from the opposite
        // // direction.
        // theta = currentAngle - (TWO_PI - theta);
        // }

        return theta;
    }

    /**
     *
     * @param vector2 A vector based on axis input
     * @return An array of four directional angles; see snakeDriveGetAngle() for
     *         more information.
     */
    public double[] crabDriveGetAngle(Vector2d joystickVector) {
        double angles[] = new double[4];

        //if (!this.crabCenterRotationMode) {

        // The reference vector points straight forward.
        // Its length is 1.
        final Vector2d referenceVector = new Vector2d(0, -1);

        // The joystick vector comes from our args.
        // We need to normalize it so its value is 1.
        final double joystickVectorLength = joystickVector.magnitude();
        if (joystickVectorLength < JOYSTICK_EPSILON) {
            // The joystick is inside our dead zone. There's nothing to rotate toward.
            turn_enabled = false;
            for (int i = 0; i < 4; i++) {
                angles[i] = this.pivotMotors.get(i).getEncoder().getPosition() * TWO_PI;
            }
            return angles;
        } else {
            // This shows that we are not in the dead zone of the controller.
            // This means that the robot will attempt to rotate to the joystick angle.
            for (int i = 0; i < 4; i++) {
                angles[i] = 0;
            }
            turn_enabled = true;
        }

        final Vector2d normalizedVector = new Vector2d(joystickVector.x / joystickVectorLength,
                joystickVector.y / joystickVectorLength);
        // SmartDashboard.putNumber("Atan2 theta", Math.atan2(normalizedVector.y,
        // normalizedVector.x));
        double joystickAngle = Math.atan2(normalizedVector.y, normalizedVector.x);

        // Atan2() has a range of [-180, 180), with 0 on the positive X axis.
        // Shift so 0 is on the positive Y axis, then wrap the range to [0,
        // 360).
        joystickAngle = modulus(joystickAngle + Math.PI / 2.0, TWO_PI);

        for (int i = 0; i < 4; i++) {
            var motor = this.pivotMotors.get(i);
            double currentRotations = motor.getEncoder().getPosition();
            double currentAngle = currentRotations * TWO_PI;

            double theta = getNewAngle(currentAngle, joystickAngle);
            angles[i] = theta;
            SmartDashboard.putNumber("joystick angle", joystickAngle * 180 / Math.PI);
        }
        /*
         * } else { // NEED to align the four wheels to be tanget to a circle around the
         * center, // that completely circumscribes a circle throught the four wheels //
         * also known as set them on a circle through all four wheels double length =
         * WHEEL_DRIVE_VERTICAL_WHEEL_TO_CENTER_DISTANCE; double width =
         * WHEEL_DRIVE_HORIZONTAL_WHEEL_TO_CENTER_DISTANCE;
         *
         * double rawAngle = Math.tan(length / width); // each wheel needs to be set to
         * the angle, but based on a different quadrant // probably easiest to do a
         * switch case for each wheel, rather than find the // equation
         *
         * for (int i = 0; i < 4; i++) { switch (i) { // reminder that 0 is FL, 1 is BL,
         * 2 is BR, 3 is FR case 0: // 90 + rawAngle break; case 1: // 90 - rawAngle
         * break; case 2: // raw angle break; case 3: // 180 - rawAngle break;
         *
         * } } }
         */

        // SmartDashboard.putNumber("getNewAngle(3)", angles[3] / Math.PI * 180);
        // SmartDashboard.putNumber("CurrentAngle(3)",
        // this.pivotMotors.get(3).getEncoder().getPosition() * 360);
        return angles;
    }

    public void goalAnglesReached() {
        int counter = 0;
        for (int i = 0; i < 4; i++) {
            var m = this.pivotMotors.get(i);
            CANEncoder encoder = m.getEncoder();
            // if this.goalThetas[i] =
        }
    }

    public void setOppositeAngle() {
        double[] thetas = new double[4];
        for (int i = 0; i < 4; i++) {
            var m = this.pivotMotors.get(i);
            CANEncoder encoder = m.getEncoder();

            double positionRadians = encoder.getPosition() * TWO_PI;
            positionRadians = modulus(positionRadians, TWO_PI);

            double newPositionRadians = positionRadians + Math.PI;
            newPositionRadians = modulus(newPositionRadians, TWO_PI);

            thetas[i] = newPositionRadians;
        }
        this.setGoalAngles(thetas);
    }

    /**
     * Checks whether the wheels have reached their desired positions saved in goalThetas.
     */
    public boolean isGoalThetasReached() {
        for (int i = 0; i <4; i++) {
            var m = this.pivotMotors.get(i);
            CANEncoder encoder = m.getEncoder();
            final double encoderPositionRadians = encoder.getPosition()*TWO_PI;
            if ((Math.abs(encoderPositionRadians - goalThetas[i]) > GOAL_ROTATION_EPSILON_RADIANS)) {
                return false;
            }
        }
        return true;
    }

}