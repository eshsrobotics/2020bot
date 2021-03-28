package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.controls.ControlScheme;
import frc.robot.controls.CrabDriveScheme;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

import static frc.robot.Constants.*;

public class NewWheelDriveSubsystem extends SubsystemBase {
    private boolean sneakMode = false;
    private Trajectory autonomousTrajectory;
    private SwerveDriveKinematics kinematics;
    private Timer trajectoryTimer;
    private Gyro robotGyro;
    private SwerveDriveOdometry odometry;

    /**
     * When drive is called, retain goal speeds
     */
    private List<SwerveModuleState> goalStates;

    /**
     * This array contains four PWM-driven SpeedControllers, one for each wheel. We
     * use this abstract class on purpose: any motor controllers than can be driven
     * by PWM signals can drive the wheels forward, not just the Spark MAX. Normal
     * Sparks and CIM motors will work just as well!
     */
    private List<CANSparkMax> speedMotors;

    /**
     * This is an array of 4 boolean values, one for each of the 4 speed motors. 
     * Whenever the reversal flag is true for a motor, then we will reverse the motor's direction in software.
     */
    private List<Boolean> reversalFlags;

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
     * These fudge factors are the current values given by the encoders after
     * calibrating the wheels. These values are specified in units of rotations.
     * 
     * These values tell us what values to set the wheels to when calibrating.
     */
    private List<Double> pivotEncoderFudgeFactors;

    /**
     * The list of encoder positions as they are measured during construction.
     * They are measured in units of rotation.
     */
    private List<Double> initialEncoderPositions;

    /**
     * PID constants. Each of these appears on the SmartDashboard. We're going to
     * have to experiment with these.
     *
     * See
     * https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/smartdashboard/index.html
     * for more information.
     */
    private double kP = 1, kI = 1e-4, kD = 1, kIz = 0, kFF = 0, kMaxOutput = 1, kMinOutput = -1;

    /**
     * A way of mapping user input into robot movement. This controls which
     * ControlScheme we are using a the moment.
     */
    private ControlScheme currentControlScheme = null;

    /**
     * This is the source for user input during periodic.
     */
    private InputSubsystem userInput = null;

    /**
     * Initializes an instance of this class.
     * 
     * @param inputSubsystem Lets us take 2D vectors from user input, and translate
     *                       it into movement.
     */
    public NewWheelDriveSubsystem(InputSubsystem inputSubsystem) {
        setName("NewWheelDriveSubsystem");
        final double h = Constants.WHEEL_DRIVE_HORIZONTAL_WHEEL_TO_CENTER_DISTANCE;
        final double v = Constants.WHEEL_DRIVE_VERTICAL_WHEEL_TO_CENTER_DISTANCE;
        
        // assuming that -h is left, and -v is the back side of the robot
        kinematics = new SwerveDriveKinematics(new Translation2d(-h, +v), // FRONT_LEFT
                                               new Translation2d(-h, -v), // BACK_LEFT
                                               new Translation2d(+h, -v), // BACK_RIGHT
                                               new Translation2d(+h, +v)); // FRONT_RIGHT
        trajectoryTimer = null;
        
        currentControlScheme = new CrabDriveScheme(kinematics);
        userInput = inputSubsystem;

        this.robotGyro = new ADXRS450_Gyro();
        // Because we do not know if the gyro is ready at this point, we lazy-initialize
        // the odometry
        // object.
        this.odometry = null;

        // Fill the speedMotors list with 4 nulls and then overwrite them.
        //
        // By doing things this way, we make this code immune to changes in
        // the values of the indexing constants (FRONT_LEFT, FRONT_RIGHT, and
        // so on.)
        this.reversalFlags = new ArrayList<Boolean>();
        Collections.addAll(this.reversalFlags, new Boolean[] { true, true, false, false });
        this.speedMotors = new ArrayList<CANSparkMax>();
        Collections.addAll(this.speedMotors, new CANSparkMax[] { null, null, null, null });
        this.speedMotors.set(FRONT_LEFT, new CANSparkMax(FRONT_LEFT_DRIVE_MOTOR_CAN_ID, MotorType.kBrushless));
        this.speedMotors.set(FRONT_RIGHT, new CANSparkMax(FRONT_RIGHT_DRIVE_MOTOR_CAN_ID, MotorType.kBrushless));
        this.speedMotors.set(BACK_LEFT, new CANSparkMax(BACK_LEFT_DRIVE_MOTOR_CAN_ID, MotorType.kBrushless));
        this.speedMotors.set(BACK_RIGHT, new CANSparkMax(BACK_RIGHT_DRIVE_MOTOR_CAN_ID, MotorType.kBrushless));
        for (int i = 0; i < 4; i++) {
            this.speedMotors.get(i).stopMotor();
            this.speedMotors.get(i).set(0);
            this.speedMotors.get(i).setInverted(this.reversalFlags.get(i));
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

        this.initialEncoderPositions = this.getInitialEncoderValues();

        // Even though we aren't driving, we still need an initial goal.
        List<SwerveModuleState> swerveModuleStates = new ArrayList<SwerveModuleState>();
        for (int i = 0; i < 4; i++) {
            swerveModuleStates.add(new SwerveModuleState(0, new Rotation2d(0)));
        }
        //this.drive(swerveModuleStates);
    }

    public NewWheelDriveSubsystem() {
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
            m.getEncoder().setPosition(0);
        }
    }
    
    /**
     * Grabs the value of the encoders at the time of construction.
     * @return Returns a list of four encoder positions, starting with the front
     * left and proceeding counterclockwise. The encoder positions are specified
     * in units of rotations.
     */
    public List<Double> getInitialEncoderValues() {
        var result = new ArrayList<Double>();
        Collections.addAll(result, new Double[] {
            this.pivotMotors.get(FRONT_LEFT).getEncoder().getPosition(),
            this.pivotMotors.get(BACK_LEFT).getEncoder().getPosition(),
            this.pivotMotors.get(BACK_RIGHT).getEncoder().getPosition(),
            this.pivotMotors.get(FRONT_RIGHT).getEncoder().getPosition(), 
        });
        return result;
    }
    
    /**
     * When the robot is disabled, we return to the initial positions of the
     * pivot wheels so the drivers do not have to calibrate as often.
     * @param initialEncoderValues An array of encoder values such as what
     * getInitialEncoderValues() returns.
     */
    private void setPivotPositions(List<Double> initialEncoderValues) {

        for (int i = 0; i < this.pivotMotors.size(); i = i + 1) {
            CANSparkMax m = this.pivotMotors.get(i);
            CANPIDController pidController = m.getPIDController();
            pidController.setReference(initialEncoderValues.get(i), ControlType.kPosition);
        };
    }

    public void resetPivotPositions() {
        this.setPivotPositions(this.initialEncoderPositions);
    }

	/**
     * This function allows this object to set a timer whenever it receives a
     * trajectory
     * 
     * @param newTrajectory The trajectory to set.
     */
    public void setTrajectory(Trajectory newTrajectory) {
        this.autonomousTrajectory = newTrajectory;
        this.trajectoryTimer = new Timer();
        this.trajectoryTimer.start();
    }

    /**
     * Instantaneously update our drive speed and direction.
     * 
     * @param swerveModuleStates An array of direction/speed pairs, one for each
     *                           wheel (4 in total).
     */
    public void drive(List<SwerveModuleState> swerveModuleStates) {
        this.goalStates = swerveModuleStates;

        // Update the swerve drive odometry (position)
        if (odometry == null) {
            // We are assuming gyro is ready by now
            // Set the Odometry to set itself to the origin (0, 0) whenever the Gyro turns
            // on.
            odometry = new SwerveDriveOdometry(kinematics, robotGyro.getRotation2d());
        }

        odometry.update(robotGyro.getRotation2d(), swerveModuleStates.get(0), swerveModuleStates.get(1),
                swerveModuleStates.get(2), swerveModuleStates.get(3));

        // Actually move the motors using the swerve module states
        // We will only call setReference in response to changes in the goal angles and
        // speeds (swerveModuleStates)
        // This will hopefully address motor overheating issued caused when we called
        // this function during periodic

        for (int i = 0; i < pivotMotors.size(); i++) {
            var m = pivotMotors.get(i);
            var pidController = m.getPIDController();
            // We dont think we need a redundant setReferance call
            // var encoder = m.getEncoder();
            // var currentPositionInRotations = encoder.getPosition();

            final double goalPositionInRotations = swerveModuleStates.get(i).angle.getDegrees() / 360;
            pidController.setReference(goalPositionInRotations, ControlType.kPosition);
        }

    }

    /**
     * Turns sneak on by changing sneakMode to true.
     * sneakMode makes it so that the robot moves at a reduced percentage of its normal speed.
     * This is important when trying to perform more precise maneuvers, such as lining up to shoot balls or move objects.
     */
    public void enableSneak() {
        sneakMode = true;
    }

    /**
     * Turns sneak off by changing sneakMode to false.
     */
    public void disableSneak() {
        sneakMode = false;
    }

    /**
     * Instantaneous updates to the drive.
     */
    @Override 
    public void periodic() {

        // Here, our rotation profile constraints were a max velocity
        // of 1 rotation per second and a max acceleration of 180 degrees
        // per second squared.
        var controller = new HolonomicDriveController(new PIDController(1, 0, 0),
                                                      new PIDController(1, 0, 0), 
                                                      new ProfiledPIDController(1, 0, 0, 
                                                                               new TrapezoidProfile.Constraints(6.28, 3.14)));

        // If there is an autonomous trajectory, follow it  
        if (autonomousTrajectory != null && trajectoryTimer.get() < autonomousTrajectory.getTotalTimeSeconds()) {
            Trajectory.State state = autonomousTrajectory.sample(trajectoryTimer.get());
            
            // In this call, we want the robot face 0 degrees relative to its starting angle. 
            ChassisSpeeds velocities = controller.calculate(odometry.getPoseMeters(), state, Rotation2d.fromDegrees(0.0));
            SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(velocities);
            
            ArrayList<SwerveModuleState> foo = new ArrayList<SwerveModuleState>();
            Collections.addAll(foo, swerveModuleStates);
            this.drive(foo);
        } else {
            // Teleop mode (Driving according to human input)
            Vector2d strafingVector = userInput.getVector();
            Vector2d turningVector = new Vector2d(userInput.getCrabTurnValue(), 0);
            SmartDashboard.putNumber("strafe x", strafingVector.x);
            SmartDashboard.putNumber("strafe y", strafingVector.y);
            SmartDashboard.putNumber("turn x", turningVector.x);
            SmartDashboard.putNumber("turn y", turningVector.y);

            strafingVector = deadzone(strafingVector);
            turningVector = deadzone(turningVector);

            if (strafingVector.x != 0 || strafingVector.y != 0 || turningVector.x != 0) {
                var swerveModuleStates = currentControlScheme.driveAndTurn(strafingVector.x, strafingVector.y, turningVector.x);
                this.drive(swerveModuleStates);
            } else {
                // Keep the current direction, but make the goal speed 0
                if (this.goalStates != null) {
                    this.goalStates.forEach(swerveModuleState -> {
                        swerveModuleState.speedMetersPerSecond = 0;
                    });
                }
            }
        }   

        // Make the speed motors reach their goal speeds.
        for (int i = 0; i < this.speedMotors.size(); i++) {
            if (goalStates == null) {
                continue;
            }
            CANSparkMax m = this.speedMotors.get(i);
            double currentSpeed = m.get();
            SmartDashboard.putNumber(String.format("currentSpeed[%d]", i), currentSpeed);
            final double DRIVE_SPEED_EPSILON = 0.05;

            // deltaSpeed is the change we want to apply to our current speed.
            double deltaSpeed = (goalStates.get(i).speedMetersPerSecond / Constants.MAX_ROBOT_SPEED_MPS) - currentSpeed;

            if (Math.abs(deltaSpeed) > DRIVE_SPEED_EPSILON) {

                // We haven't reached our speed yet.  Accelerate *toward* that speed.
                double sign = Math.signum(deltaSpeed);
                final double ACCELERATION = 0.07;

                double newSpeed = currentSpeed + sign * ACCELERATION;
                if (this.sneakMode) {
                    newSpeed *= DRIVE_SNEAK_MODIFIER;
                }

                Vector2d leftJoystickVector = userInput.getVector();
                final double magnitude = currentControlScheme.getMagnitude(leftJoystickVector.x, leftJoystickVector.y);

                // The goalSpeed should always be between -1 and 1.  But just in case...
                final double MIN_SPEED = -1.0 * DRIVE_SPEED_MULTIPLIER * magnitude;
                final double MAX_SPEED = 1.0 * DRIVE_SPEED_MULTIPLIER * magnitude;
                // SmartDashboard.putNumber(String.format("goalSpeeds[%d]", i), this.goalSpeeds[i]);

                // Clamp to the desired range.
                newSpeed = Math.min(MAX_SPEED, Math.max(MIN_SPEED, newSpeed));

                // Actually set our speed.
                if (Math.abs(newSpeed) < DRIVE_SPEED_EPSILON) {
                    m.stopMotor();
                } else {
                    m.set(newSpeed);
                }
                
            } else {
                // Target speed attained, nothing to do.
            }
        } // end (for each speedMotor)
    }

    /**
     * Creates a portion of no movement on the joystick so that the drivetrain is less jittery.
     * 
     * @param userInputVector is a vector with components between -1 and 1.
     * @return Returns an adjusted vector.
     *         If this vector is the zero vector, the joystick is completely deadzoned.
     */
    private Vector2d deadzone(Vector2d userInputVector) {
        Vector2d result = new Vector2d(userInputVector.x, userInputVector.y);
        if (Math.abs(userInputVector.x) <= DEADZONE) {
            result.x = 0;
        }

        if (Math.abs(userInputVector.y) <= DEADZONE) {
            result.y = 0;
        }
        
        return result;
    }

}