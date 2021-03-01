package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
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

    
    /**
     * Initializes an instance of this class.
     */
    public NewWheelDriveSubsystem() {
        setName("NewWheelDriveSubsystem");
        final double h = Constants.WHEEL_DRIVE_HORIZONTAL_WHEEL_TO_CENTER_DISTANCE;
        final double v = Constants.WHEEL_DRIVE_VERTICAL_WHEEL_TO_CENTER_DISTANCE;
        // assuming that -h is left, and -v is the back side of the robot
        kinematics = new SwerveDriveKinematics(new Translation2d(-h, +v),  // FRONT_LEFT
                                               new Translation2d(-h, -v),  // BACK_LEFT
                                               new Translation2d(+h, -v),  // BACK_RIGHT
                                               new Translation2d(+h, +v)); // FRONT_RIGHT
        trajectoryTimer = null;

        this.robotGyro = new ADXRS450_Gyro();
        // Because we do not know if the gyro is ready at this point, we lazy-initialize the odometry
        // object.
        this.odometry = null;

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
     * This function allows this object to set a timer whenever it receives a trajectory
     * @param newTrajectory The trajectory to set.
     */
    public void setTrajectory(Trajectory newTrajectory) {
        this.autonomousTrajectory = newTrajectory;
        this.trajectoryTimer = new Timer();
        this.trajectoryTimer.start();
    }

    /**
     * Instantaneously update our drive speed and direction. 
     * @param swerveModuleStates An array of direction/speed pairs, one for each wheel (4 in total). 
     */
    public void drive(List<SwerveModuleState> swerveModuleStates) {
        this.goalStates = swerveModuleStates;

        // Update the swerve drive odometry (position)
        if (odometry == null) {
            // We are assuming gyro is ready by now
            // Set the Odometry to set itself to the origin (0, 0) whenever the Gyro turns on.
            odometry = new SwerveDriveOdometry(kinematics, robotGyro.getRotation2d());
        } 

        odometry.update(robotGyro.getRotation2d(),
                        swerveModuleStates.get(0),
                        swerveModuleStates.get(1),
                        swerveModuleStates.get(2),
                        swerveModuleStates.get(3));
                        
        // Actually move the motors using the swerve module states
        // We will only call setReference in response to changes in the goal angles and speeds (swerveModuleStates)
        // This will hopefully address motor overheating issued caused when we called this function during periodic
        

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
            // TODO: IMPLEMENT
        }

        // Make the speed motors reach their goal speeds.
        for (int i = 0; i < this.speedMotors.size(); i++) {
            var m = this.speedMotors.get(i);
            double currentSpeed = m.get();
            SmartDashboard.putNumber(String.format("currentSpeed[%d]", i), currentSpeed);
            final double DRIVE_SPEED_EPSILON = 0.01;
            double deltaSpeed = (goalStates.get(i).speedMetersPerSecond / Constants.MAX_ROBOT_SPEED_MPS) - currentSpeed;

            if (Math.abs(deltaSpeed) > DRIVE_SPEED_EPSILON) {

                // We haven't reached our speed yet.  Accelerate *toward* that speed.
                double sign = Math.signum(deltaSpeed);
                final double ACCELERATION = 0.04;

                double newSpeed = currentSpeed + sign * ACCELERATION;
                // if (this.sneakMode) {
                //     newSpeed *= DRIVE_SNEAK_MODIFIER;
                // }

                // The goalSpeed should always be between -1 and 1.  But just in case...
                final double MIN_SPEED = -1.0;
                final double MAX_SPEED = 1.0;
                // SmartDashboard.putNumber(String.format("goalSpeeds[%d]", i), this.goalSpeeds[i]);

                // Clamp to the desired range.
                newSpeed = Math.min(MAX_SPEED, Math.max(MIN_SPEED, newSpeed));

                // Actually set our speed.
                m.set(newSpeed);
            } else {
                // Target speed attained, nothing to do.
            }
        } // end (for each speedMotor)
    }

}