package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.Collection;
import java.util.List;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Controller;
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
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

public class NewWheelDriveSubsystem extends SubsystemBase {
    
    private Trajectory autonomousTrajectory;
    private SwerveDriveKinematics kinematics;
    private Timer trajectoryTimer; 
    private Gyro robotGyro;
    private SwerveDriveOdometry odometry;

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
                        
        // TODO: Actually move the motors using the swerve module states
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

          
        if (autonomousTrajectory != null) {
            Trajectory.State state = autonomousTrajectory.sample(trajectoryTimer.get());
            
            // In this call, we want the robot face 0 degrees relative to its starting angle. 
            ChassisSpeeds velocities = controller.calculate(odometry.getPoseMeters(), state, Rotation2d.fromDegrees(0.0));
            
            SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(velocities);
            this.drive(List<SwerveModuleState>.of(swerveModuleStates));
        }
    }

}