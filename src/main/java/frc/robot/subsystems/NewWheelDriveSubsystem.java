package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

public class NewWheelDriveSubsystem extends SubsystemBase {
    
    private Trajectory autonomousTrajectory;
    private SwerveDriveKinematics kinematics;
    private Timer trajectoryTimer; 

    /**
     * Initializes an instance of this class!?
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
    }

    /**
     *This function allows this object to set a timer whenever it recieves a trajectory
     * @param newTrajectory The trajectory to set.
     */
    public void setTrajectory(Trajectory newTrajectory) {
        this.autonomousTrajectory = newTrajectory;
        this.trajectoryTimer = new Timer();
        this.trajectoryTimer.start();
    }

    /**
     * Instantaneous updates to the drive.
     */
    @Override 
    public void periodic() {
        if (autonomousTrajectory != null) {
            Trajectory.State state = autonomousTrajectory.sample(trajectoryTimer.get());
        }
    }

}