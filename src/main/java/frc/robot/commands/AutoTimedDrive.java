package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.*;


import frc.robot.subsystems.InputSubsystem;
import frc.robot.subsystems.WheelDriveSubsystem;


/**
 * When this command is running, the robot will be in *crab drive mode*:
 * 
 * - One joystick controls the strafing direction. - One joystick controls the
 * speed.
 */
public class AutoTimedDrive extends CommandBase {

    private final WheelDriveSubsystem wheelDriveSubsystem;
    private double m_time;
    private Timer autoTimer = new Timer();
    private double startTime;
    /**
     * Initializes this object by explicitly specifying the subsystems it requires.
     */
    public AutoTimedDrive(WheelDriveSubsystem wheelDriveSubsystem, double time) {
        this.wheelDriveSubsystem = wheelDriveSubsystem;
        m_time = time;
        this.addRequirements(wheelDriveSubsystem);
    }

    /**
     * We are finished as soon as the wheel drive says we are in snake drive mode.
     */
    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        double [] thetas = {0,0,0,0};
        this.wheelDriveSubsystem.setGoalAngles(thetas);
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp()-startTime > m_time;
    }

    /**
     * This function is run continuously for as long as the command is active.
     */
    @Override
    public void execute() {

        double [] speeds = {0.5, 0.5, 0.5, 0.5};
        this.wheelDriveSubsystem.setDriveSpeeds(speeds);
    }
}