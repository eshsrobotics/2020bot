package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WheelDriveSubsystem;

/**
 * When this command is running, the robot will be in *crab drive mode*:
 *
 * - One joystick controls the strafing direction. - One joystick controls the
 * speed.
 */
public class AutoTimedDrive extends CommandBase {
    private Command commandToUse;

    private final WheelDriveSubsystem wheelDriveSubsystem;
    private double m_time;
    private Timer autoTimer = new Timer();
    private double startTime;

    /**
     * Initializes this object by explicitly specifying the subsystems it requires.
     *
     * @param wheelDriveSubsystem A subsystem that can be used to control turning and driving the bot.
     * @param intakeSubsystem A subsystem that can be used for controlling the intake and belts.
     * @param shooterSubsystem A subsystem that can be used to control the shooter flywheels.  If
     * @param visionTrackingCommand A composite command that rotates smartly towards a vision target, if found.
     * @param time The amount of time, in seconds, to spend driving forward in order to gain minimum autonomous points.
     */
    public AutoTimedDrive(WheelDriveSubsystem wheelDriveSubsystem,
                          IntakeSubsystem intakeSubsystem,
                          ShooterSubsystem shooterSubsystem,
                          Command visionTrackingCommand,
                          double time) {

        this.wheelDriveSubsystem = wheelDriveSubsystem;
        m_time = time;
        this.addRequirements(wheelDriveSubsystem);
        this.addRequirements(shooterSubsystem);
        this.addRequirements(intakeSubsystem);

        // This operates on the assumption that we are facing away from the goal.
        // It just drives us across the line to get the points, then stops.
        Command oldAutonomousCommandSequence = new InstantCommand(() -> {
            double[] thetas = { 0, 0, 0, 0 };
            this.wheelDriveSubsystem.setGoalAngles(thetas);
            double[] speeds = { 0.5, 0.5, 0.5, 0.5 };
            this.wheelDriveSubsystem.setDriveSpeeds(speeds);
        }).andThen(new WaitCommand(time)).andThen(new InstantCommand(() -> {
            this.wheelDriveSubsystem.setDriveSpeeds(new double[] { 0, 0, 0, 0 });
        }));

        // This command lines up the wheels to drive forward, drives for
        // time seconds, and then stops.  It's used as part of more complicated
        // autonomous commands.
        //
        // This subcommand operates under the assumption that the robot's front end
        // (the shooter) is facing toward the opposite side of the field.
        Command driveForward = new InstantCommand(() -> {
            double[] thetas = { 0, 0, 0, 0 };
            this.wheelDriveSubsystem.setGoalAngles(thetas);
            double[] speeds = { -0.5, -0.5, -0.5, -0.5 };
            this.wheelDriveSubsystem.setDriveSpeeds(speeds);
            shooterSubsystem.startShooter(.55, .55);
        }).andThen(new WaitCommand(time));

        // This operates on the assumption we are already facing the target and aligned
        // toward it as well. It drives the robot across the line for the points, then
        // moves closer to the target to shoot.
        //
        // I guess testing and debugging this is not a priority right now.
        //
        // this.newAutonomousCommandSequenceWithoutVision = driveForward
        // .andThen(new InstantCommand(() -> {
        //     double[] speeds = { 0.5, 0.5, 0.5, 0.5 };
        //     this.wheelDriveSubsystem.setDriveSpeeds(speeds);
        // })).andThen(new WaitCommand(time)).andThen(new InstantCommand(() -> {
        //     this.wheelDriveSubsystem.setDriveSpeeds(new double[] { 0, 0, 0, 0 });
        //     intakeSubsystem.enablesBelts();
        // })).andThen(new WaitCommand(5)).andThen(new InstantCommand(() -> {
        //     intakeSubsystem.disablesBelts();
        //     shooterSubsystem.stopShooter();
        // }));

        // This uses the Limelight to track vision targets...or at least, it's supposed to.
        Command newAutonomousCommandSequenceWithVision = driveForward.andThen(visionTrackingCommand);

        // Which command do we use?
        this.commandToUse = oldAutonomousCommandSequence;
    }

    /**
     * Begin the autonomous command.
     */
    @Override
    public void initialize() {
        CommandScheduler.getInstance().schedule(this.commandToUse);

        // startTime = Timer.getFPGATimestamp();
        // double[] thetas = { 0, 0, 0, 0 };
        // this.wheelDriveSubsystem.setGoalAngles(thetas);
    }

    /**
     * Lets the scheduler know when the autonomous command is complete.
     */
    @Override
    public boolean isFinished() {
        return commandToUse.isFinished();

        // return Timer.getFPGATimestamp() - startTime > m_time;
    }

    /**
     * This function is run continuously for as long as the command is active.
     */
    @Override
    public void execute() {
        // Don't think this is needed.
        // commandToUse.execute();

        // double[] speeds = { 0.5, 0.5, 0.5, 0.5 };
        // this.wheelDriveSubsystem.setDriveSpeeds(speeds);
    }
}