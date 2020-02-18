package frc.robot.commands;

import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.InputSubsystem;
import frc.robot.subsystems.WheelDriveSubsystem;

/**
 * When this command is running, the robot will be in *crab drive mode*:
 * 
 * - One joystick controls the strafing direction. - One joystick controls the
 * speed.
 */
public class CrabDriveModeCommand extends CommandBase {

    private final WheelDriveSubsystem wheelDriveSubsystem;
    private final InputSubsystem inputSubsystem;

    /**
     * Initializes this object by explicitly specifying the subsystems it requires.
     */
    public CrabDriveModeCommand(WheelDriveSubsystem wheelDriveSubsystem, InputSubsystem inputSubsystem) {
        this.wheelDriveSubsystem = wheelDriveSubsystem;
        this.inputSubsystem = inputSubsystem;
        this.addRequirements(wheelDriveSubsystem, inputSubsystem);
    }

    /**
     * We are finished as soon as the wheel drive says we are in snake drive mode.
     */
    @Override
    public boolean isFinished() {
        return this.wheelDriveSubsystem.getDriveMode() != WheelDriveSubsystem.DriveMode.CRAB_MODE;
    }

    /**
     * This function is run continuously for as long as the command is active.
     */
    @Override
    public void execute() {
        // NOTE: If NetworkTables-input 

        Vector2d directionalVector = this.inputSubsystem.getVector();
        double[] goalCrabThetas = this.wheelDriveSubsystem.crabDriveGetAngle(directionalVector);
        this.wheelDriveSubsystem.setGoalAngles(goalCrabThetas);

        super.execute();
    }    
}