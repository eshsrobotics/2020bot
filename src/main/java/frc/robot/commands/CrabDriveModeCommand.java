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
        boolean but1 = this.inputSubsystem.getControllerOneButt();
        boolean but2 = this.inputSubsystem.getControllerTwoButt();
        boolean but3 = this.inputSubsystem.getControllerThreeButt();
        boolean but4 = this.inputSubsystem.getControllerFourButt();
        double[] goalCrabThetas = this.wheelDriveSubsystem.crabDriveGetAngle(directionalVector, but1, but2, but3, but4);
        double[] goalTrueThetas = new double[4];
        double[] goalTrueSpeeds = new double[4];
        for (int i = 0; i < 4; i++) {
            if (0 <= i && i <= 3) {
                goalTrueThetas[i] = goalCrabThetas[i];
            }
            if (4 <= i && i <= 7) {
                goalTrueSpeeds[i - 4] = goalCrabThetas[i];
            }
        }
        //double shooterSpeed = goalCrabThetas[4];
        this.wheelDriveSubsystem.setGoalAngles(goalTrueThetas);
        //this.wheelDriveSubsystem.setDriveSpeeds(goalTrueSpeeds);

        super.execute();
    }    
}