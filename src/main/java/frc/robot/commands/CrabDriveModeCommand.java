package frc.robot.commands;

import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
       
        this.wheelDriveSubsystem.setGoalAngles(goalCrabThetas);
        
        // Maximum value of each joystick channel is postive one. Maximum length of joystick vector is the hypotenuse of a right traingle with sides of one and one.    
        final double MAX_JOYSTICK_MAGNITUDE = Math.sqrt(1 + 1);

        // Speed is between zero and one. 
        double speed = directionalVector.magnitude() / MAX_JOYSTICK_MAGNITUDE;
        speed = speed * Constants.DRIVE_SPEED_MULTIPLIER;
        double[] driveSpeeds = {
            speed,speed,speed,speed
        };
        this.wheelDriveSubsystem.setDriveSpeeds(driveSpeeds);

        super.execute();
    }    
}