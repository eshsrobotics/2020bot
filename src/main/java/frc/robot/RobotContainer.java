/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.CrabDriveModeCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.BeltButton;
import frc.robot.subsystems.ClimbDownButton;
import frc.robot.subsystems.ClimbUpButton;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.InputSubsystem;
import frc.robot.subsystems.IntakeButton;
import frc.robot.subsystems.ShootButton;
import frc.robot.subsystems.WheelDriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SneakButton;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 *           
 * 
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
    private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

    private final WheelDriveSubsystem wheelDrive = new WheelDriveSubsystem();
    private final InputSubsystem inputSubsystem = new InputSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    private final VisionSubsystem visionSubsystem = new VisionSubsystem();

    private final ShootButton shootButton = new ShootButton(inputSubsystem);
    private final ClimbUpButton climbUpButton = new ClimbUpButton(inputSubsystem);
    private final ClimbDownButton climbDownButton = new ClimbDownButton(inputSubsystem);
    private final SneakButton sneakButton = new SneakButton(inputSubsystem);
    private final BeltButton beltButton = new BeltButton(inputSubsystem);
    private final IntakeButton intakeButton = new IntakeButton(inputSubsystem);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings.
        configureButtonBindings();

        this.wheelDrive.calibrate();
        this.wheelDrive.setDefaultCommand(new CrabDriveModeCommand(wheelDrive, inputSubsystem));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Our buttons are a bit special because they support three different
        // input sources at once (joysticks, XBox-compatible controllers, and
        // keyboard input via NetworkTables-input.jar.) So for now, we need
        // Button wrapper classes around the InputSubsystem which handles all
        // of these input sources.

        // this.shooterTop.setDefaultCommand(new ShooterCommand(shooterTop,
        // inputSubsystem, shooterBot));
        // System.out.println(shootButton.get());
        // SmartDashboard.putBoolean("shoot button", shootButton.get());
        sneakButton.whenPressed(new InstantCommand(() -> {
            wheelDrive.enableSneak();
        })).whenReleased(new InstantCommand(() -> {
            wheelDrive.disableSneak();
        }));

        shootButton.whenPressed(new InstantCommand(() -> {
            SmartDashboard.putNumber("shoot number", 1);
            intakeSubsystem.disableIntake();
            shooterSubsystem.changeSpeedModifier(inputSubsystem.getJoystickSlider());
            CommandScheduler.getInstance()
                    .schedule(new InstantCommand(() -> shooterSubsystem.startShooter(0.75, 0.75))
                            .andThen(new WaitCommand(0.4).andThen(() -> intakeSubsystem.enablesBelts())
                                    .withInterrupt(() -> shootButton.get() == false)));

        })).whenReleased(new InstantCommand(() -> {
            SmartDashboard.putNumber("shoot number", 2);
            intakeSubsystem.disablesBelts();
            shooterSubsystem.stopShooter();
            intakeSubsystem.enableIntake();
        }));

        climbDownButton.whenPressed(new InstantCommand(() -> {
            climberSubsystem.takeClimberDown(1.0);
        })).whenReleased(new InstantCommand(() -> {
            climberSubsystem.stopClimber();
        }));

        climbUpButton.whenPressed(new InstantCommand(() -> {
            climberSubsystem.takeClimberUp(1.0);
        })).whenReleased(new InstantCommand(() -> {
            climberSubsystem.stopClimber();
        }));

        beltButton.whenPressed(new InstantCommand(() -> {
            intakeSubsystem.enablesBelts();
        })).whenReleased(new InstantCommand(() -> {
            intakeSubsystem.disablesBelts();
        }));

        intakeButton.whenPressed(new InstantCommand(() -> {
            intakeSubsystem.enableIntake();
        })).whenReleased(new InstantCommand(() -> {
            intakeSubsystem.disableIntake();
        }));

        Command foo = new PerpetualCommand(
            new InstantCommand(() -> {
                SmartDashboard.putNumber("distance", visionSubsystem.getSolutionDistance()); 
            }).andThen(new WaitCommand(.05))
        );

        CommandScheduler.getInstance().schedule(foo);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return m_autoCommand;
    }
}
