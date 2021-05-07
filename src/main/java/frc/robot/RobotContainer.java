/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Constants.DEVIATION_EPSILON_DEGREES;
import static frc.robot.Constants.crabRotationThetas;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.button.NextTrajectoryButton;
import frc.robot.button.PreviousTrajectoryButton;
import frc.robot.commands.AutoTimedDrive;
import frc.robot.subsystems.BeltButton;
import frc.robot.subsystems.ClimbDownButton;
import frc.robot.subsystems.ClimbUpButton;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.InputSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightDisableButton;
import frc.robot.subsystems.LimeLightEnableButton;
import frc.robot.subsystems.NewWheelDriveSubsystem;
import frc.robot.subsystems.ReverseBeltsButton;
import frc.robot.subsystems.ShootButton;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SneakButton;
import frc.robot.subsystems.TestButton;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.WheelDriveSubsystem;

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
    // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

    private final WheelDriveSubsystem wheelDrive = null;    
    private final InputSubsystem inputSubsystem = new InputSubsystem();
    private final NewWheelDriveSubsystem newWheelDrive = new NewWheelDriveSubsystem(inputSubsystem);
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    private final VisionSubsystem limelight = new VisionSubsystem();

    private AutoTimedDrive m_autoCommand = null;

    private final TestButton testButton = new TestButton(inputSubsystem);
    private final ShootButton shootButton = new ShootButton(inputSubsystem);
    private final ClimbUpButton climbUpButton = new ClimbUpButton(inputSubsystem);
    private final ClimbDownButton climbDownButton = new ClimbDownButton(inputSubsystem);
    private final SneakButton sneakButton = new SneakButton(inputSubsystem);
    private final BeltButton beltButton = new BeltButton(inputSubsystem);
    private final ReverseBeltsButton intakeButton = new ReverseBeltsButton(inputSubsystem);
    private final NextTrajectoryButton nextTrajectoryButton = new NextTrajectoryButton(inputSubsystem);
    private final PreviousTrajectoryButton previousTrajectoryButton = new PreviousTrajectoryButton(inputSubsystem);
    // private final Auton auton = new Auton(inputSubsystem);
    private final LimeLightDisableButton limeLightDisableButton = new LimeLightDisableButton(inputSubsystem);
    private final LimeLightEnableButton limeLightEnableButton = new LimeLightEnableButton(inputSubsystem);

    private double topSpeed;
    private double botSpeed;

    /**
     * This is a special vision tracking command used during both teleop and autonomous.
     *
     * - It causes the robot to swerve into crab rotation mode, then sets the speed
     *   of the wheels based on how horizontally distance the vision target is.
     * - Once it is sufficiently aligned with the vision target, it activates the
     *   belt and flywheels and shoots at a power proportional to the calculated
     *   distance.
     * - It instantly cancels if it can't see a vision solution.
     *
     * Autonomous schedules this command once it is done driving foward.
     *
     * Teleop only schedules this command when the user presses a tracking button,
     * and cancels the command if the user releases the button.
    */
    private Command visionTrackingCommand;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        // Configure the button bindings.
        configureButtonBindings();

        // The following command runs at all times, allowing the user to control the
        // wheelDrive.
        // this.wheelDrive.setDefaultCommand(new CrabDriveModeCommand(wheelDrive, inputSubsystem));

        // The vision tracking command is complex.  Compose it in bits and pieces.
        Command prepareToRotate = new InstantCommand(() -> {
            // Stop driving.
            double[] speeds = { 0, 0, 0, 0 };
            this.wheelDrive.setDriveSpeeds(speeds);

            // Enter crabRotationMode.
            double[] goalThetas = crabRotationThetas;
            this.wheelDrive.setGoalAngles(goalThetas);

        }).andThen(new WaitCommand(0.1)); // Wait for wheels to align.  It's quick.

        Command waitUntilTargetIsClose = new WaitUntilCommand(() -> {
            // Command 1 of the parallel race: does nothing, but quits as
            // soon as the vision target is close enough.
            double horizontalDeviation = limelight.getSolutionHorizontalDeviationDegrees();
            if (Math.abs(horizontalDeviation) <= DEVIATION_EPSILON_DEGREES) {
                // We think we've hit our target.
                return true;
            } else {
                // We still need to find our target.
                return false;
            }
        });

        // Command rotateTowardTarget = new RunCommand(() -> {
        //     // Command 2 of the parallel race: Look at the vision target.  If it
        //     // exists, rotate left or right based on the horizontal angle of deviation.
        //     //
        //     // Assumption: this corresponds to the execute() for the RunCommand.
        //     double horizontalDeviation = limelight.getSolutionHorizontalDeviationDegrees();

        //     // Convert our horizontal deviation to a number between 1 and 0, where
        //     // 0 is far left and 1 is far right.
        //     final double u = (horizontalDeviation + MAX_HORIZONTAL_DEVIATION_DEGREES) /
        //         (2 * MAX_HORIZONTAL_DEVIATION_DEGREES);

        //     // Converting u to a value which varies from +1 on the left side to -1 on
        //     // the right side, where 0 is the middle.
        //     final double v = -2 * (u - .5);

        //     // Convert v into a speed that will rotate us toward the center.
        //     //
        //     // - When the visual target is leftmost (v == -1), our speed needs to
        //     //   be hard right (+MAX_SPEED.)
        //     // - When the visual target is at the center (v == 0), our speed needs
        //     //   to be minimal (MIN_SPEED.)
        //     // - When the visual target is rightmost (v = +1), our speed needs to
        //     //   be hard to the left (-MAX_SPEED.)
        //     final double MIN_SPEED = 0.07;
        //     final double MAX_SPEED = 0.6;
        //     double speed;
        //     if (v <= 0) {
        //         speed = MIN_SPEED + v * (MIN_SPEED - MAX_SPEED);
        //     } else {
        //         speed = -(MIN_SPEED + v * (MAX_SPEED - MIN_SPEED));
        //     }

        //     this.wheelDrive.setDriveSpeeds(new double[] { speed, speed, speed, speed });

        // }, this.wheelDrive, this.limelight);

        // Command stopRotatingAndShoot = new InstantCommand(() -> {
        //     // Stop rotating.
        //     this.wheelDrive.setDriveSpeeds(new double[] { 0, 0, 0, 0 });

        //     // Activate the flywheels at speeds appropriate for the calculated distance.
        //     final double solutionDistanceInches = limelight.getSolutionDistance();
        //     final double[] flywheelSpeeds = shooterSubsystem.getShootSpeedsOffVision(solutionDistanceInches);
        //     final double topFlywheelSpeed = flywheelSpeeds[0];
        //     final double bottomFlywheelSpeed = flywheelSpeeds[1];
        //     shooterSubsystem.startShooter(topFlywheelSpeed, bottomFlywheelSpeed);
        // })
        // .andThen(new WaitCommand(0.2))      // Wait for flywheels to reach max speed.
        // .andThen(new InstantCommand(() -> {
        //     intakeSubsystem.enablesBelts(); // Open fire.
        // }))
        // .andThen(new WaitCommand(5.0))      // We assume it takes this much time to unload the shooter.
        // .andThen(new InstantCommand(() -> { // Deactivate shooter.
        //     intakeSubsystem.disablesBelts();
        //     shooterSubsystem.stopShooter();
        // }));

        // // With all the subcommands prepared, the vision tracking command is
        // // simple to define.
        // this.visionTrackingCommand = prepareToRotate
        // .andThen(
        //     // These two commands race with each other -- they run in parallel until
        //     // either one of them finishes.
        //     waitUntilTargetIsClose.raceWith(rotateTowardTarget))
        // .andThen(stopRotatingAndShoot)
        // .withInterrupt(() -> {
        //     // This cancels tracking if the vision solution is lost.
        //     return !limelight.solutionFound();
        // });

        // // Construct the command we'll use for autonomous.
        // final double SECONDS_TO_MOVE_FORWARD = 1.0;
        // m_autoCommand = new AutoTimedDrive(wheelDrive,
        //                                    intakeSubsystem,
        //                                    shooterSubsystem,
        //                                    this.visionTrackingCommand,
        //                                    SECONDS_TO_MOVE_FORWARD);

    }

    /**
     * Reset the pivot wheels to the initial pivot positions when the robot is
     * disabled.
     */
    public void resetDrive() {
        this.newWheelDrive.resetPivotPositions();
    }

    /**
     * Gets a composite {@link Command} that:
     *
     * 1. Stops moving and aligns the wheels for crab rotation mode
     * 2. Rotates until the vision solution is roughly centered
     * 3. Stops rotating and shoots
     * 4. Gives up and self-cancels if the vision solution is lost
     */
    public Command getVisionTrackingCommand() {
        return this.visionTrackingCommand;
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
            newWheelDrive.enableSneak();
        })).whenReleased(new InstantCommand(() -> {
            newWheelDrive.disableSneak();
        }));

        shootButton.whenPressed(new InstantCommand(() -> {
            wheelDrive.setGoalAngles(crabRotationThetas);
            intakeSubsystem.disableIntake();
            shooterSubsystem.changeSpeedModifier(inputSubsystem.getJoystickSlider());
            double shootDistance = limelight.getSolutionDistance();

            if (shootDistance > 0) {
                topSpeed = shooterSubsystem.getShootSpeedsOffVision(shootDistance)[0];
                botSpeed = shooterSubsystem.getShootSpeedsOffVision(shootDistance)[1];
            } else {
                topSpeed = 0.75;
                botSpeed = 0.75;
            }
            CommandScheduler.getInstance()
                    .schedule(new InstantCommand(() -> shooterSubsystem.startShooter(topSpeed, botSpeed))
                            .andThen(new WaitCommand(0.4).andThen(() -> intakeSubsystem.enablesBelts())
                                    .withInterrupt(() -> shootButton.get() == false)));

        })).whenReleased(new InstantCommand(() -> {
            //SmartDashboard.putNumber("shoot number", 2);
            intakeSubsystem.disablesBelts();
            shooterSubsystem.stopShooter();
            //intakeSubsystem.enableIntake();
        }));

        climbDownButton.whenPressed(new InstantCommand(() -> {
            climberSubsystem.takeClimberDown(1.0);
        })).whenReleased(new InstantCommand(() -> {
            climberSubsystem.stopClimber();
        }));

        limeLightDisableButton.whenPressed(new InstantCommand(() -> {
            limelight.changeLimelightState(false);
        })).whenReleased(new InstantCommand(() -> {
        }));

        limeLightEnableButton.whenPressed(new InstantCommand(() -> {
            limelight.changeLimelightState(true);
        })).whenReleased(new InstantCommand(() -> {
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
            intakeSubsystem.enableReverseBelts();
        })).whenReleased(new InstantCommand(() -> {
            intakeSubsystem.disablesBelts();
        }));

        // // When this button is held down, if the vision tracking command ends
        // // (due to losing the vision target or emptying its volley), restart
        // // it.
        // //
        // // When the button is released, the command is canceled and normal
        // // control resumes.
        // limeLightEnableButton.whileActiveOnce(visionTrackingCommand
        // .andThen(new InstantCommand(() -> {
        //     // If the vision tracking command ends, and the button is still held down,
        //     // then simply schedule it again.
        //     CommandScheduler.getInstance().schedule(visionTrackingCommand);
        // })));

        // testButton.whenPressed(new InstantCommand(wheelDrive::setOppositeAngle));
        
        testButton.whenPressed(() -> {
            this.resetDrive();
        }, this.newWheelDrive);

        nextTrajectoryButton.whenPressed(() -> {
            this.newWheelDrive.nextTrajectory();
        }, this.newWheelDrive);

        previousTrajectoryButton.whenPressed(() -> {
            this.newWheelDrive.previousTrajectory();
        }, this.newWheelDrive);
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