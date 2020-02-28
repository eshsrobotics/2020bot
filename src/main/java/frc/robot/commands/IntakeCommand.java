/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.InputSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class IntakeCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem  intake;
  private final InputSubsystem controller;

  /**
   * Creates a new IntakeCommand .
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeCommand (IntakeSubsystem  subsystem, InputSubsystem inputSubsystem) {
    intake = subsystem;
    controller = inputSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem, inputSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (this.controller.getControllerThreeButt()) {
      this.intake.enableIntake();
    } else {
      this.intake.disableIntake();
    }
    if (this.controller.getControllerFourButt()) {
      this.intake.enablesBelts();
    } else {
      this.intake.disablesBelts();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
