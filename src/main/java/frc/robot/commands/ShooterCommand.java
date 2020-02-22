/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.InputSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

/**
 * An example command that uses an example subsystem.
 */
public class ShooterCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooter shooterTop;
  private final Shooter shooterBot;
  private final InputSubsystem controller;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShooterCommand(Shooter a, InputSubsystem b, Shooter c) {
    shooterTop = a;
    shooterBot = c;
    controller = b;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterTop, controller, shooterBot); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = 0;
    if (this.controller.getControllerOneButt()) {
      speed = 1;
    } else if (this.controller.getControllerTwoButt()) {
      speed = -1;
    } 
    this.shooterTop.changeSpeed(-speed *0.9);
    this.shooterBot.changeSpeed(speed);
    super.execute();
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
