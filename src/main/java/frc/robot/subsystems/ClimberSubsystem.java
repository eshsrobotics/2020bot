/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Spark;

import static frc.robot.Constants.*;

/**
 * The ShootSubsystem manages the shooter -- a group of two flywheels that can
 * spin in opposite directions to launch a ball that the belt subsystem has
 * placed behind them.
 */
public class ClimberSubsystem extends SubsystemBase {
  private Spark rightMotor, leftMotor;

  public ClimberSubsystem() {
    this.rightMotor = new Spark(RIGHT_CLIMBER_MOTOR_PWM);
    this.leftMotor = new Spark(LEFT_CLIMBER_MOTOR_PWM);

    this.rightMotor.stopMotor();
    this.leftMotor.stopMotor();
  }

  /**
   * Enables the shooter with the given flywheel speeds.  You can also call
   * this function to change the shooter speed if it's already running.
   *
   * This function takes care of making sure the shooters run in the proper
   * direction itself, so the sign of the arguments is ignored.
   *
   * There is a natural (albeit short) ramping-up period for the flywheels to
   * attain the desired speed; any commands using this subsystem should take
   * that into account be introducing a brief, sequential wait after
   * activating the shooter before releasing the belt payload.
   *
   * @param speed    The velocity of the wheels, between 0.0 (0%) and
   *                    1.0 (100%).
   */
  public void takeClimberDown(double speed) {

    // Signs for the arguments are ignored.  Otherwise, the caller would be
    // able to reverse the shooter and launch balls back into the belt
    // system.
    speed = Math.abs(speed);

    // Clip the arguments to be between 0.0 and 1.0.
    speed = Math.max(0.0, Math.min(1.0, speed));

    speed *= -1;

    this.rightMotor.set(-speed);
    this.leftMotor.set(speed);
  }

  public void takeClimberUp(double speed) {

    // Signs for the arguments are ignored.  Otherwise, the caller would be
    // able to reverse the shooter and launch balls back into the belt
    // system.
    speed = Math.abs(speed);

    // Clip the arguments to be between 0.0 and 1.0.
    speed = Math.max(0.0, Math.min(1.0, speed));

    this.rightMotor.set(-speed);
    this.leftMotor.set(speed);
  }

  /**
   * Stops the shooter flywheels.
   */
  public void stopClimber() {
    this.rightMotor.stopMotor();
    this.leftMotor.stopMotor();
  }

  @Override
    public void periodic() {
      //this.startShooter(0.5, 0.5);

    }

}
