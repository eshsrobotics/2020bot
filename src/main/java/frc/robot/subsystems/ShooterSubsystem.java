/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMax;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;

import static frc.robot.Constants.*;

/**
 * The ShootSubsystem manages the shooter -- a group of two flywheels that can
 * spin in opposite directions to launch a ball that the belt subsystem has
 * placed behind them.
 */
public class ShooterSubsystem extends SubsystemBase {
  private PWMSpeedController topMotor, bottomMotor;

  public ShooterSubsystem() {
    this.topMotor = new PWMSparkMax(TOP_SHOOTER_FLYWHEEL_PORT);
    this.bottomMotor = new PWMSparkMax(BOTTOM_SHOOTER_FLYWHEEL_PORT);
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
   * @param topSpeed    The velocity of the top flywheel, between 0.0 (0%) and
   *                    1.0 (100%).
   * @param bottomSpeed The velocity of the bottom flywheel, between 0.0 (0%) and
   *                    1.0 (100%).
   */
  public void startShooter(double topSpeed, double bottomSpeed) {

    // Signs for the arguments are ignored.  Otherwise, the caller would be
    // able to reverse the shooter and launch balls back into the belt
    // system.
    topSpeed    = Math.abs(topSpeed);
    bottomSpeed = Math.abs(bottomSpeed);

    // Clip the arguments to be between 0.0 and 1.0.
    topSpeed    = Math.max(0.0, Math.min(1.0, topSpeed));
    bottomSpeed = Math.max(0.0, Math.min(1.0, bottomSpeed));

    this.topMotor.setSpeed(topSpeed);
    this.bottomMotor.setSpeed(-bottomSpeed);
  }

  /**
   * Stops the shooter flywheels.
   */
  public void stopShooter() {
    this.topMotor.stopMotor();
    this.bottomMotor.stopMotor();
  }
}