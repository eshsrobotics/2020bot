/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.*;

/**
 * The ShootSubsystem manages the shooter -- a group of two flywheels that can
 * spin in opposite directions to launch a ball that the belt subsystem has
 * placed behind them.
 */
public class ShooterSubsystem extends SubsystemBase {
  private CANSparkMax topMotor, bottomMotor;

  private double shooterModifier;

  private double topSpeedOverride = -1;
  private double bottomSpeedOverride = -1;

  public ShooterSubsystem() {
    this.topMotor = new CANSparkMax(TOP_SHOOTER_FLYWHEEL_CAN_ID, MotorType.kBrushless);
    this.bottomMotor = new CANSparkMax(BOTTOM_SHOOTER_FLYWHEEL_CAN_ID, MotorType.kBrushless);

    this.shooterModifier = 1.0;

    this.topMotor.stopMotor();
    this.bottomMotor.stopMotor();
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

    //SmartDashboard.putNumber("topSpeed", topSpeed);
    //SmartDashboard.putNumber("bottomSpeed", bottomSpeed);

    //double top = SmartDashboard.getNumber("topSpeed", topSpeed);
    //double bottom = SmartDashboard.getNumber("bottomSpeed", bottomSpeed);
    double top = topSpeed;
    double bottom = bottomSpeed;
    if (top > 0) {
      // User supplied an override topSpeed.
      topSpeedOverride = top;
    }
    if (bottom > 0) {
      // User supplied an override bottomSpeed.
      bottomSpeedOverride = bottom;
    }

    if (topSpeedOverride > 0) {
      topSpeed = topSpeedOverride;
    }

    if (bottomSpeedOverride > 0){
      bottomSpeed = bottomSpeedOverride;
    }
    // Signs for the arguments are ignored.  Otherwise, the caller would be
    // able to reverse the shooter and launch balls back into the belt
    // system.
    topSpeed    = Math.abs(top);
    bottomSpeed = Math.abs(bottom);

    // Clip the arguments to be between 0.0 and 1.0.
    topSpeed    = Math.max(0.0, Math.min(1.0, topSpeed));
    bottomSpeed = Math.max(0.0, Math.min(1.0, bottomSpeed));

    this.topMotor.set(-topSpeed/*shooterModifier*/);
    this.bottomMotor.set(bottomSpeed/*shooterModifier*/);

    //SmartDashboard.putNumber("topSpeed", top);
    //SmartDashboard.putNumber("bottomSpeed", bottom);


  }

  /**
   * Stops the shooter flywheels.
   */
  public void stopShooter() {
    this.topMotor.stopMotor();
    this.bottomMotor.stopMotor();
  }

  public void changeSpeedModifier(double newSpeed) {
    this.shooterModifier = newSpeed;
  }

  public double[] getShootSpeedsOffVision(double distanceInches) {
    //0 is top, 1 is bot
    double[] speeds = {0,0};

    //top equation
    speeds[0] = (((-2.2187)*Math.pow(10, -8)) * Math.pow(distanceInches, 3)) + ((0.0000259508)*Math.pow(distanceInches, 2)) + ((-0.00844196)*distanceInches) + (1.47272);

    //bot equation
    speeds[1] = ((1.5684*Math.pow(10, -7)) * Math.pow(distanceInches, 3)) + ((-0.000103375)*Math.pow(distanceInches, 2)) + ((0.0205054)*distanceInches) + (-0.450662);
    return speeds;
  }

  @Override
    public void periodic() {
      //this.startShooter(0.5, 0.5);

    }

}
