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
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.subsystems.InputSubsystem;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  private double speed;
  private PWMSparkMax speedMotor;
  private InputSubsystem controller;

  public Shooter(double a, int speedMotorPort, InputSubsystem inputSubsystem) {
    this.controller = inputSubsystem;
    this.speed = a;
    this.speedMotor = new PWMSparkMax(speedMotorPort);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    /*if (this.controller.getControllerOneButt()) {
      this.speed = 0.3;
    } else if (this.controller.getControllerTwoButt()) {
      this.speed = -0.3;
    } else {
      this.speed = 0;
    }*/

    this.speedMotor.setSpeed(this.speed);
  }

  public void changeSpeed(double newSpeed) {
    this.speed = newSpeed;
  }
}
