/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class InputSubsystem extends SubsystemBase {

  private boolean joystickAttached = false;
  private Joystick joystick = null;

  private boolean controllerAttached = false;
  private XboxController controller = null;
  /**
   * Creates a new ExampleSubsystem.
   */
  public InputSubsystem() {
    initializeJoystick(0);
    initializeController(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Vector2d getVector() {
    double xValue = 0.0;
    double yValue = 0.0;

    if (this.joystickAttached) {
      xValue = this.joystick.getX();
      yValue = this.joystick.getY();
    } else if (this.controllerAttached) {
      xValue = this.controller.getX(Hand.kLeft);
      yValue = this.controller.getY(Hand.kLeft);
    }

    if (true /*check if network tables is up*/) {
      boolean buttonW = false, buttonA = false, buttonS = false, buttonD = false;
      if (buttonW) {

      }
      if (buttonA) {
        
      }
      if (buttonS) {
        
      }
      if (buttonD) {
        
      }
    }

    Vector2d joystickVector;
    joystickVector = new Vector2d(xValue, yValue);
    return joystickVector;
  }

  /**
   * Tries to set up a connected joystick, and subsequently checks if that joystick exists. If it does exist,
   * then a joystick object is set up. If it is not connected, then the boolean joystickAttached is set to false.
   */
  public void initializeJoystick(int port) {
    try {
      this.joystick = new Joystick(port);
      this.joystickAttached = true;
    } catch (Exception e) {
      this.joystickAttached = false;
      this.joystick = null;
    } 
  }
  
  public void initializeController(int port) {
    try {
      this.controller = new XboxController(port);
      this.controllerAttached = true;
    } catch (Exception e) {
      this.controllerAttached = false;
      this.controller = null;
    } 
  }
}
