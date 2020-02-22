/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.buttons.NetworkButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

/**
 * This is the most heavily used subsystem in the 2020bot codebase.  Every
 * command that depends on asking the user what to do requires this
 * subsystem.
 *
 * It is designed as a convenience wrapper for our three supported human input
 * methods: The traditional single joystick, the two-joystick XBox controller
 * family, and keyboard input using networktables-input.jar.  Dependent
 * Commands don't have to worry about which channel is assigned to what, which
 * button was hit, or which joystick was plugged in -- that's *our* job.
 */
public class InputSubsystem extends SubsystemBase {

  private boolean joystickAttached = false;
  private Joystick joystick = null;

  private boolean controllerAttached = false;
  private XboxController controller = null;

  private NetworkTable inputTable;

  // The edu.wpi.first.wpilibj.buttons.NetworkButton class seems to be missing
  // as of the WPILib 2020.2.2 release.
  //
  // We'll work around it by using NetworkTableEntry objects for the same
  // NetworkTable...with the same name...and hoping that we can pull booleans
  // out of them.
  private NetworkTableEntry driveUpButtonEntry;
  private NetworkTableEntry driveLeftButtonEntry;
  private NetworkTableEntry driveDownButtonEntry;
  private NetworkTableEntry driveRightButtonEntry;
  private NetworkTableEntry turnLeftButtonEntry;
  private NetworkTableEntry turnRightButtonEntry;

  /**
   * Makes sure that we know, right away, whether a joystick or controller are
   * attached.
   *
   * NOTE: Right now, we always look for joysticks on port 0.  That means we
   * do not support having two different HID devices at once -- we could, I
   * suppose, but we didn't really want to bother.  Choose one.
   */
  public InputSubsystem() {
    initializeJoystick(0);
    initializeController(0);

    NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
    if (networkTableInstance.isConnected()) {
      this.inputTable = networkTableInstance.getTable("inputTable");
      if (this.inputTable != null) {
        // network tables is up
        NetworkTableInstance.getDefault().setUpdateRate(0.0166);

        // These are vector controls, used during both crab and snake mode.
        driveUpButtonEntry = inputTable.getEntry(DRIVE_VECTOR_UP_BUTTON);
        driveLeftButtonEntry = inputTable.getEntry(DRIVE_VECTOR_LEFT_BUTTON);
        driveDownButtonEntry = inputTable.getEntry(DRIVE_VECTOR_DOWN_BUTTON);
        driveRightButtonEntry = inputTable.getEntry(DRIVE_VECTOR_RIGHT_BUTTON);

        // These are only used during crab mode.
        turnLeftButtonEntry = inputTable.getEntry(DRIVE_AUXILIARY_LEFT_TURN_BUTTON);
        turnRightButtonEntry = inputTable.getEntry(DRIVE_AUXILIARY_RIGHT_TURN_BUTTON);
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * This function abstracts away the concept of the "main directional
   * control" -- the thing that determines the primary movement of the robot.
   * Fro the single joystick, this is the joystick itself; for the XBox
   * controller, this is one of the two joysticks; and for the keyboard, this
   * is WASD.
   *
   * @return A vector indicating the direction that the user wants.
   */
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

    if (NetworkTableInstance.getDefault().isConnected() && this.inputTable != null) {
      // Handle the main direction vector.
      if (driveUpButtonEntry.getBoolean(false)) {
        yValue = -1.0;
      } else if (driveDownButtonEntry.getBoolean(false)) {
        yValue = 1.0;
      }
      if (driveLeftButtonEntry.getBoolean(false)) {
        xValue = -1.0;
      } else if (driveRightButtonEntry.getBoolean(false)) {
        xValue = 1.0;
      }
    }

    Vector2d inputVector;
    inputVector = new Vector2d(xValue, yValue);
    return inputVector;
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

  public boolean getControllerOneButt() {
    boolean x = this.controller.getRawButton(1);
    return x;
  }
  public boolean getControllerTwoButt() {
    boolean x = this.controller.getRawButton(2);
    return x;
  }
  public boolean getControllerThreeButt() {
    boolean x = this.controller.getRawButton(3);
    return x;
  }
  public boolean getControllerFourButt() {
    boolean x = this.controller.getRawButton(4);
    return x;
  }
  public boolean getControllerFiveButt() {
    boolean x = this.controller.getRawButton(5);
    return x;
  }
  public boolean getControllerSixButt() {
    boolean x = this.controller.getRawButton(6);
    return x;
  }
  public boolean getControllerSevenButt() {
    boolean x = this.controller.getRawButton(7);
    return x;
  }
  public boolean getControllerEightButt() {
    boolean x = this.controller.getRawButton(8);
    return x;
  }
}
