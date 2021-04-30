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
//import edu.wpi.first.wpilibj.buttons.NetworkButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

/**
 * This is the most heavily used subsystem in the 2020bot codebase. Every
 * command that depends on asking the user what to do requires this subsystem.
 *
 * It is designed as a convenience wrapper for our three supported human input
 * methods: The traditional single joystick, the two-joystick XBox controller
 * family, and keyboard input using networktables-input.jar. Dependent Commands
 * don't have to worry about which channel is assigned to what, which button was
 * hit, or which joystick was plugged in -- that's *our* job.
 */
public class InputSubsystem extends SubsystemBase {

  private Joystick joystick = null;
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
  private NetworkTableEntry driveSneakButtonEntry;

  private NetworkTableEntry shootTopFiveFiveButtonEntry;

  private NetworkTableEntry shootBotFiveFiveButtonEntry;
  private NetworkTableEntry shootBotMaxButtonEntry;

  private NetworkTableEntry nextTrajectoryButtonEntry;
  private NetworkTableEntry previousTrajectoryButtonEntry;

  /**
   * Makes sure that we know, right away, whether a joystick or controller are
   * attached.
   *
   * NOTE: Right now, we always look for joysticks on port 0. That means we do not
   * support having two different HID devices at once -- we could, I suppose, but
   * we didn't really want to bother. Choose one.
   */
  public InputSubsystem() {
    initialize(0);

    NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
    if (networkTableInstance.isConnected()) {
      this.inputTable = networkTableInstance.getTable("inputTable");
      if (this.inputTable != null) {
        // network tables is up
        NetworkTableInstance.getDefault().setUpdateRate(0.0166);

        // These are vector controls, used during both crab and snake mode.
        driveUpButtonEntry = inputTable.getEntry("z");
        driveLeftButtonEntry = inputTable.getEntry("x");
        driveDownButtonEntry = inputTable.getEntry("c");
        driveRightButtonEntry = inputTable.getEntry("v");
        driveSneakButtonEntry = inputTable.getEntry("b");

        // These are only used during crab mode.
        turnLeftButtonEntry = inputTable.getEntry("n");
        turnRightButtonEntry = inputTable.getEntry("m");

        shootTopFiveFiveButtonEntry = inputTable.getEntry("1");

        shootBotMaxButtonEntry = inputTable.getEntry("P");

        nextTrajectoryButtonEntry = inputTable.getEntry("RIGHT");
        previousTrajectoryButtonEntry = inputTable.getEntry("LEFT");
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * This function abstracts away the concept of the "main directional control" --
   * the thing that determines the primary movement of the robot. From the single
   * joystick, this is the joystick itself; for the XBox controller, this is one
   * of the two joysticks; and for the keyboard, this is WASD.
   *
   * @return A vector indicating the direction that the user wants.
   */
  public Vector2d getVector() {
    double xValue = 0.0;
    double yValue = 0.0;

    if (this.joystick != null) {
      xValue = this.joystick.getX();
      yValue = this.joystick.getY();
    } else if (this.controller != null) {
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

      // if (turnLeftButtonEntry.getBoolean(false)) {
      // xValue = -1.0;
      // yValue = 0/0;
      // } else if (turnRightButtonEntry.getBoolean(false)) {
      // xValue = 1.0;
      // yValue = 2654876653658987654533.0*0.0;
      // }
    }

    Vector2d inputVector;
    inputVector = new Vector2d(xValue, yValue);
    return inputVector;
  }

  public double getCrabTurnValue() {
    double xValue = 0.0;

    if (this.joystick != null) {
      xValue = this.joystick.getZ();
    } else if (this.controller != null) {
      xValue = this.controller.getX(Hand.kRight);
    }

    if (NetworkTableInstance.getDefault().isConnected() && this.inputTable != null) {
      // Handle the main direction vector.
      if (turnLeftButtonEntry.getBoolean(false)) {
        xValue = -0.7;
      } else if (turnRightButtonEntry.getBoolean(false)) {
        xValue = 0.7;
      } else {
        xValue = 0;
      }
    }

    return xValue;
  }

  /**
   * Tries to set up a connected joystick _or_ XBox-compatible controller. If the
   * controller or joystick does not exist, we set this.joystick or
   * this.controller to null (respectively.) Both cannot be true at the same time,
   * since they use the same port.
   */
  public void initialize(int port) {
    try {
      this.controller = new XboxController(port);
      SmartDashboard.putString("controller name", this.controller.getName());
      for (int i = 0; i < KNOWN_CONTROLLER_NAMES.length; ++i) {
        if (KNOWN_CONTROLLER_NAMES[i].equals(this.controller.getName())) {
          // This is a known XBox controller.
          this.joystick = null;
          return;
        }
      }

      // This is not a known XBox controller.
      this.controller = null;
      this.joystick = new Joystick(port);

    } catch (Exception e) {

      // Nothing worked. Probably no HID device is attached.
      this.controller = null;
      this.joystick = null;
    }
  }

  /**
   * Returns true if shoot button is held down, false otherwise.
   */
  public boolean getShootButton() {
    if (this.joystick != null) {
      return this.joystick.getTrigger();
    } else if (this.controller != null) {
      return this.controller.getRawButton(CONTROLLER_SHOOT_TRIGGER_BUTTON);
    } else {
      return false;
    }
  }

  public boolean getShootTopFiveFiveButton() {
    if (NetworkTableInstance.getDefault().isConnected() && this.inputTable != null) {

      if (shootTopFiveFiveButtonEntry.getBoolean(false)) {
        return true;
      }
    }
    return false;
  }

  public boolean getShootTopSixZeroButton() {
    if (NetworkTableInstance.getDefault().isConnected() && this.inputTable != null) {

      if (shootTopFiveFiveButtonEntry.getBoolean(false)) {
        return true;
      }
    }
    return false;
  }

  public boolean getShootTopSixFiveButton() {
    if (NetworkTableInstance.getDefault().isConnected() && this.inputTable != null) {

      if (shootTopFiveFiveButtonEntry.getBoolean(false)) {
        return true;
      }
    }
    return false;
  }

  public boolean getShootTopSevenZeroButton() {
    if (NetworkTableInstance.getDefault().isConnected() && this.inputTable != null) {

      if (shootTopFiveFiveButtonEntry.getBoolean(false)) {
        return true;
      }
    }
    return false;
  }

  public boolean getShootTopSevenFiveButton() {
    if (NetworkTableInstance.getDefault().isConnected() && this.inputTable != null) {

      if (shootTopFiveFiveButtonEntry.getBoolean(false)) {
        return true;
      }
    }
    return false;
  }

  public boolean getShootTopEightZeroButton() {
    if (NetworkTableInstance.getDefault().isConnected() && this.inputTable != null) {

      if (shootTopFiveFiveButtonEntry.getBoolean(false)) {
        return true;
      }
    }
    return false;
  }

  public boolean getShootTopEightFiveButton() {
    if (NetworkTableInstance.getDefault().isConnected() && this.inputTable != null) {

      if (shootTopFiveFiveButtonEntry.getBoolean(false)) {
        return true;
      }
    }
    return false;
  }

  public boolean getShootTopNineZeroButton() {
    if (NetworkTableInstance.getDefault().isConnected() && this.inputTable != null) {

      if (shootTopFiveFiveButtonEntry.getBoolean(false)) {
        return true;
      }
    }
    return false;
  }

  public boolean getShootTopNineFiveButton() {
    if (NetworkTableInstance.getDefault().isConnected() && this.inputTable != null) {

      if (shootTopFiveFiveButtonEntry.getBoolean(false)) {
        return true;
      }
    }
    return false;
  }

  public boolean getShootTopMaxButton() {
    if (NetworkTableInstance.getDefault().isConnected() && this.inputTable != null) {

      if (shootTopFiveFiveButtonEntry.getBoolean(false)) {
        return true;
      }
    }
    return false;
  }

  public boolean getShootBotFiveFiveButton() {
    if (NetworkTableInstance.getDefault().isConnected() && this.inputTable != null) {

      if (shootBotFiveFiveButtonEntry.getBoolean(false)) {
        return true;
      }
    }
    return false;
  }

  public boolean getShootBotSixZeroButton() {
    if (NetworkTableInstance.getDefault().isConnected() && this.inputTable != null) {

      if (shootBotFiveFiveButtonEntry.getBoolean(false)) {
        return true;
      }
    }
    return false;
  }

  public boolean getShootBotSixFiveButton() {
    if (NetworkTableInstance.getDefault().isConnected() && this.inputTable != null) {

      if (shootBotFiveFiveButtonEntry.getBoolean(false)) {
        return true;
      }
    }
    return false;
  }

  public boolean getShootBotSevenZeroButton() {
    if (NetworkTableInstance.getDefault().isConnected() && this.inputTable != null) {

      if (shootBotFiveFiveButtonEntry.getBoolean(false)) {
        return true;
      }
    }
    return false;
  }

  public boolean getShootBotSevenFiveButton() {
    if (NetworkTableInstance.getDefault().isConnected() && this.inputTable != null) {

      if (shootBotFiveFiveButtonEntry.getBoolean(false)) {
        return true;
      }
    }
    return false;
  }

  public boolean getShootBotEightZeroButton() {
    if (NetworkTableInstance.getDefault().isConnected() && this.inputTable != null) {

      if (shootBotFiveFiveButtonEntry.getBoolean(false)) {
        return true;
      }
    }
    return false;
  }

  public boolean getShootBotEightFiveButton() {
    if (NetworkTableInstance.getDefault().isConnected() && this.inputTable != null) {
      if (shootBotFiveFiveButtonEntry.getBoolean(false)) {
        return true;
      }
    }
    return false;
  }

  public boolean getShootBotNineZeroButton() {
    if (NetworkTableInstance.getDefault().isConnected() && this.inputTable != null) {

      if (shootBotFiveFiveButtonEntry.getBoolean(false)) {
        return true;
      }
    }
    return false;
  }

  public boolean getShootBotNineFiveButton() {
    if (NetworkTableInstance.getDefault().isConnected() && this.inputTable != null) {

      if (shootBotFiveFiveButtonEntry.getBoolean(false)) {
        return true;
      }
    }
    return false;
  }

  public boolean getShootBotMaxButton() {
    if (NetworkTableInstance.getDefault().isConnected() && this.inputTable != null) {

      if (shootBotMaxButtonEntry.getBoolean(false)) {
        return true;
      }
    }
    return false;
  }

  public boolean getSneakButton() {
    if (this.joystick != null) {
      return this.joystick.getRawButton(7);
    } else if (this.controller != null) {
      return this.controller.getRawButton(7);
    }
    if (NetworkTableInstance.getDefault().isConnected() && this.inputTable != null) {
      // Handle the main direction vector.
      if (driveSneakButtonEntry.getBoolean(false)) {
        return true;
      }
    }
    return false;
  }

  public boolean getLimeLightDisableButton() {
    if (this.joystick != null) {
      return this.joystick.getRawButton(10);
    } else if (this.controller != null) {
      return this.controller.getRawButton(10);
    }
    if (NetworkTableInstance.getDefault().isConnected() && this.inputTable != null) {
      // Handle the main direction vector.
      if (driveSneakButtonEntry.getBoolean(false)) {
        return true;
      }
    }
    return false;
  }

  public boolean getLimeLightEnableButton() {
    if (this.joystick != null) {
      return this.joystick.getRawButton(9);
    } else if (this.controller != null) {
      return this.controller.getRawButton(9);
    }
    if (NetworkTableInstance.getDefault().isConnected() && this.inputTable != null) {
      // Handle the main direction vector.
      if (driveSneakButtonEntry.getBoolean(false)) {
        return true;
      }
    }
    return false;
  }

  public boolean getClimbUpButton() {
    if (joystick != null) {
      return this.joystick.getRawButton(6);
    } else if (controller != null) {
      SmartDashboard.putBoolean("button 6", this.controller.getRawButton(7));
      return this.controller.getRawButton(6);
    } else {
      return false;
    }
  }

  public boolean getClimbDownButton() {
    if (joystick != null) {
      return this.joystick.getRawButton(5);
    } else if (controller != null) {
      SmartDashboard.putBoolean("button 5", this.controller.getRawButton(5));
      return this.controller.getRawButton(5);
    } else {
      return false;
    }
  }

  public double getJoystickSlider() {
    if (joystick != null) {
      SmartDashboard.putNumber("shooter speed", this.joystick.getRawAxis(3));
      return this.joystick.getRawAxis(3);
    } else {
      return 0.0;
    }
  }

  public boolean getBeltButton() {
    if (joystick != null) {
      return this.joystick.getRawButton(3);
    } else if (controller != null) {
      return this.controller.getRawButton(3);
    } else {
      return false;
    }
  }

  public boolean getReverseBeltButton() {
    if (joystick != null) {
      return this.joystick.getRawButton(4);
    } else if (controller != null) {
      return this.controller.getRawButton(4);
    } else {
      return false;
    }
  }

  /**
   * This function allows use of button 6 for testing commands. It is not intended
   * to be used during actual driving.
   * 
   * @return True if the buttin is pressed, false if not.
   */
  public boolean getTestButton() {
    if (joystick != null) {
      SmartDashboard.putBoolean("testButton joy", this.controller.getRawButton(1));
      return this.joystick.getRawButton(1);

    } else if (controller != null) {
      SmartDashboard.putBoolean("testButton", this.controller.getRawButton(1));
      return this.controller.getRawButton(1);
    } else {
      return false;
    }
  }

  public boolean getTestButton2() {
    if (joystick != null) {
      SmartDashboard.putBoolean("button 6 joy", this.controller.getRawButton(6));
      return this.joystick.getRawButton(2);

    } else if (controller != null) {
      SmartDashboard.putNumber("button 6", this.controller.getRawButton(6) ? 1 : 0);
      return this.controller.getRawButton(2);
    } else {
      return false;
    }
  }

  public boolean getClockwiseButton() {
    if (joystick != null) {
      return this.joystick.getRawButton(1);
    } else if (controller != null) {
      return this.controller.getRawButton(1);
    } else {
      return false;
    }
  }

  public boolean getCounterclockwiseButton() {
    if (joystick != null) {
      return this.joystick.getRawButton(2);
    } else if (controller != null) {
      return this.controller.getRawButton(2);
    } else {
      return false;
    }
  }

  /**
   * Alerts when user presses a button that corresponds to Next Trajectory. 
   * @return The button the user pressed (Joystick, controller, keyboard) if it corresponds to Next Trajectory
   */
  public boolean getNextTrajectoryButton() {
    if (joystick != null) {
      // The button is the small button to the top right of the joystick.
      return joystick.getRawButton(6);
    } else if (controller != null) {
      // it is the options button (small button to the left of the Letter buttons).
      return controller.getRawButton(8);
    } else {
      // Keyboard control.
      return nextTrajectoryButtonEntry.getBoolean(false);
    }
  }

  /**
   * Determines if the user wants to navigate previous trajectory or not 
   * @return Returns true if the user is pressing the relevant button, using three different control
   * set-ups (X-box, big joystick, & a keyboard).
   */

  public boolean getPreviousTrajectoryButton() {
    if (joystick != null) {
      // The button is the small button to the top left of the joystick.
      return joystick.getRawButton(5);
    } else if (controller != null) {
      // This is the share button (small button to the right of the arrow pad).
      return controller.getRawButton(7);
    } else {
      // Keyboard control.
      return previousTrajectoryButtonEntry.getBoolean(false);
    }   
  }
}