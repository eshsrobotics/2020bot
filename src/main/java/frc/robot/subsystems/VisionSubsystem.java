// ---------------------------------------------------------------------------- //
// Copyright (c) 2018-2019 FIRST.                                               //
// Copyright (c) 2020 ESHS P.O.T.A.T.O.E.S.                                     //
// All Rights Reserved.                                                         //
// Open Source Software - may be modified and shared by FRC teams. The code     //
// must be accompanied by the FIRST BSD license file in the root directory of   //
// the project.                                                                 //
// ---------------------------------------------------------------------------- //

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class VisionSubsystem extends SubsystemBase {

    double tv, tx, ty, ta, ts, distance;

    boolean enabled;

    final NetworkTable table;

    /**
     * Setting a parameter value to this variable means that the LimeLight
     * provided nothing useful for it.
     */
    static final double NO_USEFUL_VALUE = -1000.0;

    /**
    * Creates a new ExampleSubsystem.
    */
    public VisionSubsystem() {
        tv = tx = ty = ta = ts = NO_USEFUL_VALUE;

        enabled = true;

        distance = 0;
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    /**
    * Grab the critical vision values from the Limelight each frame.
    */
    @Override
    public void periodic() {

        // Whether the limelight has any valid targets (0 or 1)
        tv = table.getEntry("tv").getDouble(0);

        // Horizontal Offset From Crosshair final To Target (-27 final degrees to 27 degrees)
        tx = table.getEntry("tx").getDouble(VisionSubsystem.NO_USEFUL_VALUE);

        // Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
        ty = table.getEntry("ty").getDouble(VisionSubsystem.NO_USEFUL_VALUE);

        // Target Area (0% of image to 100% of image)
        ta = table.getEntry("ta").getDouble(VisionSubsystem.NO_USEFUL_VALUE);

        // Skew or rotation (-90 degrees to 0 degrees)
        ts = table.getEntry("ts").getDouble(VisionSubsystem.NO_USEFUL_VALUE);

        SmartDashboard.putNumber("distance", this.getSolutionDistance());
        //SmartDashboard.putNumber("tx", tx);
        //SmartDashboard.putNumber("tv", tv);
        //SmartDashboard.putNumber("distance-center", ROBOT_HEIGHT_METERS / (Math.tan(.5 * VERTICAL_FOV_RADIANS)));
    }
    
    /**
     * Checks whether or not a valid target is present on screen.
     * @return True if one or more targets are found, false otherwise.
     */
    public boolean solutionFound() {
        if (enabled) {
            return (tv > 0);
        } else {
            return false;
        }
    }

    public void changeLimelightState(boolean newState) {
        this.enabled = newState;
    }

    /**
     * If there is a solution, this number tells us how much we need to rotate to move the center of the Limelight
     * so it aligns with the center of the solution horizontally.
     * 
     * If no solution is found, we return {@link NO_USEFUL_VALUE}.
     * 
     * @return A horizontal angle, in degrees (not radians.)
     */
    public double getSolutionHorizontalDeviationDegrees() {
        return tx;
    }

    /**
     * If there is a solution, this number tells us how much we need to rotate to move the center of the Limelight
     * so it aligns with the center of the solution vertically.
     * 
     * If no solution is found, we return {@link NO_USEFUL_VALUE}.
     * 
     * @return A vertical angle, in degrees (not radians.)
     */
    public double getSolutionVerticalDeviationDegrees() {
        return ty;
    }

    /**
     * Finds distance from the center of the limelight to the target when a target is present (not a true measure of 
     * distance because it works independently of field conditions, but it doesn't need to be accurate -- 
     * it just needs to be consistent and proportional).
     * @return Distance to center of the limelight's vision solution IN INCHES, not pixels.
     */
    public double getSolutionDistance() {
        if (!(solutionFound())) { 
            return -1;
        }
        //final double distanceToScreenCenterMeters = ROBOT_HEIGHT_METERS / (Math.tan(.5 * VERTICAL_FOV_RADIANS));
        
        // Picture the isosceles triangle with its apex at the Limelight, its base
        // distanceToScreenCenterMeters pixels away, and with tx representing the angular
        // deviation, in degrees, from the screen center to the vision target's center.
        //
        // The long sides of this triangle represent the distance to the center of the 
        // vision target.
        //
        // Now, consider the right triangle formed by cutting the isosceles triangle in half.
        // Clearly the hypotenuse of the triangle -- the distance to the target -- is 
        // cos tx / distanceToScreenCenterMeters.
       
        //final double verticalDeviationRadians = ty * (2 * Math.PI / 360);
        //final double distanceToTargetMeters = Math.cos(verticalDeviationRadians) / distanceToScreenCenterMeters;

        double limelightHeight = 21.0;
        double height = 98.25- limelightHeight;
        double limelightBaseAngle = Math.PI/6.0;
        double tempYAngle = Math.toRadians(this.ty);
        double distanceToTargetInches = height / Math.tan(limelightBaseAngle + tempYAngle);
        return distanceToTargetInches;
    }

    //unneeded function?
    public double getMotorPower() {
        //check for target
        if (this.tv != 1) {
            return 0.0;
        }
  
        double distance = this.distance;
        //abritrary distance to power calculations
        double power = distance / 100;

        
        return power;
    }

    public boolean isRobotLinedUp() {
        return (this.tx > 1);
    }
}
