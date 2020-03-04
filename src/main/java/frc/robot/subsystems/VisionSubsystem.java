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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    double tv, tx, ty, ta, ts, distance;

    /**
     * Setting a parameter value to this variable means sthat the LimeLight
     * provided nothing useful for it.
     */
    static final double NO_USEFUL_VALUE = -1000.0;

    /**
    * Creates a new ExampleSubsystem.
    */
    public VisionSubsystem() {
        tv = tx = ty = ta = ts = NO_USEFUL_VALUE;
        distance = 0;
    }

    /**
    * Grab the critical vision values from the Limelight each frame.
    */
    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

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
    }

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
}
