package frc.robot.controls;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.robot.Constants;

/**
 * This is a simple control scheme that implements the 2020 competition Driving Scheme for testing.
 */
public class CrabDriveScheme implements ControlScheme {

    @Override
    public List<SwerveModuleState> drive(double x, double y) {
        double thetaRadians = Math.atan2(y, x);
        double speed = Math.sqrt(x * x + y * y);
        final double maximumSpeed = Math.sqrt(2);
        final double speedMetersPerSecond = Constants.MAX_ROBOT_SPEED_MPS * speed/maximumSpeed;
        
        var result = new ArrayList<SwerveModuleState>();
        for (int i = 0; i < 4; i++) {
            result.add(new SwerveModuleState(speedMetersPerSecond, new Rotation2d(thetaRadians)));
        }
        return result;
    }

    @Override
    public List<SwerveModuleState> turn(double turnSpeed) {
        
        return null;
    }
    
}
