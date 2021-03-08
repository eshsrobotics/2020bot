package frc.robot.controls;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

import static frc.robot.Constants.*;

/**
 * This is a simple control scheme that implements the 2020 competition Driving Scheme for testing.
 */
public class CrabDriveScheme implements ControlScheme {

    private SwerveDriveKinematics kinematics = null;

    public CrabDriveScheme(SwerveDriveKinematics kinematics) {
        this.kinematics = kinematics;
    }

    @Override
    public List<SwerveModuleState> drive(double x, double y) {
        double thetaRadians = Math.atan2(y, x);
        double speed = Math.sqrt(x * x + y * y);
        final double maximumSpeed = Math.sqrt(2);
        final double speedMetersPerSecond = MAX_ROBOT_SPEED_MPS * speed/maximumSpeed;
        
        var result = new ArrayList<SwerveModuleState>();
        for (int i = 0; i < 4; i++) {
            result.add(new SwerveModuleState(speedMetersPerSecond, new Rotation2d(thetaRadians)));
        }
        return result;
    }

    /**
     * This controls turning for crab rotation. 
     * In this instance, the robot stops translating and points its wheels in a diamond configuration.
     * Reference: docs/snake-rotation.png
     * 
     * @param turnSpeed A floating point number ranging from -1.0 to 1.0.
     *                  It represents a power level for the drive wheels in the crab rotation configuration.
     *                  Positive values turn the robot chassis clockwise. Negative values turn the robot chassis counterclockwise.
     * @return Array of four swerveModuleStates, starting from the front left and proceeding counter-clockwise.
     * TODO: Verify the turn rotation given a positive turnSpeed 
     */
    @Override
    public List<SwerveModuleState> turn(double turnSpeed) {
        return driveAndTurn(0, 0, turnSpeed);
    }
    
    @Override
    public List<SwerveModuleState> driveAndTurn(double x, double y, double turnSpeed) {
        // Biggest challenge is that kinematics depends on actual units
        // Need to convert unitless variables x, y, and turnspeed, use made up constants to do this
        double xMetersPerSecond = x * MAX_ROBOT_SPEED_MPS;
        double yMetersPerSecond = y * MAX_ROBOT_SPEED_MPS;
        final double MAX_ROBOT_TURN_SPEED_RADIANS_PER_SEC = 2 * Math.PI;
        double thetaRadiansPerSecond = turnSpeed * MAX_ROBOT_TURN_SPEED_RADIANS_PER_SEC;
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xMetersPerSecond, yMetersPerSecond, thetaRadiansPerSecond);
    
        SwerveModuleState[] swerveModuleStates = this.kinematics.toSwerveModuleStates(chassisSpeeds);
        ArrayList<SwerveModuleState> swerveModuleStatesList = new ArrayList<SwerveModuleState>();
        Collections.addAll(swerveModuleStatesList, swerveModuleStates);
        return swerveModuleStatesList;  
    }
}
