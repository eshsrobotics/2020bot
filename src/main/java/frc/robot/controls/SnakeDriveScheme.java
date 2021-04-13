package frc.robot.controls;

import java.util.List;

import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

public class SnakeDriveScheme implements ControlScheme {
    @Override
    public List<SwerveModuleState> driveAndTurn(double x, double y, double turnSpeed) {
        // TODO Auto-generated method stub
        return null;
    }
    @Override
    public double getMagnitude(double x, double y) {
        return Math.abs(y);
    }
}
