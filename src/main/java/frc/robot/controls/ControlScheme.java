package frc.robot.controls;

import java.util.List;

import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

/**
 * A control scheme converts the user input into swerve module 
 * states (shopping cart directions and speeds). 
 */
public interface ControlScheme {
    List<SwerveModuleState> drive (double x, double y);
    List<SwerveModuleState> turn (double turnSpeed);
}
