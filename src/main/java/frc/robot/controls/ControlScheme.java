package frc.robot.controls;

import java.util.List;

import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

/**
 * A control scheme converts the user input into swerve module 
 * states (shopping cart directions and speeds). 
 */
public interface ControlScheme {
    /**
     * Translates horizontal and vertical joystick channel and a turning channel into unified vroom vroom.
     * The way this happens varies between control schemes(ie arcade drive, crab drive, snake drive, etc.)
     * @param x The horizontal channel, -1 to 1.
     * @param y The vertical channel,   -1 to 1.
     * @param turnSpeed -1(maximum turn counter clockwise) to 1(maximum turn clockwise) 0 is no turn. It is dimensionless.
     * @return Shopping cart angles and speeds (robot go vroom)
     */
    List<SwerveModuleState> driveAndTurn (double x, double y, double turnSpeed);

    /**
     * Summarizes the maximum speed that the given joystick vector is outputting.
     * @param x The horizontal channel, -1 to 1.
     * @param y The vertical channel,   -1 to 1.
     * @return Returns a number between 0 and 1.  
     */
    double getMagnitude(double x, double y);
}
