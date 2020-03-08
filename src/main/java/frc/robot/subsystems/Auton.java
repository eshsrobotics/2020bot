package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * This class is unused.
 */
public class Auton extends Button {

    private InputSubsystem inputSubsystem;

    public Auton(InputSubsystem inputSubsystem) {
        this.inputSubsystem = inputSubsystem;
    }

    /**
     * Returns true if the shoot button is depressed (regardless of input control method),
     * and false otherwise.
     */
    @Override
    public boolean get() {
        return true;
    }
}
