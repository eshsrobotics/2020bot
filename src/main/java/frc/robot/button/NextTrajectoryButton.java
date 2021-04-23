package frc.robot.button;

import edu.wpi.first.wpilibj2.command.button.Button;

import frc.robot.subsystems.InputSubsystem;

/**
 * Button to move to the next trajectory so the driver can select which trajectory they want to use
 */
public class NextTrajectoryButton extends Button {
    private InputSubsystem inputSubsystem;

    public NextTrajectoryButton(InputSubsystem inputSubsystem) {
        this.inputSubsystem = inputSubsystem;
    }

    /**
     * Returns true if the shoot button is depressed (regardless of input control method),
     * and false otherwise.
     */
    @Override
    public boolean get() {
        return this.inputSubsystem.getNextTrajectoryButton();
    }
}
