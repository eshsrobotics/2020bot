package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * A simple Button class that allows you to attach commands.
 */
public class ShootButton extends Button {

    private InputSubsystem inputSubsystem;

    public ShootButton(InputSubsystem inputSubsystem) {
        this.inputSubsystem = inputSubsystem;
    }

    /**
     * Returns true if the shoot button is depressed (regardless of input control method),
     * and false otherwise.
     */
    @Override
    public boolean get() {
        return this.inputSubsystem.getShootButton();
    }

}
