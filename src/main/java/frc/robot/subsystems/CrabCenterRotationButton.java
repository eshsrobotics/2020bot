package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * A simple Button class that allows you to attach commands to the act of
 * shooting via Button utility functions like <a
 * href="https://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj2/command/button/Button.html#whenPressed(edu.wpi.first.wpilibj2.command.Command)">whenPressed</a>
 * and <a
 * href="https://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj2/command/button/Button.html#whenReleased(edu.wpi.first.wpilibj2.command.Command)">whenReleased</a>.
 *
 * Ideally, we'd like to be able to handle this without making yet another
 * tiny, one-shot class, but we're not quite sure how yet.  Instant commands
 * are easier to piece together than instant buttons.
 */
public class CrabCenterRotationButton extends Button {

    private InputSubsystem inputSubsystem;

    public CrabCenterRotationButton(InputSubsystem inputSubsystem) {
        this.inputSubsystem = inputSubsystem;
    }

    /**
     * Returns true if the shoot button is depressed (regardless of input control method),
     * and false otherwise.
     */
    @Override
    public boolean get() {
        //return (this.inputSubsystem.getCrabTurnValue() > Constants.JOYSTICK_EPSILON);
        return this.inputSubsystem.getTestButton();
    }
}
