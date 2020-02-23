import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class IntakeSubsystem extends SubsystemBase {

    private PWMSpeedController intakeMotor;
    private boolean error = false;

    /**
     * Intializes this object.
     */
    public IntakeSubsystem () {
        try {
            intakeMotor = new Spark(INTAKE_PORT);
        } catch (Exception e) {
            // Something went wrong during intialization, disable the subsytem.
            error = true;
        }
    }
    /**
     * Enables intakes.
     */
    public void enableIntake() {
        if (!error) {
            this.intakeMotor.setSpeed(INTAKE_SPEED);
        }
    }

    /**
     * Disables intakes.
     */
    public void disableIntake() {
        if (!error) {
            this.intakeMotor.stopMotor();
        }
    }
}
