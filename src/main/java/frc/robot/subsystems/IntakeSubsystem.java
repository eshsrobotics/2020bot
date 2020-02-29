package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class IntakeSubsystem extends SubsystemBase {

    private PWMSpeedController intakeMotor;
    private boolean intakeError = false;
    private boolean leftBeltError = false; 
    private boolean rightBeltError = false; 
    private PWMSpeedController leftBeltMotor; 
    private PWMSpeedController rightBeltMotor; 

    /**
     * Intializes this object.
     */
    public IntakeSubsystem () {
        try {
            intakeMotor = new Spark(INTAKE_PORT);
        } catch (Exception e) {
            // Something went wrong during intialization, disable the subsytem.
            intakeError = true;
        }
        try {
            leftBeltMotor = new Spark(LEFT_BELT_PORT);
        } catch (Exception e) {
            // Something went wrong during intialization, disable the subsytem.
            leftBeltError = true;
        }
        try {
            rightBeltMotor = new Spark(RIGHT_BELT_PORT);
        } catch (Exception e) {
            // Something went wrong during intialization, disable the subsytem.
            rightBeltError = true;
        }
    }
    /**
     * Enables intakes.
     */
    public void enableIntake() {
        if (!intakeError) {
            this.intakeMotor.setSpeed(INTAKE_SPEED);
        }
    }

    /**
     * Disables intakes.
     */
    public void disableIntake() {
        if (!intakeError) {
            this.intakeMotor.stopMotor();
        }
    }


    /**
     * Enables both right and left belts. 
     */

    public void enablesBelts() {
        if(!leftBeltError && !rightBeltError) {
            this.leftBeltMotor.set(BELT_SPEED); 
            this.rightBeltMotor.set(BELT_SPEED); 
        }
    }



    /**
     * Disables both right and left belts. 
     */

     public void disablesBelts() {
        if(!leftBeltError && !rightBeltError) {
            this.leftBeltMotor.stopMotor(); 
            this.rightBeltMotor.stopMotor(); 
        }
     }
    }

    