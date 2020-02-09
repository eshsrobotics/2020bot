package frc.robot.subsystems;

/*----------------------------------------------------------------------------*/

/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMax;

import frc.robot.Constants;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.controller.PIDController;

/**
 * This class is designed to take inputs from OI and JoystickInput, and control
 * the driving motors for driving based on those inputs. It will handle all
 * modes of driving, such as CLASSIC and SNAKE.
 */
public class WheelDriveSubsystem extends SubsystemBase {
    public PWMSparkMax angleMotor;
    public PWMSparkMax speedMotor;
    public PIDController pidController;
    private double[] goalThetas = new double[4];

    public WheelDriveSubsystem(int angleMotor, int speedMotor, int encoder) {
        this.angleMotor = new PWMSparkMax(angleMotor);
        this.speedMotor = new PWMSparkMax(speedMotor);
        this.pidController = new PIDController(1, 0, 0);

        pidController.setIntegratorRange(-1.0, 1.0);
        pidController.enableContinuousInput(-1.0, 1.0);
        // pidController.enable();
    }

    private final double MAX_VOLTS = 12;

    public void drive(double Speed, double Angle) {
        speedMotor.set(Speed);

        double setpoint = Angle * (MAX_VOLTS * 0.5) + (MAX_VOLTS * 0.5); // Optimization offset can be calculated here.
        if (setpoint < 0) {
            setpoint = MAX_VOLTS + setpoint;
        }
        if (setpoint > MAX_VOLTS) {
            setpoint = setpoint - MAX_VOLTS;
        }

        pidController.setSetpoint(setpoint);
    }

    public void setGoalAngles(double thetaFL, double thetaBL, double thetaFR, double thetaBR) {
        for (int i=0;i<4;i++) {
            double angle = 0.0;
            switch (i) {
                case 0:
                    angle = thetaFL;
                    break;
                case 1:
                    angle = thetaBL;
                    break;
                case 2:
                    angle = thetaFR;
                    break;
                case 3: 
                    angle = thetaBR;
                    break;
            }
            goalThetas[i] = angle;
            
        }
    }

    /**
     * exists to constantly run with the scheduler
     */
    @Override
    public void periodic() {
        /*
        here we would set all of the wheels to the required angles and speeds based on goalThetas and etc
        */
    }

    /**
     * 
     * @param turningRadius which is in meters.   
     * @return An array of four directional angles.
     * 
     *         All angles are given in radians, in respect to the starting position of the wheels. The starting angle is pi/2 radians. 
     *         To find the absolutel angle of the wheel, it would be pi/2 + the directional angle. 
     *         The convention is that counterclockwise pivot angles are positive, while clockwise pivot angles are negative.
     */
    public static double[] snakeDriveGetAngle(double turningRadius) {
        double angles[] = new double[4];

        double innerTheta = Math.atan(Constants.WHEEL_DRIVE_VERTICAL_WHEEL_TO_CENTER_DISTANCE / (turningRadius - Constants.WHEEL_DRIVE_HORIZONTAL_WHEEL_TO_CENTER_DISTANCE));
        double outerTheta = Math.atan(Constants.WHEEL_DRIVE_VERTICAL_WHEEL_TO_CENTER_DISTANCE / (turningRadius + Constants.WHEEL_DRIVE_HORIZONTAL_WHEEL_TO_CENTER_DISTANCE));

        angles[Constants.FRONT_LEFT] = innerTheta;
        angles[Constants.BACK_LEFT] = -innerTheta;
        angles[Constants.FRONT_RIGHT] = outerTheta;
        angles[Constants.BACK_RIGHT] = -outerTheta;

        return angles;
    }

}
