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
import edu.wpi.first.wpilibj.controller.PIDController;

public class WheelDrive extends SubsystemBase {
    public static SparkMax angleMotor;
    public static SparkMax speedMotor;
    public PIDController pidController; //find sparkmax PID


    public WheelDrive (int angleMotor, int speedMotor, int encoder) {
        this.angleMotor = new SparkMax (angleMotor);
        this.speedMotor = new SparkMax (speedMotor);
        pidController = new PIDController (1, 0, 0, new AnalogInput (encoder), this.angleMotor);
    
        pidController.setInputRange(-1.0, 1.0);
        pidController.setContinuous ();
        pidController.enable ();
    }
    private final double MAX_VOLTS = 12; 

    public void drive (double Speed, double Angle){
        speedMotor.set (speed);

        double setpoint = angle * (MAX_VOLTS * 0.5) + (MAX_VOLTS * 0.5); // Optimization offset can be calculated here.
        if (setpoint < 0) {
            setpoint = MAX_VOLTS + setpoint;
        }
        if (setpoint > MAX_VOLTS) {
            setpoint = setpoint - MAX_VOLTS;
        }
    
        pidController.setSetpoint (setpoint);
    }}

