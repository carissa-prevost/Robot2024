// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  CANSparkFlex intakeMotor;
  CANSparkFlex jointMotor;

  SparkPIDController jointPID;


  public Intake() {

    // define + configure the intake motor
    intakeMotor = new CANSparkFlex(Constants.Intake.intakeMotorID, MotorType.kBrushless);
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.setInverted(false);

    // define + configure the motor that deploys + stows the intake
    jointMotor = new CANSparkFlex(Constants.Intake.armID, MotorType.kBrushless);
    jointMotor.restoreFactoryDefaults();
    jointMotor.setIdleMode(IdleMode.kBrake);
    jointMotor.setInverted(false);

    // defines + configures the PID controller for the joint
    jointPID = jointMotor.getPIDController();

    jointPID.setP(Constants.Intake.kP);
    jointPID.setI(Constants.Intake.kI);
    jointPID.setD(Constants.Intake.kD);
    jointPID.setFF(Constants.Intake.kFF);
    jointPID.setOutputRange(Constants.Intake.minOutput, Constants.Intake.maxOutput);

  }

  public void runIntake(double speed){

    intakeMotor.set(speed);

  }

  public void moveArm(double speed){

    jointMotor.set(speed);
    
  }

  public void deployIntake(){

    jointMotor.set(0.5);
    // jointPID.setReference(Constants.Intake.deployedPosition, CANSparkMax.ControlType.kPosition);

  }

  public void stowIntake(){

    jointMotor.set(-0.5);

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
