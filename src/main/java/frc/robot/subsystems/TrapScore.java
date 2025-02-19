// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class TrapScore extends SubsystemBase {
  /** Creates a new TrapScore. */

  CANSparkFlex trapArm;

  public TrapScore() {

    trapArm = new CANSparkFlex(Constants.TrapScorer.armMotorID, MotorType.kBrushless);
    trapArm.restoreFactoryDefaults();
    trapArm.setIdleMode(IdleMode.kBrake);

  }

  public void setSpeedZero (){

    trapArm.set(0);

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
