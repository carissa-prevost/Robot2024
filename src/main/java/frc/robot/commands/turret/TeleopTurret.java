// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import java.lang.Math;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;

/* Rotate Command for Turret During Teleop */
public class TeleopTurret extends Command {
  /** Creates a new TeleopTurret. */
  private Turret s_Turret;
  private DoubleSupplier m_joystickPosition;

  public TeleopTurret(Turret s_Turret, DoubleSupplier m_joystickPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Turret = s_Turret;
    addRequirements(s_Turret);
    
    this.m_joystickPosition = m_joystickPosition;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
   public void execute() {
    
    // translates the value of a joystick into a form that a motor can accept 
    double j_Position = m_joystickPosition.getAsDouble();
   
    // rotates the turret based on the position of the joystick
    s_Turret.rotateTurret(j_Position);
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}