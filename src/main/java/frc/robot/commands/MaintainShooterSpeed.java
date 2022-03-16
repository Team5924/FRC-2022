// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MaintainShooterSpeed extends CommandBase {
  private final ShooterSubsystem m_shooter;

  /** Creates a new MaintainShooterSpeed. */
  public MaintainShooterSpeed(ShooterSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = subsystem;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /*// The minimum speed to score a ball at 6 ft
    m_shooter.setSpeed(0.1);*/
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.runMotor();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
