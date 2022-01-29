// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TurretFindLeft extends CommandBase {
  private TurretSubsystem m_turret;
  private LimelightSubsystem m_limelight;
  /** Creates a new TurretFind. */
  public TurretFindLeft(TurretSubsystem subsystem, LimelightSubsystem limelight) {
    m_turret = subsystem;
    m_limelight= limelight;
    addRequirements(m_turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  boolean tv;
  @Override
  public void execute() {
    tv = m_limelight.isTargetDetected();
    m_turret.PIDmove(1024);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_limelight.isTargetDetected();
  }
}
