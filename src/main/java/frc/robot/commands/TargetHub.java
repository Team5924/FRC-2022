// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TargetHub extends CommandBase {
  private final LimelightSubsystem m_limelight;
  private final TurretSubsystem m_turret;
  private int attemptsToFindTarget = 0;
  private boolean targetDetected = false;
  private boolean endCommand = false;

  /** Creates a new TargetHub. */
  public TargetHub(LimelightSubsystem limelightSubsystem, TurretSubsystem turretSubsystem) {
    m_limelight = limelightSubsystem;
    m_turret = turretSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (targetDetected) {
      attemptsToFindTarget = 0;
      if (!m_turret.isTurning()) {
        m_turret.turnTurret(m_limelight.getHorizontalOffset() * TurretConstants.TURRET_GEARBOX_RATIO);
      }
    } else {
      if (attemptsToFindTarget < 10) {
        if (m_limelight.isTargetDetected()) {
          targetDetected = true;
        } else {
          attemptsToFindTarget++;
        }
      } else {
        endCommand = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (endCommand) {
      return true;
    } else if (m_limelight.isTargetDetected() && m_limelight.getHorizontalOffset() <= TurretConstants.ACCEPTABLE_ERROR) {
      return true;
    } else {
      return false;
    }
  }
}
