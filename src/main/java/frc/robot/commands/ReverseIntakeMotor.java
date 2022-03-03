// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class ReverseIntakeMotor extends CommandBase {
  private final IntakeSubsystem m_intake;
  private boolean wasRunning;

  /** Creates a new ReverseIntakeMotor. */
  public ReverseIntakeMotor(IntakeSubsystem subsystem) {
    m_intake = subsystem;
    addRequirements(m_intake);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wasRunning = m_intake.isIntakeMotorRunning();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.reverseMotor();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (wasRunning) {
      m_intake.enableIntakeMotor();
    } else {
      m_intake.disableIntakeMotor();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
