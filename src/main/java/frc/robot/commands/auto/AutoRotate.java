// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoRotate extends CommandBase {
  private DriveSubsystem m_drivetrain;

  private double startPos;
  private double currPos;

  /** Creates a new AutoRotate. */
  public AutoRotate(DriveSubsystem system) {
    m_drivetrain = system;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startPos = m_drivetrain.getLeftVelocity();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Robot rotates at half the max speed
    m_drivetrain.tankSquaredDrive(0.4, -0.4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // AutoRotate should end once the robot has done a ~180 degree turn
    return false;
  }
}
