// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveSubsystem;

public class Taxi extends CommandBase {
  private DriveSubsystem m_drivetrain;

  private double startPos;
  private double currPos;

  /** Creates a new AutoMove. */
  public Taxi(DriveSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = subsystem;
    addRequirements(m_drivetrain);
  }

  // Accepts inches
  public int taxiDistance(int distance) {
    /**
     * 1 rotation = 4 in. traveled
     * 1 rotation = 4096 sensor units/100ms
     * Distance/4 = x rotations.
     * x * 4096 = y sensor units/100ms = taxiDistance
     */
    return (distance / 4) * 4096;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startPos = m_drivetrain.getLeftVelocity();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Auto-driving at half the max speed
    m_drivetrain.tankSquaredDrive(-0.4, -0.4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    currPos = m_drivetrain.getLeftVelocity();
    /**
     * Distance Tables
     * 7 ft. (84 in.)
     * 7.5 ft (90 in.)
     * 8 ft. (96 in.)
     */
    return Math.abs(currPos - startPos) >= taxiDistance(90);
  }
}
