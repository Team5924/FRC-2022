// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Rotate extends CommandBase {
  private final DriveSubsystem m_drive;
  private double degrees;
  // 0 <= speed <= 1
  private double speed;

  private double arcLength;
  private double rotations;
  private double rotationsInSensorUnits;

  private double startPos;
  private double currPos;

  private boolean isClockwise;

  /** Creates a new Rotate. */
  public Rotate(DriveSubsystem driveSubsystem, double degrees, double speed) {
    m_drive = driveSubsystem;
    /**
     * Degrees will be converted into radians
     * input as degrees for human convienience
     */
    this.degrees = degrees;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startPos = m_drive.getLeftVelocity();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /**
     * Given degrees, rotate the robot to the specified angle.
     * Arc length = Degree * 28 in. (radius)
     * Rotations = Arc Length / 4pi (circumference)
     * Rotations in sensor units = Rotations * 4096
     */

    /**
     * This checks if the arcLength is greater than the circumference,
     * if so, it will turn the other way to reduce distance and time
     */
    degrees = degrees * (Math.PI / 180);
    arcLength = degrees * 14;
    if (arcLength <= (14 * Math.PI)) {
      isClockwise = true;
    } else if (arcLength > (14 * Math.PI)) {
      isClockwise = false;
    }

    rotations = arcLength / (4 * Math.PI);
    rotationsInSensorUnits = rotations * 4096;

    if (isClockwise) {
      m_drive.tankDrive(speed, -speed);
    } else {
      m_drive.tankDrive(-speed, speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /** command will finish after the robot has rotated to the specific angle, or
     * explained differently, after the robot has traveled the arc length
     */
    currPos = m_drive.getLeftVelocity();
    return (Math.abs(currPos - startPos) >= rotationsInSensorUnits - 15);
  }
}
