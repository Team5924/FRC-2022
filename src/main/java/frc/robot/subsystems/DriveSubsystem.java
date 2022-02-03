// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  private final WPI_TalonSRX m_leftFrontTalon = new WPI_TalonSRX(DriveConstants.LEFT_FRONT_TALON);
  private final WPI_TalonSRX m_rightFrontTalon = new WPI_TalonSRX(DriveConstants.RIGHT_FRONT_TALON);
  private final WPI_TalonSRX m_leftBackTalon = new WPI_TalonSRX(DriveConstants.LEFT_BACK_TALON);
  private final WPI_TalonSRX m_rightBackTalon = new WPI_TalonSRX(DriveConstants.RIGHT_BACK_TALON);

  private final MotorControllerGroup m_leftTalons = new MotorControllerGroup(m_leftFrontTalon, m_leftBackTalon);
  private final MotorControllerGroup m_rightTalons = new MotorControllerGroup(m_rightFrontTalon, m_rightBackTalon);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftTalons, m_rightTalons);

  /** Creates a new Drivetrain. */
  public DriveSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_drive.tankDrive(-leftSpeed, rightSpeed, true);
  }
}
