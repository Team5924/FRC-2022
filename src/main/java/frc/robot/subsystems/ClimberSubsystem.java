// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  CANSparkMax m_climberSpark = new CANSparkMax(ClimberConstants.CLIMBER_SPARK, MotorType.kBrushless);
  RelativeEncoder m_encoder = m_climberSpark.getEncoder();

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber Position", m_encoder.getPosition());
  }

  public void extendClimber() {
    m_climberSpark.set(0.2);
  }

  public void retractClimber() {
    m_climberSpark.set(-0.2);
  }

  public void stopClimber() {
    m_climberSpark.stopMotor();
  }
}
