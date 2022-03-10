// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
  private final CANSparkMax m_turretSpark = new CANSparkMax(TurretConstants.TURRET_SPARK, MotorType.kBrushless);
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;

  private double reference;

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
    m_turretSpark.restoreFactoryDefaults();
    m_turretSpark.setInverted(true);

    m_pidController = m_turretSpark.getPIDController();

    m_encoder = m_turretSpark.getEncoder();
    // Make encoder output position in degrees instead of rotations
    m_encoder.setPositionConversionFactor(360);

    m_pidController.setP(TurretConstants.P);
    m_pidController.setI(TurretConstants.I);
    m_pidController.setD(TurretConstants.D);
    m_pidController.setIZone(TurretConstants.I_ZONE);
    m_pidController.setFF(TurretConstants.FF);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Turret Centered?", isTurretCentered());
  }

  public void setTurretPosition(double degrees) {
    reference = degrees;
    m_pidController.setReference(reference * TurretConstants.TURRET_GEARBOX_RATIO, ControlType.kPosition);
  }

  public double getPosition() {
    return m_encoder.getPosition() / TurretConstants.TURRET_GEARBOX_RATIO;
  }

  public boolean isTurning() {
    return m_encoder.getVelocity() != 0;
  }

  public boolean isTurretCentered() {
    return Math.abs(getPosition()) <= TurretConstants.ACCEPTABLE_ERROR;
  }

  public void zeroEncoder() {
    m_encoder.setPosition(0);
  }

  public void setEncoder() {
    m_encoder.setPosition(30 * TurretConstants.TURRET_GEARBOX_RATIO);
  }
}
