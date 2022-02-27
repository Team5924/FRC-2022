// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
  private final NetworkTable m_limelightTable;
  private final CANSparkMax m_turretSpark = new CANSparkMax(TurretConstants.TURRET_SPARK, MotorType.kBrushless);
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
    m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    m_turretSpark.restoreFactoryDefaults();
    m_turretSpark.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_turretSpark.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_turretSpark.setSoftLimit(SoftLimitDirection.kForward, 110/360);
    m_turretSpark.setSoftLimit(SoftLimitDirection.kReverse, 110/360);

    m_pidController = m_turretSpark.getPIDController();

    m_encoder = m_turretSpark.getEncoder();

    m_pidController.setP(TurretConstants.P);
    m_pidController.setI(TurretConstants.I);
    m_pidController.setD(TurretConstants.D);
    m_pidController.setIZone(TurretConstants.I_ZONE);
    m_pidController.setFF(TurretConstants.FF);
    m_pidController.setOutputRange(TurretConstants.MIN_OUTPUT, TurretConstants.MAX_OUTPUT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getHorizontalOffset() {
    return m_limelightTable.getEntry("tx").getDouble(0);
  }

  public double getVerticalOffset() {
    return m_limelightTable.getEntry("ty").getDouble(0);
  }

  public boolean isTargetDetected() {
    return m_limelightTable.getEntry("tv").getDouble(0) == 1;
  }

  public double getDistance() {
    // Unit: Meters
    return (TurretConstants.HEIGHT_2 - TurretConstants.HEIGHT_1) / Math.toRadians(Math.tan(TurretConstants.ANGLE_1 + getVerticalOffset()));
  }

  public void target() {
    if (!isTargetDetected()) {

    }
    m_pidController.setReference(degreesToSensorUnits(getVerticalOffset()), ControlType.kPosition);
  }

  private double degreesToSensorUnits(double degrees) {
    return degrees / 360 * 4096;
  }
}
