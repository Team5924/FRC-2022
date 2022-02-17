// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  private final WPI_TalonFX m_leftMaster = new WPI_TalonFX(DriveConstants.LEFT_FRONT_TALON);
  private final WPI_TalonFX m_rightMaster = new WPI_TalonFX(DriveConstants.RIGHT_FRONT_TALON);
  private final WPI_TalonFX m_leftSlave = new WPI_TalonFX(DriveConstants.LEFT_BACK_TALON);
  private final WPI_TalonFX m_rightSlave = new WPI_TalonFX(DriveConstants.RIGHT_BACK_TALON);

  /** Creates a new Drivetrain. */
  public DriveSubsystem() {
    m_leftSlave.follow(m_leftMaster);
    m_rightSlave.follow(m_rightMaster);
    configLeftTalon(m_leftMaster);
    configRightTalon(m_rightMaster);
  }

  private void configLeftTalon(WPI_TalonFX motorController) {
    		/* Factory Default all hardware to prevent unexpected behaviour */
		motorController.configFactoryDefault();

		/* Config neutral deadband to be the smallest possible */
		motorController.configNeutralDeadband(0.001, DriveConstants.TIMEOUT_MS);

		/* Config sensor used for Primary PID [Velocity] */
    motorController.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, DriveConstants.PID_LOOP_IDX, DriveConstants.TIMEOUT_MS);

    motorController.configAllowableClosedloopError(0, 1, DriveConstants.TIMEOUT_MS);

    // Don't invert left side, this is default
    motorController.setInverted(TalonFXInvertType.CounterClockwise);

		/* Config the peak and nominal outputs */
		motorController.configNominalOutputForward(0, DriveConstants.TIMEOUT_MS);
		motorController.configNominalOutputReverse(0, DriveConstants.TIMEOUT_MS);
		motorController.configPeakOutputForward(1, DriveConstants.TIMEOUT_MS);
		motorController.configPeakOutputReverse(-1, DriveConstants.TIMEOUT_MS);

		/* Config the Velocity closed loop gains in slot0 */
		motorController.config_kF(DriveConstants.PID_LOOP_IDX, DriveConstants.F, DriveConstants.TIMEOUT_MS);
		motorController.config_kP(DriveConstants.PID_LOOP_IDX, DriveConstants.P, DriveConstants.TIMEOUT_MS);
		motorController.config_kI(DriveConstants.PID_LOOP_IDX, DriveConstants.I, DriveConstants.TIMEOUT_MS);
		motorController.config_kD(DriveConstants.PID_LOOP_IDX, DriveConstants.D, DriveConstants.TIMEOUT_MS);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 *
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
        // _talon.setSensorPhase(true);

  }

  private void configRightTalon(WPI_TalonFX motorController) {
    /* Factory Default all hardware to prevent unexpected behaviour */
    motorController.configFactoryDefault();

    /* Config neutral deadband to be the smallest possible */
    motorController.configNeutralDeadband(0.001, DriveConstants.TIMEOUT_MS);

    /* Config sensor used for Primary PID [Velocity] */
    motorController.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, DriveConstants.PID_LOOP_IDX, DriveConstants.TIMEOUT_MS);

    motorController.configAllowableClosedloopError(0, 1, DriveConstants.TIMEOUT_MS);

    // Invert right side
    motorController.setInverted(TalonFXInvertType.Clockwise);

    /* Config the peak and nominal outputs */
    motorController.configNominalOutputForward(0, DriveConstants.TIMEOUT_MS);
    motorController.configNominalOutputReverse(0, DriveConstants.TIMEOUT_MS);
    motorController.configPeakOutputForward(1, DriveConstants.TIMEOUT_MS);
    motorController.configPeakOutputReverse(-1, DriveConstants.TIMEOUT_MS);

    /* Config the Velocity closed loop gains in slot0 */
    motorController.config_kF(DriveConstants.PID_LOOP_IDX, DriveConstants.F, DriveConstants.TIMEOUT_MS);
    motorController.config_kP(DriveConstants.PID_LOOP_IDX, DriveConstants.P, DriveConstants.TIMEOUT_MS);
    motorController.config_kI(DriveConstants.PID_LOOP_IDX, DriveConstants.I, DriveConstants.TIMEOUT_MS);
    motorController.config_kD(DriveConstants.PID_LOOP_IDX, DriveConstants.D, DriveConstants.TIMEOUT_MS);

    /*
    * Talon FX does not need sensor phase set for its integrated sensor
    * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
    * and the user calls getSelectedSensor* to get the sensor's position/velocity.
    *
    * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
    */
    // _talon.setSensorPhase(true);
}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Velocity", getLeftVelocity());
    SmartDashboard.putNumber("Right Velocity", getRightVelocity());
  }

  public double getLeftVelocity() {
    return m_leftMaster.getSelectedSensorVelocity();
  }

  public double getRightVelocity() {
    return m_rightMaster.getSelectedSensorVelocity();
  }

  public void tankSquaredDrive(double leftSpeed, double rightSpeed) {
    SmartDashboard.putNumber("Left Stick Y", leftSpeed);
    SmartDashboard.putNumber("Right Stick Y", rightSpeed);
    double leftSpeedWithDeadband = MathUtil.applyDeadband(leftSpeed, 0.5);
    double rightSpeedWithDeadband = MathUtil.applyDeadband(rightSpeed, 0.5);
    SmartDashboard.putNumber("Left Stick Y With Deadband", leftSpeedWithDeadband);
    SmartDashboard.putNumber("Right Stick Y With Deadband", rightSpeedWithDeadband);
    m_leftMaster.set(ControlMode.Velocity, leftSpeedWithDeadband * leftSpeedWithDeadband * DriveConstants.MAX_VELOCITY);
    m_rightMaster.set(ControlMode.Velocity, rightSpeedWithDeadband * rightSpeedWithDeadband * DriveConstants.MAX_VELOCITY);
  }
}
