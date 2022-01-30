// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
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

    configTalon(m_leftMaster);
    configTalon(m_rightMaster);
  }

  private void configTalon(WPI_TalonFX motorController) {
    		/* Factory Default all hardware to prevent unexpected behaviour */
		motorController.configFactoryDefault();

		/* Config neutral deadband to be the smallest possible */
		motorController.configNeutralDeadband(0.001);

		/* Config sensor used for Primary PID [Velocity] */
    motorController.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, DriveConstants.kPIDLoopIdx, DriveConstants.TIMEOUT_MS);


		/* Config the peak and nominal outputs */
		motorController.configNominalOutputForward(0, DriveConstants.TIMEOUT_MS);
		motorController.configNominalOutputReverse(0, DriveConstants.TIMEOUT_MS);
		motorController.configPeakOutputForward(1, DriveConstants.TIMEOUT_MS);
		motorController.configPeakOutputReverse(-1, DriveConstants.TIMEOUT_MS);

		/* Config the Velocity closed loop gains in slot0 */
		motorController.config_kF(DriveConstants.kPIDLoopIdx, DriveConstants.kF, DriveConstants.TIMEOUT_MS);
		motorController.config_kP(DriveConstants.kPIDLoopIdx, DriveConstants.kP, DriveConstants.TIMEOUT_MS);
		motorController.config_kI(DriveConstants.kPIDLoopIdx, DriveConstants.kI, DriveConstants.TIMEOUT_MS);
		motorController.config_kD(DriveConstants.kPIDLoopIdx, DriveConstants.kD, DriveConstants.TIMEOUT_MS);
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
    // This method will be called once per scheduler run
  }

  public double getLeftVelocity() {
    return m_leftMaster.getSelectedSensorVelocity();
  }

  public double getRightVelocity() {
    return m_rightMaster.getSelectedSensorVelocity();
  }

  public void tankLinearDrive(double leftSpeed, double rightSpeed) {
    if(Math.abs(leftSpeed) > .02) { m_leftMaster.set(ControlMode.Velocity, leftSpeed);}
    if(Math.abs(leftSpeed) > .02) { m_rightMaster.set(ControlMode.Velocity, rightSpeed);}
  }

  public void tankSquaredDrive(double leftSpeed, double rightSpeed) {
    if(Math.abs(leftSpeed) > .02) { m_leftMaster.set(ControlMode.Velocity, Math.exp(leftSpeed) * DriveConstants.MAX_VELOCITY); }
    if(Math.abs(leftSpeed) > .02) { m_rightMaster.set(ControlMode.Velocity, Math.exp(rightSpeed) * DriveConstants.MAX_VELOCITY);}
  }
}
