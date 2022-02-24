// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

public class DriveSubsystem extends SubsystemBase {

  private final WPI_TalonFX m_leftMaster = new WPI_TalonFX(DriveConstants.LEFT_FRONT_TALON);
  private final WPI_TalonFX m_rightMaster = new WPI_TalonFX(DriveConstants.RIGHT_FRONT_TALON);
  private final WPI_TalonFX m_leftSlave = new WPI_TalonFX(DriveConstants.LEFT_BACK_TALON);
  private final WPI_TalonFX m_rightSlave = new WPI_TalonFX(DriveConstants.RIGHT_BACK_TALON);

  /** Creates a new Drivetrain. */
  public DriveSubsystem() {
    // Sychronizes motors with other motors on the same side of the robot
    m_leftSlave.follow(m_leftMaster);
    m_rightSlave.follow(m_rightMaster);
    // Configure talon settings
    configTalon(m_leftMaster);
    configTalon(m_rightMaster);
  }

  private void configTalon(WPI_TalonFX motorController) {
    // Factory default hardware to prevent unexpected behaviour
		motorController.configFactoryDefault();

		// Config neutral deadband to be the smallest possible
		motorController.configNeutralDeadband(0.001, DriveConstants.TIMEOUT_MS);

		// Config sensor used for Primary PID [Velocity]
    motorController.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, DriveConstants.PID_LOOP_IDX, DriveConstants.TIMEOUT_MS);

    // Config allowable closed loop error
    motorController.configAllowableClosedloopError(0, 1, DriveConstants.TIMEOUT_MS);

		// Config the peak and nominal outputs
		motorController.configNominalOutputForward(0, DriveConstants.TIMEOUT_MS);
		motorController.configNominalOutputReverse(0, DriveConstants.TIMEOUT_MS);
		motorController.configPeakOutputForward(1, DriveConstants.TIMEOUT_MS);
		motorController.configPeakOutputReverse(-1, DriveConstants.TIMEOUT_MS);

    // Config amp limits to avoid breaking a circuit
    motorController.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, DriveConstants.CURRENT_LIMIT, DriveConstants.TRIGGER_THRESHOLD_CURRENT, DriveConstants.TRIGGER_THRESHOLD_TIME));

    // Config motors to brake when neutral (superior for drivetrain)
    motorController.setNeutralMode(NeutralMode.Brake);

		// Config the Velocity closed loop gains in slot0
		motorController.config_kF(DriveConstants.SLOT_IDX, DriveConstants.F, DriveConstants.TIMEOUT_MS);
		motorController.config_kP(DriveConstants.SLOT_IDX, DriveConstants.P, DriveConstants.TIMEOUT_MS);
		motorController.config_kI(DriveConstants.SLOT_IDX, DriveConstants.I, DriveConstants.TIMEOUT_MS);
		motorController.config_kD(DriveConstants.SLOT_IDX, DriveConstants.D, DriveConstants.TIMEOUT_MS);
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
    SmartDashboard.putNumber("Left Velocity Feet", sensorToFeetPerSecond(getLeftVelocity()));
    SmartDashboard.putNumber("Right Velocity Feet", sensorToFeetPerSecond(getRightVelocity()));
  }

  public double getLeftVelocity() {
    return m_leftMaster.getSelectedSensorVelocity();
  }

  public double getRightVelocity() {
    return m_rightMaster.getSelectedSensorVelocity();
  }

  public void tankSquaredDrive(double leftJoystick, double rightJoystick) {
    double leftJoystickWithDeadband = MathUtil.applyDeadband(leftJoystick, OIConstants.DRIVER_CONTROLLER_DEADBAND);
    double rightJoystickWithDeadband = MathUtil.applyDeadband(rightJoystick, OIConstants.DRIVER_CONTROLLER_DEADBAND);
    setLeftVelocityFromJoystick(leftJoystickWithDeadband);
    setRightVelocityFromJoystick(rightJoystickWithDeadband);
  }

  private void setLeftVelocityFromJoystick(double input) {
    double speed = squareKeepSign(input) * DriveConstants.MAX_VELOCITY;
    m_leftMaster.set(ControlMode.Velocity, speed);
    SmartDashboard.putNumber("Left Velocity Intended", speed);
    SmartDashboard.putNumber("Left Difference", speed - getLeftVelocity());
  }

  private void setRightVelocityFromJoystick(double input) {
    // Multiply by -1 to invert that side. Technically .setInverted(true) should be called on the master motor for this side but that didn't work for me
    double speed = squareKeepSign(input) * DriveConstants.MAX_VELOCITY * -1;
    m_rightMaster.set(ControlMode.Velocity, speed);
    SmartDashboard.putNumber("Right Velocity Intended", speed);
    SmartDashboard.putNumber("Right Difference", speed - getRightVelocity());
  }

  private double squareKeepSign(double num) {
    if (num < 0) {
      return num * num * -1;
    } else {
      return num * num;
    }
  }

  private double sensorToFeetPerSecond(double sensor) {
    /*
    Multiply by 10 to change to per second from per 100ms
    Divide by 2048 to get rotations from sensor units
    Multiply by 4π to get inches from rotations
    Divide by 12 to get feet from inches
    Divide by 9 to account for gearbox
    */
    return sensor * 10 / 2048 * 4 * Math.PI / 12 / 9;
  }
}
