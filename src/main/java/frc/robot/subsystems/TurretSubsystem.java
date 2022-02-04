package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
    private final WPI_TalonSRX m_turretMotor = new WPI_TalonSRX(TurretConstants.TURRET_TALON);

    public TurretSubsystem() {
    }

    public void configureTurret() {

    /* Factory default hardware to prevent unexpected behavior */
    m_turretMotor.configFactoryDefault();

    /* Configure Sensor Source for Pirmary PID */
    m_turretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, TurretConstants.PIDLOOP_ID, TurretConstants.TIMEOUT_MS);

    /* set deadband to super small 0.001 (0.1 %).
    The default deadband is 0.04 (4 %) */
    m_turretMotor.configNeutralDeadband(0.001, TurretConstants.TIMEOUT_MS);

    /**
     * Configure Talon SRX Output and Sensor direction accordingly Invert Motor to
     * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
     * sensor to have positive increment when driving Talon Forward (Green LED)
     */
    m_turretMotor.setSensorPhase(false);
    m_turretMotor.setInverted(true);

    /* Set relevant frame periods to be at least as fast as periodic rate */
    m_turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, TurretConstants.TIMEOUT_MS);
    m_turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, TurretConstants.TIMEOUT_MS);

    /* Set the peak and nominal outputs */
    m_turretMotor.configNominalOutputForward(0, TurretConstants.TIMEOUT_MS);
    m_turretMotor.configNominalOutputReverse(0, TurretConstants.TIMEOUT_MS);
    m_turretMotor.configPeakOutputForward(1, TurretConstants.TIMEOUT_MS);
    m_turretMotor.configPeakOutputReverse(-1, TurretConstants.TIMEOUT_MS);

    /* Set Motion Magic gains in slot0 - see documentation */
    m_turretMotor.selectProfileSlot(TurretConstants.SLOT_IDX, TurretConstants.PIDLOOP_ID);
    m_turretMotor.config_kF(TurretConstants.SLOT_IDX, TurretConstants.KF, TurretConstants.TIMEOUT_MS);
    m_turretMotor.config_kP(TurretConstants.SLOT_IDX, TurretConstants.KP, TurretConstants.TIMEOUT_MS);
    m_turretMotor.config_kI(TurretConstants.SLOT_IDX, TurretConstants.KI, TurretConstants.TIMEOUT_MS);
    m_turretMotor.config_kD(TurretConstants.SLOT_IDX, TurretConstants.KD, TurretConstants.TIMEOUT_MS);

    /* Set acceleration and vcruise velocity - see documentation */
    m_turretMotor.configMotionCruiseVelocity(3000, TurretConstants.TIMEOUT_MS);
    m_turretMotor.configMotionAcceleration(3000, TurretConstants.TIMEOUT_MS);

    /* Set allowable error */
    m_turretMotor.configAllowableClosedloopError(TurretConstants.SLOT_IDX, TurretConstants.ALLOWABLE_ERROR);

    /* Zero the sensor once on robot boot up */
    m_turretMotor.setSelectedSensorPosition(0, TurretConstants.PIDLOOP_ID, TurretConstants.TIMEOUT_MS);
    }

    //Moves the turret to the designated error in encoder untis.
    public void PIDmove(double error) {
        m_turretMotor.set(TalonSRXControlMode.MotionMagic, error);
    }

    //returns the encoder position of the shooter
    public double getPOS() {
        return m_turretMotor.getSelectedSensorPosition();
    }

    //returns the error used in the PID loop
    public double getDebugError() {
        return m_turretMotor.getClosedLoopError();
    }

    //returns true if the turret is accurate enough to shoot
    public boolean sufficientlyTargeted() {
        return Math.abs(m_turretMotor.getClosedLoopError()) < 5 ? true : false;
    }

    //moves the turret left
    public void moveLeft(double power) {
        m_turretMotor.set(power * -.2);
    }

    //moves the turret right
    public void moveRight(double power) {
        m_turretMotor.set(power * .2);
    }
}