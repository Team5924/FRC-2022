package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.Constants.ShooterConstants;
public class ShooterSubsystem extends SubsystemBase {

    /**
     * The shooter has two motors spinning, relative to each other,
     * in the opposite dierction. The leaderSparkMax is also the
     * PIDController, and the other one follows.
     */

    private CANSparkMax m_leaderShooterSpark = new CANSparkMax(ShooterConstants.LEADER_SHOOTER_SPARK, MotorType.kBrushless);
    private CANSparkMax m_followerShooterSpark = new CANSparkMax(ShooterConstants.FOLLOWER_SHOOTER_SPARK, MotorType.kBrushless);

    private SparkMaxPIDController m_PIDController;

    // Subject to change
    private RelativeEncoder m_encoder;

    //private double shooterSetpoint;

    // Constructor for ShooterSubsystem class
    public ShooterSubsystem() {
        m_leaderShooterSpark.restoreFactoryDefaults();
        m_followerShooterSpark.restoreFactoryDefaults();

        m_followerShooterSpark.follow(m_leaderShooterSpark, true);

        m_PIDController = m_leaderShooterSpark.getPIDController();

        m_encoder = m_leaderShooterSpark.getEncoder();

        m_PIDController.setFF(ShooterConstants.FF);
        m_PIDController.setP(ShooterConstants.P);
        m_PIDController.setI(ShooterConstants.I);
        m_PIDController.setD(ShooterConstants.D);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        //SmartDashboard.putNumber("Shooter Setpoint", shooterSetpoint);
        SmartDashboard.putNumber("Shooter Speed", m_encoder.getVelocity());
        SmartDashboard.putBoolean("Shooter At Speed", isShooterAtSpeed());
    }

    // Checks to see if shooter is ready to fire
    public boolean isShooterAtSpeed() {
        return Math.abs(m_encoder.getVelocity() - ShooterConstants.SHOOTER_SPEED) <= ShooterConstants.ACCEPTABLE_RPM_ERROR;
    }

    public void runMotor() {
        // speed is in RPM
        m_PIDController.setReference(ShooterConstants.SHOOTER_SPEED, CANSparkMax.ControlType.kVelocity);
    }
}
