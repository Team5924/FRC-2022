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

    private double shooterSetpoint;

    // Constructor for ShooterSubsystem class
    public ShooterSubsystem() {
        m_leaderShooterSpark.restoreFactoryDefaults();
        m_followerShooterSpark.restoreFactoryDefaults();

        m_followerShooterSpark.follow(m_leaderShooterSpark);

        m_PIDController = m_leaderShooterSpark.getPIDController();

        m_encoder = m_leaderShooterSpark.getEncoder();

        m_PIDController.setP(ShooterConstants.P);
        m_PIDController.setI(ShooterConstants.I);
        m_PIDController.setD(ShooterConstants.D);
    }

    /**
     * ========== FREEDOM UNITS ==========
    */

    public void setSpeed(double speed) {
        shooterSetpoint = speed;
        setFeedForward(shooterSetpoint);
        // "speed" should be in RPM
        m_PIDController.setReference(speed, CANSparkMax.ControlType.kVelocity);
    }

    // Calculates how fast (in ft/s) the ball must be shot from a certain distance
    public double getShotVelocity(double distance) {
       // Velocity for Distance Away: y = 0.7125x + 17.725
       return (0.7125 * distance) + 17.725;
    }

    // Calculates RPM for shooter to shoot ball at a certain velocity (in ft/s)
    public double shotVelocityToShooterRPM(double velocity) {
        // Velocity to RPM: y = (25137/157.89)x
        return (25137/157.89) * velocity;
    }

    // Checks to see if shooter is ready to fire
    public boolean isShooterAtSpeed() {
        return Math.abs(m_encoder.getVelocity() - shooterSetpoint) <= ShooterConstants.ACCEPTABLE_RPM_ERROR;
    }

    /**
     * Feed forward changes based on the shooterSetpoint
    */
    private void setFeedForward(double speed) {
        // Reference: https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#calculating-velocity-feed-forward-gain-kf
        // FF = (1 X 1 RPM) / Target RPM
        m_PIDController.setFF(1/speed);
    }
}
