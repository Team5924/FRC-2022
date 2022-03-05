package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    /*
        The shooter has two motors spinning, relative to each other,
        in the opposite dierction. The leaderSparkMax is also the
        PIDController, and the other one follows.
    */

    private LimelightSubsystem m_limelight;

    private CANSparkMax m_leaderShooterSpark = new CANSparkMax(ShooterConstants.LEADER_SHOOTER_SPARK, MotorType.kBrushless);
    private CANSparkMax m_followerShooterSpark = new CANSparkMax(ShooterConstants.FOLLOWER_SHOOTER_SPARK, MotorType.kBrushless);

    private SparkMaxPIDController m_PIDController;

    private RelativeEncoder m_encoder;

    private double F;
    private double velocity;
    private double rpm;
    private double speed;

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

    // Freedom Units
    public double distanceToVelocity() {
       // Velocity for Distance Away: y = 0.7125x + 17.725
       velocity = (0.7125 * m_limelight.getDistance()) + 17.725;
       return velocity;
    }

    public double velocityToRPM() {
        // Velocity to RPM: y = (25137/157.89)x
        rpm = (25137/157.89) * velocity;
        return rpm;
    }

    public double RPMToSpeed() {
        // RPM to Speed: y = (24015211.1/1.0102949748 * 10^11)(x-4410) + 1
        speed = 24015211.1 / (1.0102949748 * Math.pow(10, 11)) * (rpm - 4410) + 1;
        return speed;
    }

    /*
        Feed forward changes based on the target rpm, which changes based
        on the robot's distance away from the target.
    */
    public void setFeedForward(double distance) {
        // Reference: https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#calculating-velocity-feed-forward-gain-kf
        // kF = (1 X 1 RPM) / Target RPM
        F = 1/rpm;
        m_PIDController.setFF(F);
    }

    public void setSpeed(double speed) {
        // "speed" should be in RPM
        m_PIDController.setReference(speed, CANSparkMax.ControlType.kVelocity);
    }

    @Override
    public void periodic() {
        // temp - for troubleshooting
        SmartDashboard.putNumber("Shooter Velocity", velocity);
        SmartDashboard.putNumber("Shooter kF", F);
    }

/*

    Useless code for the time being


    private double distanceToSpeed(double distance) {
        // Write a converstion method for distance to speed
        return distance;
    }

    public void setSpeedForDistance(double distance) {
        double speed = distanceToSpeed(distance);
        m_PIDController.setReference(speed, ControlType.kVelocity);
    }

    public boolean isAtSpeedForDistance(double distance) {
        // Write a converstion method for distance to speed later
        double speed = distanceToSpeed(distance);
        return Math.abs(m_encoder.getVelocity() - speed) <= ShooterConstants.ACCEPTABLE_RPM_ERROR;
    }
*/
}
