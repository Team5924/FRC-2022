package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid; // https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj/DoubleSolenoid.html
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private final Compressor compressor = new Compressor(IntakeConstants.CTRE_PCM, PneumaticsModuleType.CTREPCM);

    private final DoubleSolenoid leftDoubleSolenoid = new DoubleSolenoid(IntakeConstants.CTRE_PCM, PneumaticsModuleType.CTREPCM, IntakeConstants.LEFT_PNEUMATIC_FORWARD, IntakeConstants.LEFT_PNEUMATIC_REVERSE);
    private final DoubleSolenoid rightDoubleSolenoid = new DoubleSolenoid(IntakeConstants.CTRE_PCM, PneumaticsModuleType.CTREPCM, IntakeConstants.RIGHT_PNEUMATIC_FORWARD, IntakeConstants.RIGHT_PNEUMATIC_REVERSE);

    private final CANSparkMax leaderIntakeSpark = new CANSparkMax(IntakeConstants.LEADER_INTAKE_SPARK, MotorType.kBrushless);
    private final CANSparkMax followerIntakeSpark = new CANSparkMax(IntakeConstants.FOLLOWER_INTAKE_SPARK, MotorType.kBrushless);

    public IntakeSubsystem() {
        compressor.disable();

        leftDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
        rightDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);

        leaderIntakeSpark.restoreFactoryDefaults();
        followerIntakeSpark.restoreFactoryDefaults();

        // followerIntakeSpark is inverted relative to leaderIntakeSpark
        followerIntakeSpark.follow(leaderIntakeSpark, true);
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Left Pneumatic State", solenoidStatusAsString(leftDoubleSolenoid));
        SmartDashboard.putString("Right Pneumatic State", solenoidStatusAsString(rightDoubleSolenoid));
        SmartDashboard.putBoolean("Compressor Status", compressor.enabled());
        SmartDashboard.putBoolean("Enough Pressure", compressor.getPressureSwitchValue());
        SmartDashboard.putBoolean("Intake Deployed", isIntakeDeployed());
    }

    private String solenoidStatusAsString(DoubleSolenoid solenoid) {
        DoubleSolenoid.Value solenoidState = solenoid.get();
        if (solenoidState.equals(DoubleSolenoid.Value.kForward)) {
            return "Forward";
        } else if (solenoidState.equals(DoubleSolenoid.Value.kReverse)) {
            return "Reverse";
        } else if (solenoidState.equals(DoubleSolenoid.Value.kOff)) {
            return "Off";
        } else {
            return "Error";
        }
    }

    public void deployIntake() {
        leftDoubleSolenoid.set(DoubleSolenoid.Value.kForward);
        rightDoubleSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void retractIntake() {
        leftDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
        rightDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public boolean isIntakeDeployed() {
        // returns the status of the intake, used to determine whether to retract or deploy intake
        return leftDoubleSolenoid.get().equals(DoubleSolenoid.Value.kForward) && rightDoubleSolenoid.get().equals(DoubleSolenoid.Value.kForward); 
    }

    public void enableCompressor() {
        compressor.enableDigital();
    }

    public void disableCompressor() {
        compressor.disable();
    }

    public boolean isCompressorOn() {
        return compressor.enabled();
    }

    public void enableIntakeMotor() {
        leaderIntakeSpark.set(0.3);
    }

    public void disableIntakeMotor() {
        leaderIntakeSpark.stopMotor();
    }

    public void reverseMotor() {
        leaderIntakeSpark.set(-0.3);
    }

    public boolean isIntakeMotorRunning() {
        // intakeSpark.get() returns 0 when the motor is not running
        return leaderIntakeSpark.get() != 0;
    }
}
