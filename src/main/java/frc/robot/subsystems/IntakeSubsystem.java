package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private final Compressor compressor = new Compressor(IntakeConstants.CTRE_PCM, PneumaticsModuleType.CTREPCM);
    
    private final DoubleSolenoid leftDoubleSolenoid = new DoubleSolenoid(IntakeConstants.CTRE_PCM, PneumaticsModuleType.CTREPCM, IntakeConstants.LEFT_PNEUMATIC_FORWARD, IntakeConstants.LEFT_PNEUMATIC_REVERSE);
    private final DoubleSolenoid rightDoubleSolenoid = new DoubleSolenoid(IntakeConstants.CTRE_PCM, PneumaticsModuleType.CTREPCM, IntakeConstants.RIGHT_PNEUMATIC_FORWARD, IntakeConstants.RIGHT_PNEUMATIC_REVERSE);

    private final WPI_TalonSRX intakeTalon = new WPI_TalonSRX(IntakeConstants.INTAKE_TALON); // Talon 5

    public IntakeSubsystem() {
        leftDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
        rightDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Left Pneumatic State", solenoidStatus(leftDoubleSolenoid));
        SmartDashboard.putString("Right Pneumatic State", solenoidStatus(rightDoubleSolenoid));
        SmartDashboard.putBoolean("Compressor Status", compressor.enabled());
        SmartDashboard.putBoolean("Enough Pressure", compressor.getPressureSwitchValue());
        SmartDashboard.putBoolean("Intake Deployed", isIntakeDeployed());
    }

    private String solenoidStatus(DoubleSolenoid solenoid) {
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

    public void intake() {
        // moves the motors
        intakeTalon.set(-1);

    }

    public void deploy() {
        leftDoubleSolenoid.set(DoubleSolenoid.Value.kForward);
        rightDoubleSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void retract() {
        leftDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
        rightDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public boolean isIntakeDeployed() {
        return leftDoubleSolenoid.get().equals(DoubleSolenoid.Value.kForward) && rightDoubleSolenoid.get().equals(DoubleSolenoid.Value.kForward); 
    }
}
