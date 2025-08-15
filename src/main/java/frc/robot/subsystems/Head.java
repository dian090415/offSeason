package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Head extends SubsystemBase {

    private final TalonFX main = new TalonFX(7);
    private final TalonFX intake = new TalonFX(8);
    private final double ratio = 0.0;// TODO
    private final double metersPerangle = 0.0;// TODO

    public Head() {
        this.Config();
    }

    public void Config() {
        // in init function
        var talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
        
        // set Motion Magic Expo settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0; // Unlimited cruise velocity
        motionMagicConfigs.MotionMagicExpo_kV = 0.12; // kV is around 0.12 V/rps
        motionMagicConfigs.MotionMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)

        // Brake 模式
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // 馬達正反轉
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        main.getConfigurator().apply(talonFXConfigs);
        intake.getConfigurator().apply(talonFXConfigs);
    }
    public void Magicgo(){
        main.setControl(new MotionMagicExpoVoltage(0).withSlot(0).withPosition(0)); //TODO
    }
    public void setVoltage(double speed) {
        speed = MathUtil.clamp(speed, -12, 12);
        this.main.setVoltage(speed);
    }
    public void intakesetVoltage(double speed) {
        speed = MathUtil.clamp(speed, -12, 12);
        this.intake.setVoltage(speed);
    }
    public void stop(){
        this.main.stopMotor();
    }
}
