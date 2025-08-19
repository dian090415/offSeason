package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Head extends SubsystemBase {
    private final MotionMagicVoltage head = new MotionMagicVoltage(0.0);

    private final TalonFX main = new TalonFX(20);
    private final TalonFX intake = new TalonFX(21);
    private final double metersPerangle = (1/0.0263671875) * (1/360);// 馬達一圈轉0.026圈*360
    private final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(0);

    public Head() {
        this.Config();
    }

    public void Config() {
        // in init function
        var talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.0; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 15.0; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic Expo settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 9999; // Unlimited cruise velocity

        // Brake 模式
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // 馬達正反轉
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        talonFXConfigs.CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(70.0)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(50.0);
        talonFXConfigs.Feedback.SensorToMechanismRatio = metersPerangle;

        main.getConfigurator().apply(talonFXConfigs);
        intake.getConfigurator().apply(talonFXConfigs);
    }

    public void Magicgo(double Position) {
        main.setControl(m_request.withPosition(Position));
    }

    public Command magicgocCommand(double Position){
        return run(() -> this.Magicgo(Position));
    }
    public Command intake(){
        return run(() -> this.intake.setVoltage(5));
    }
    public void stop() {
        this.main.stopMotor();
        this.intake.stopMotor();
    }
    public void setVoltage(double volt){
        this.main.setVoltage(volt);
    }
        @Override
    public void periodic() {
        SmartDashboard.putNumber("headencoder", this.main.getPosition().getValueAsDouble());
    }
}
