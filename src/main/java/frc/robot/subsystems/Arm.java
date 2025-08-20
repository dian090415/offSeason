package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

public class Arm extends SubsystemBase {
    private final CANcoder canEncoder = new CANcoder(30);
    private final TalonFX lMain = new TalonFX(24);
    private final TalonFX lFollow = new TalonFX(25);
    private final TalonFX rMain = new TalonFX(27);
    private final TalonFX rFollow = new TalonFX(26);
    private final PIDController pidController = new PIDController(1.0, 0, 0);

    // 你的機械比
    private final double ratio = 123.456789;
    private final double metersPerAngle = 1.0 / ratio;

    // Motion Magic 控制物件
    private final MotionMagicExpoVoltage motionMagic = new MotionMagicExpoVoltage(
            this.canEncoder.getPosition().getValueAsDouble() * ratio);

    public Arm() {
        // 設定馬達追隨
        lFollow.setControl(new Follower(lMain.getDeviceID(), false));
        rMain.setControl(new Follower(lMain.getDeviceID(), true));
        rFollow.setControl(new Follower(lMain.getDeviceID(), true));

        // 設定 TalonFX
        configureMotors();

        // 將 TalonFX 的位置與 CANcoder 對齊
        double initialPosition = canEncoder.getPosition().getValueAsDouble() * ratio;
        lMain.getConfigurator().setPosition(initialPosition);
    }

    private void configureMotors() {
        // in init function
        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.0; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 0.25; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0.0; // no output for integrated error
        slot0Configs.kD = 0.0000005; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic Expo settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0; // Unlimited cruise velocity
        motionMagicConfigs.MotionMagicExpo_kV = 0.05; // kV is around 0.12 V/rps
        motionMagicConfigs.MotionMagicExpo_kA = 0.01; // Use a slower kA of 0.1 V/(rps/s)

        // 電流限制
        talonFXConfigs.CurrentLimits.withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(70.0)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(50.0);

        // Brake 模式
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // 馬達正反轉
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // 套用設定
        lMain.getConfigurator().apply(talonFXConfigs);
        rMain.getConfigurator().apply(talonFXConfigs);
        lFollow.getConfigurator().apply(talonFXConfigs);
        rFollow.getConfigurator().apply(talonFXConfigs);
    }

    // Motion Magic 移動
    public void moveTo(double targetPosition) {
        SmartDashboard.putBoolean("armmove", true);
        lMain.setControl(motionMagic.withPosition(targetPosition));
    }

    public void pidmove(double position) {
        this.lMain.setVoltage(this.pidController.calculate(this.lMain.getPosition().getValueAsDouble(), position));
    }

    public double encoder() {
        return lMain.getPosition().getValueAsDouble();
    }

    public Command moveToCommand(double targetPosition) {
        return run(() -> moveTo(targetPosition));
    }

    public void setVoltage(double speed) {
        speed = MathUtil.clamp(speed, -12, 12);
        lMain.setVoltage(speed);
    }

    public void stop() {
        lMain.stopMotor();
    }
    public boolean armatgoal(double position){
        if(Math.abs(position - this.encoder()) <= 1){
            return true;
        }else{
            return false;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("armEncoder", encoder());
        SmartDashboard.putNumber("canEncoder", canEncoder.getAbsolutePosition().getValueAsDouble() * ratio);
        SmartDashboard.putBoolean("armatgoal", this.armatgoal(1));
    }
}
