package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
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

    // 你的機械比
    private final double ratio = 123.456789;
    private final double metersPerAngle = 1.0 / ratio;

    // Motion Magic 控制物件
    private final MotionMagicExpoVoltage motionMagic = new MotionMagicExpoVoltage(0);

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
        var config = new TalonFXConfiguration();

        // Slot0 PID
        var slot0 = config.Slot0;
        slot0.kS = 0.0;
        slot0.kV = 0.0;
        slot0.kA = 0.0;
        slot0.kP = 6.0;
        slot0.kI = 0.0;
        slot0.kD = 0.0;
        slot0.kG = 0.1;

        // Motion Magic
        config.MotionMagic.MotionMagicCruiseVelocity = 9999;

        // 馬達模式
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Sensor 比例
        config.Feedback.SensorToMechanismRatio = metersPerAngle;
        config.Feedback.FeedbackSensorSource = com.ctre.phoenix6.signals.FeedbackSensorSourceValue.RemoteCANcoder;
        config.Feedback.FeedbackRemoteSensorID = 30;

        // 電流限制
        config.CurrentLimits.withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(70.0)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(50.0);

        // 套用設定
        lMain.getConfigurator().apply(config);
        rMain.getConfigurator().apply(config);
        lFollow.getConfigurator().apply(config);
        rFollow.getConfigurator().apply(config);
    }

    // Motion Magic 移動
    public void moveTo(double targetPosition) {
        lMain.setControl(motionMagic.withPosition(targetPosition));
    }

    public double encoder() {
        return lMain.getPosition().getValueAsDouble();
    }

    public Command moveToCommand(double targetPosition) {
        return runOnce(() -> moveTo(targetPosition));
    }

    public void setVoltage(double speed) {
        speed = MathUtil.clamp(speed, -12, 12);
        lMain.setVoltage(speed);
    }

    public void stop() {
        lMain.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("armEncoder", encoder());
        SmartDashboard.putNumber("canEncoder", canEncoder.getAbsolutePosition().getValueAsDouble());
    }
}
