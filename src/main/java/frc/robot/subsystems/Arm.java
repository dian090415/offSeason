package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Main;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;

public class Arm extends SubsystemBase {
    private final CANcoder canEncoder = new CANcoder(30);
    private final TalonFX lMain = new TalonFX(24);
    private final TalonFX lFollow = new TalonFX(25);
    private final TalonFX rMain = new TalonFX(27);
    private final TalonFX rFollow = new TalonFX(26);
    private final PIDController pidController = new PIDController(1.0, 0, 0);

    // -----------------------------------新程式-------------------------------------------------

    public static final double MOTOR_ROTATIONS_PER_ARM_ROTATION = 79.3651 * 14.0 / 9.0;
    public static final double ENCODER_OFFSET_ROTATIONS = 0.12109375 + 0.125; // = 0.24609375
    public static final double K_V = 12.0 / (100 / MOTOR_ROTATIONS_PER_ARM_ROTATION);
    public static final double K_A = 0.25 /* v/oldRot/s^2 */ * 9.0 / 14.0; /* newRot/oldRot */;
    public static final Angle CCW_LIMIT = Degrees.of(110);
    public static final Angle CW_LIMIT = Degrees.of(40);
    private StatusSignal<Angle> m_angleSig = lMain.getPosition();
    private double m_goalRotations;
      private MotionMagicVoltage m_profileReq = new MotionMagicVoltage(0);

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
        CANcoderConfig();
    }


//----------------------------------------------------新code---------------------------------------------------------------------------

    private void configureMotors() {
        // in init function
        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.1; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = K_V; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = K_A; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 140.0; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kD = 0.4; // A velocity error of 1 rps results in 0.1 V output
        slot0Configs.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);// 使用閉迴路誤差的方向來決定 kS 正負號

        talonFXConfigs.MotionMagic
                .withMotionMagicCruiseVelocity(0.5)// 巡航
                .withMotionMagicAcceleration(0.66);// 加速度

        // 電流限制
        talonFXConfigs.CurrentLimits.withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(70.0)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(50.0);

        // Brake 模式
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // 馬達正反轉
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        talonFXConfigs.Feedback
                .withFeedbackRemoteSensorID(30)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                .withSensorToMechanismRatio(1)
                .withRotorToSensorRatio(MOTOR_ROTATIONS_PER_ARM_ROTATION);
        talonFXConfigs.SoftwareLimitSwitch
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(CCW_LIMIT)
                .withReverseSoftLimitThreshold(Degrees.of(1.34))
                .withReverseSoftLimitEnable(true);
        talonFXConfigs.CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40);

        // 套用設定
        lMain.getConfigurator().apply(talonFXConfigs);
        rMain.getConfigurator().apply(talonFXConfigs);
        lFollow.getConfigurator().apply(talonFXConfigs);
        rFollow.getConfigurator().apply(talonFXConfigs);
    }
    public void CANcoderConfig(){
      var CANcoderConfig = new CANcoderConfiguration();
      CANcoderConfig.MagnetSensor.MagnetOffset = ENCODER_OFFSET_ROTATIONS;
      CANcoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = Units.degreesToRotations(250);
      CANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
      canEncoder.getConfigurator().apply(CANcoderConfig);
    }

    public double getAngleRotations() {
        return m_angleSig.getValueAsDouble();
      }
    
      public double getAngleRadians() {
        return Units.rotationsToRadians(m_angleSig.getValueAsDouble());
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

    public boolean armatgoal(double position) {
        if (Math.abs(position - this.encoder()) <= 2.5) {
            return true;
        } else {
            return false;
        }
    }

    public double getLengthKg() {
        return MathUtil.interpolate(
          MainPivotConstants.K_G_RETRACTED,
          MainPivotConstants.K_G_EXTENDED,
          MathUtil.inverseInterpolate(
              ElevatorConstants.MIN_LENGTH.in(Meters),
              ElevatorConstants.MAX_LENGTH.in(Meters),
              getLengthMeters()));
      }

    public double getKgVolts() {
        return Math.cos(getAngleRadians())
            * getLengthKg();
      }

    public Command hold() {
        return sequence(runOnce(() -> m_goalRotations = getAngleRotations()),
          run(()->{
            lMain.setControl(
              m_profileReq.withPosition(m_goalRotations).withFeedForward(getKgVolts()));
          }));
      }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("armEncoder", encoder());
        SmartDashboard.putNumber("canEncoder", canEncoder.getAbsolutePosition().getValueAsDouble() * ratio);
        SmartDashboard.putBoolean("armatgoal", this.armatgoal(1));
    }
}
