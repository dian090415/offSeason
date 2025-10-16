package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.WristConstants;


public class Head extends SubsystemBase {
    private final MotionMagicVoltage headvolt = new MotionMagicVoltage(0.0);

    private final TalonFX head = new TalonFX(20);
    // private final TalonFX intake = new TalonFX(21);
    private final double metersPerangle = (1 / 0.0263671875) * (1 / 360);// 馬達一圈轉0.026圈*360
    private final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(0);
    // private final AnalogInput IRSensor = new AnalogInput(0);
    // -------------------新code---------------------
    public static final double K_S = 0;
    public static final double K_V = 4.548;
    public static final double K_A = 0.2 * 0.45 / 0.25;
    public static final double MOTOR_ROTATIONS_PER_ARM_ROTATION = 48.0 / 9.0 * 40.0 / 15.0 * 40.0 / 15.0;
    public static final Angle CCW_LIMIT = Degrees.of(146.8);
    public static final Angle CW_LIMIT = Degrees.of(-70);
    private MotionMagicVoltage m_profileReq = new MotionMagicVoltage(0);
    private StatusSignal<Angle> m_angleSig = head.getPosition();
    private StatusSignal<AngularVelocity> m_velocitySig = head.getVelocity();
    private StatusSignal<Double> m_setpointSig = head.getClosedLoopReference();// 取得目前閉環控制器的 目標值 (setpoint)
    private StatusSignal<Current> m_currentSig = head.getStatorCurrent();// 取得馬達的 定子電流（流經馬達線圈的電流）
    private double m_goalRotations;
    public static final Angle K_G_ANGLE = Degrees.of(35.06);//Rotations.of(-0.072);
    public static final Angle K_G_ANGLE_WITH_CORAL = Degrees.of(45);
    public static final double K_G = 0.45;
    private DoubleSupplier m_mainAngleSupplier = () -> 0;
    private VoltageOut m_voltageReq = new VoltageOut(0);
    public void setMainAngleSupplier(DoubleSupplier mainAngleSupplier) {
        m_mainAngleSupplier = mainAngleSupplier;
      }

    public Head() {
        this.Config();//刷入馬達設定
        m_setpointSig.setUpdateFrequency(50);// 設定roborio刷新頻率
        m_currentSig.setUpdateFrequency(50);// 設定roborio刷新頻率
        setDefaultCommand(hold());
        this.head.getConfigurator().setPosition(WristConstants.CW_LIMIT);
    }

    public void Config() {
        // in init function
        var talonFXConfigs = new TalonFXConfiguration();

        talonFXConfigs.Slot0.withKS(K_S).withKV(K_V).withKA(K_A).withKP(50).withKD(0);

        talonFXConfigs.CurrentLimits.withStatorCurrentLimitEnable(true).withStatorCurrentLimit(120)
                .withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(60);

        talonFXConfigs.MotionMagic.withMotionMagicCruiseVelocity(1).withMotionMagicAcceleration(2.8);

        talonFXConfigs.Feedback
                // .withFeedbackRemoteSensorID(34)
                // .withFeedbackSensorSource(FeedbackSensorSourceValue.SyncCANcoder)
                .withSensorToMechanismRatio(MOTOR_ROTATIONS_PER_ARM_ROTATION);

        talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        talonFXConfigs.SoftwareLimitSwitch.withForwardSoftLimitEnable(false)
                .withForwardSoftLimitThreshold(CCW_LIMIT)
                .withReverseSoftLimitThreshold(CW_LIMIT)
                .withReverseSoftLimitEnable(false);
        talonFXConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

        head.getConfigurator().apply(talonFXConfigs);
        // intake.getConfigurator().apply(talonFXConfigs);
    }

    // public void Magicgo(double Position) {
    //     head.setControl(m_request.withPosition(Position));
    // }

    // public Command magicgocCommand(double Position) {
    //     return run(() -> this.Magicgo(Position));
    // }

    // public Command coarlintakeexecute() {
    //     return Commands.sequence(
    //             Commands.run(() -> this.intakexcute(4), this),
    //             new WaitCommand(0.75),
    //             Commands.run(() -> this.intakexcute(7), this),
    //             new WaitCommand(0.75));
    // }

    // public Command alagelintakeexecute() {
    //     return Commands.run(() -> this.intakexcute(-7), this);
    // }

    // public Command alageput() {
    //     return Commands.runEnd(() -> this.intakexcute(7), this::intakestop, this);
    // }

    // public Command intakebackexecute() {
    //     return Commands.runEnd(() -> this.intakebackexcute(), this::intakestop, this);
    // }

    // public Command intakeputcmd() {
    //     return Commands.runEnd(() -> this.intakeput(), this::intakestop, this);
    // }

    // public Command intakestopCmd() {
    //     return Commands.run(() -> this.intakestop(), this);
    // }

    // public void intakestop() {
    //     this.intake.setVoltage(0.0);
    // }

    // public void headstop() {
    //     this.head.stopMotor();
    // }

    // public void intakexcute(double volt) {
    //     this.intake.setVoltage(volt);
    // }

    // public void intakebackexcute() {
    //     this.intake.setVoltage(-1);
    // }

    // public void intakeput() {
    //     this.intake.setVoltage(-6);
    // }

    // public void intakesetVoltage(double volt) {
    //     this.intake.setVoltage(volt);
    // }

    // public boolean isCoralIn() {
    //     return this.IRSensor.getVoltage() <= 1.0 ? true : false;
    // }

    // public boolean isCoralout() {
    //     return this.IRSensor.getVoltage() >= 1.0 ? true : false;
    // }

    //-----------------新code----------------------
    public double setpoint() {
        m_setpointSig.refresh();
        return m_setpointSig.getValueAsDouble();
      }
      public Command home() {
        return this.runOnce(()->head.getConfigurator().setPosition(CW_LIMIT)).ignoringDisable(true);//ignoringDisable(true)即使 DriverStation 是 Disabled 狀態，這個 command 也會執行（因為這個操作只是在軟體上校正 encoder，沒有真的驅動馬達）
      }
        public double getAngleRotations() {
    return m_angleSig.getValueAsDouble();
  }

  public double getAngleRadians() {
    return Units.rotationsToRadians(m_angleSig.getValueAsDouble());
  }

  public double getKgVolts() {
    return Math.cos(getAngleRadians() - K_G_ANGLE.in(Radians) + m_mainAngleSupplier.getAsDouble())* K_G;
  }

  public void setAngleRadians(double angle) {
    m_goalRotations = Units.radiansToRotations(angle);

    head.setControl(
        m_profileReq.withPosition(m_goalRotations).withFeedForward(getKgVolts()));
  }

  public Command goTo(DoubleSupplier angleSupplier) {
    return run(() -> setAngleRadians(angleSupplier.getAsDouble()));
  }

  public Command testgoTo(double angleSupplier) {
    return run(() -> setAngleRadians(angleSupplier));
  }

  public Command goTo(Supplier<Angle> angleSupplier) {
    return run(() -> setAngleRadians(angleSupplier.get().in(Radians)));
  }

  public Command voltage(DoubleSupplier voltageSupplier) {
    return run(
        () -> {
          head.setControl(m_voltageReq.withOutput(voltageSupplier.getAsDouble()));
        });
  }

  public Command hold() {
    return sequence(runOnce(() -> setAngleRadians(getAngleRadians())), Commands.idle());//Commands.idle()  進入一個「什麼都不做，但永遠不會結束」的狀態
  }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(m_currentSig, m_angleSig);//一次性向馬達控制器請求 最新的狀態數據
        if (DriverStation.isDisabled()) {
            head.set(0);
        }
    }
}
