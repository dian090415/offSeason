package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class Elevator extends SubsystemBase {

    // -----------------------這我新寫的-----------------------------------------------------------

    
    private MotionMagicVoltage profileReq = new MotionMagicVoltage(MIN_LENGTH_ROTATIONS);

    public static final Angle MainPivot_CCW_LIMIT = Degrees.of(110);// 角度限制(wpi打包模式
    public static final Angle MainPivot_CW_LIMIT = Degrees.of(40);// 角度限制(wpi打包模式

    private DoubleSupplier angleRadiansSupplier = () -> MainPivot_CW_LIMIT.in(Radians);

    public static final Per<AngleUnit, DistanceUnit> MOTOR_ROTATIONS_PER_METER_UNIT = Rotations.of(1)
            .div(Inches.of(13.0 / 50.0 * (Math.PI * 1.4397) * 2));// div馬達圈數/電梯移動的距離
    public static final double MOTOR_ROTATIONS_PER_METER = MOTOR_ROTATIONS_PER_METER_UNIT.in(Rotations.per(Meter));// 其實就是捲捲毛算給我的

    public static final Distance MIN_LENGTH = Inches.of(27.0);
    public static final Distance MIN_PADDED_LENGTH = MIN_LENGTH.plus(Inches.of(0.5));// 在最小長度的基礎上，加上 0.5 英吋作為安全墊
    public static final Distance MAX_LENGTH = Inches.of(66.0);
    public static final double MIN_LENGTH_ROTATIONS = MIN_LENGTH.in(Meters)
            * MOTOR_ROTATIONS_PER_METER;
    public static final double MAX_LENGTH_ROTATIONS = MAX_LENGTH.in(Meters)
            * MOTOR_ROTATIONS_PER_METER;

    public static final PerUnit<VoltageUnit, AngularVelocityUnit> VoltsPerRotationPerSecond = Volts
            .per(RotationsPerSecond);// 確保單位正確 per = ÷（這樣是為了wpi計算安全 不知道爲什麽但國外就這樣寫
    public static final PerUnit<VoltageUnit, AngularAccelerationUnit> VoltsPerRotationPerSecondSquared = Volts
            .per(RotationsPerSecond.per(Second));// 確保單位正確 per = ÷ （這樣是為了wpi計算安全 不知道爲什麽但國外就這樣寫
    public static final Per<VoltageUnit, AngularVelocityUnit> K_V = VoltsPerRotationPerSecond
            .ofNative(0.15 * 1.508 / 1.4397);// 換成機構的數值

    public static final Per<VoltageUnit, AngularAccelerationUnit> K_A = VoltsPerRotationPerSecondSquared
            .ofNative(0.006 * 1.508 / 1.4397 * 1.15);// 換成機構的數值

    public static final double K_C = -0.17 * 1.508 / 1.4397;
    public static final double K_G = 0.4 * 1.508 / 1.4397;

    private final TalonFX main = new TalonFX(22);
    private final TalonFX follow = new TalonFX(23);
      private StatusSignal<Angle> positionSignal = main.getPosition();

    // ---------------------我寫的廢物code爛就是爛沒藉口-----------------------------------------
    private final double ratio = 0.28;
    private final double metersPerRotation = 0.03494 * 2 * 100;
    private final double[] Levelmeter = { 5.0, 0.0, 0.00816 * 100, (0.4098 * 100) - 5, (1.032 * 100) - 5, 30.98,
            50.98 }; // 初始， L1 , L2
    // , L3 , L4 ,
    // alage1 ,
    // alage2

    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(200,
            250);
    private final ProfiledPIDController pidController = new ProfiledPIDController(0.75, 0, 0.002, m_constraints);
    private final ElevatorFeedforward ElevatorFeedforward = new ElevatorFeedforward(0.0, 0.0, 0.0, 0.0);// 0.13218, 0.0,
                                                                                                        // 0.0081402,
                                                                                                        // 0.0072639

    private final VoltageOut voltagRequire = new VoltageOut(0.0);
    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(0.5).per(Second), Volts.of(2),
                    null, (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> {
                        this.main.setControl(voltagRequire.withOutput(volts.in(Volts)));
                    },
                    null,
                    this));
    public int Level;

    public Elevator() {
        pidController.setTolerance(3, 0.05);
        this.Config();
        follow.setControl(new Follower(main.getDeviceID(), false));
        // setDefaultCommand(this.hold());
    }

    // ---------------------新輸出模式---------------------
    public void Config() {
        // in init function
        var talonFXConfigs = new TalonFXConfiguration();

        // Brake 模式
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // 馬達正反轉
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        talonFXConfigs.SoftwareLimitSwitch.withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(MAX_LENGTH_ROTATIONS)
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(MIN_LENGTH_ROTATIONS);

        talonFXConfigs.CurrentLimits.withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(80));

        talonFXConfigs.Slot0.withKS(0)
                .withKV(K_V.in(VoltsPerRotationPerSecond))
                .withKA(K_A.in(VoltsPerRotationPerSecondSquared))
                .withKP(1)// TODO
                .withKD(0.25);
        talonFXConfigs.MotionMagic.withMotionMagicAcceleration(150)// 最大加速度
                .withMotionMagicCruiseVelocity(72);// 最大速度

        main.getConfigurator().apply(talonFXConfigs);
        follow.getConfigurator().apply(talonFXConfigs);
        main.setPosition(ElevatorConstants.MIN_LENGTH_ROTATIONS);
    }

    public double goalRotations = 0.0;

      public Command home() {
    return runOnce(() -> main.setPosition(ElevatorConstants.MIN_LENGTH_ROTATIONS)).ignoringDisable(true);
  }

    public double getLengthMeters() {
        return getMotorRotations() / MOTOR_ROTATIONS_PER_METER;
    }

    public double getMotorRotations() {
        return this.main.getPosition().getValueAsDouble();
    }

    public Command hold() {
        return this.runOnce(() -> profileReq.withPosition(getMotorRotations()))
                .andThen(this.run(() -> main.setControl(profileReq.withFeedForward(getKGVolts()))));
    }

    public double getKGVolts() {
        return K_G * Math.sin(angleRadiansSupplier.getAsDouble()) + K_C;
    }

    private void goToRotations(double motorRotations) {// 輸入m
        goalRotations = motorRotations;
        main.setControl(profileReq.withPosition(motorRotations).withFeedForward(getKGVolts()));
    }

    public Command goToLength(DoubleSupplier length) {
        return this.run(
                () -> goToRotations(length.getAsDouble() * MOTOR_ROTATIONS_PER_METER));
    }

    public void setAngleSupplier(DoubleSupplier angleRadians) {
        angleRadiansSupplier = angleRadians;
    }

    // -------------profiled pid輸出模式--------------------------------
    public double goal() {
        return this.pidController.getGoal().position;
    }

    public double encoder() {
        return this.main.getPosition().getValueAsDouble() * this.metersPerRotation;
    }

    public void setLevel(int level) {
        pidController.setGoal(Levelmeter[level]);
    }

    public void setLevelwait(int level) {
        this.Level = level;
        SmartDashboard.putNumber("Level", level);
    }

    public void waitset() {
        pidController.setGoal(Levelmeter[Level]);
    }

    public Command waitsetcmd() {
        return runOnce(() -> this.waitset());
    }

    public double Magiclevel() {
        return this.pidController.getGoal().position;
    }

    public Command gogoal() {
        return moveToPositionCommand();
    }

    public void setVoltage(double speed) {
        speed = MathUtil.clamp(speed, -12, 12);
        this.main.setVoltage(speed);
    }

    public void stop() {
        this.main.stopMotor();
    }

    public Command goalto() {
        return run(() -> {
            double feedbackVoltage = pidController.calculate(this.encoder());
            double feedforwardVoltage = ElevatorFeedforward.calculate(this.encoder(),
                    pidController.getSetpoint().velocity);
            setVoltage(feedbackVoltage + feedforwardVoltage);
        });
    }

    public Command moveToPositionCommand() {
        return Commands.sequence(
                restandset(),
                goalto());
    }

    public Command restandset() {
        return Commands.runOnce(() -> {
            pidController.reset(this.encoder());
        });
    }

    public Command holdCurrentPositionCommand() {
        return runOnce(() -> pidController.setGoal(this.encoder())).andThen(this.moveToPositionCommand());
    }

    public boolean atgoal() {
        return pidController.atGoal();
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return this.sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return this.sysIdRoutine.dynamic(direction);
    }

    public Command startCommand() {
        return Commands.runOnce(SignalLogger::start);
    }

    public Command stopCommand() {
        return Commands.runOnce(SignalLogger::stop);
    }

    public Command sysIdElevatorTest() {
        return Commands.sequence(
                this.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
                        .raceWith(new WaitUntilCommand(() -> this.encoder() > 100.0)),
                new WaitCommand(0.5),

                this.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
                        .raceWith(new WaitUntilCommand(() -> this.encoder() < 20.0)),
                new WaitCommand(0.5),

                this.sysIdDynamic(SysIdRoutine.Direction.kForward)
                        .raceWith(new WaitUntilCommand(() -> this.encoder() > 100.0)),
                new WaitCommand(0.5),

                this.sysIdDynamic(SysIdRoutine.Direction.kReverse)
                        .raceWith(new WaitUntilCommand(() -> this.encoder() < 20.0)));
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            main.set(0);
        }
    }
}
