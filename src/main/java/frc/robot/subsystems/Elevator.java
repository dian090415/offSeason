package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class Elevator extends SubsystemBase {

    private final TalonFX main = new TalonFX(22);
    private final TalonFX follow = new TalonFX(23);
    private final double ratio = 0.28;
    private final double metersPerRotation = 0.03494 * 2 * 100;
    private final double[] Levelmeter = { 0.0, 0.0, 0.00816 * 100, 0.4098 * 100, 1.032 * 100 ,0.0, 0.0 }; // 初始， L1 , L2
                                                                                                          // , L3 , L4 ,
                                                                                                          // alage1 ,
                                                                                                          // alage2

    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(550,
            500);
    private final ProfiledPIDController pidController = new ProfiledPIDController(0.65, 0, 0.015, m_constraints);
    private final ElevatorFeedforward ElevatorFeedforward = new ElevatorFeedforward(0.0, 0.0, 0.0, 0.0);//0.13218, 0.0, 0.056884, 0.05076

    private final VoltageOut voltagRequire = new VoltageOut(0.0);
    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(0.5).per(Second), Volts.of(2),
                    null, (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts)
                     -> {
                        this.main.setControl(voltagRequire.withOutput(volts.in(Volts)));
                    },
                    null,
                    this));

    public Elevator() {
        pidController.setTolerance(0.2, 0.05);
        this.main.getConfigurator().setPosition(0.0);
        this.follow.getConfigurator().setPosition(0.0);
        this.Config();
        follow.setControl(new Follower(main.getDeviceID(), false));
    }

    // ---------------------新輸出模式---------------------
    public void Config() {
        // in init function
        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.13218; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.056884; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.05076; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative

        // set Motion Magic Velocity settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
        motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

        // Brake 模式
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // 馬達正反轉
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        talonFXConfigs.CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(70.0)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(50.0);

        main.getConfigurator().apply(talonFXConfigs);
        follow.getConfigurator().apply(talonFXConfigs);
    }

    public void Magicgo() {
        main.setControl(new MotionMagicExpoVoltage(0).withSlot(0).withPosition(this.Magiclevel()));
    }

    // -------------profiled pid輸出模式--------------------------------
    public double goal(){
        return this.pidController.getGoal().position;
    }
    public double encoder() {
        return this.main.getPosition().getValueAsDouble() * this.metersPerRotation;
    }

    public void setLevel(int level) {
        if (level >= 0 && level < Levelmeter.length) {
            pidController.setGoal(Levelmeter[level]);
        }
        SmartDashboard.putNumber("Level", level);
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
            SmartDashboard.putNumber("ELEsetout", feedbackVoltage + feedforwardVoltage);

        });
    }

    public Command moveToPositionCommand() {
        return Commands.sequence(
                restandset(),
                goalto()
                        .until(() -> pidController.atGoal()))
                .withTimeout(2);
    }
    public Command moveTo(){
        return Commands.sequence(
            this.moveToPositionCommand(),
            this.keepCommand());
    } 

    public Command restandset() {
        return Commands.runOnce(() -> {
            pidController.reset(this.encoder());
        });
    }

    public Command keepCommand() {
        return runEnd(
            () -> {
                double pidOutput = pidController.calculate(this.encoder(), this.encoder());
                double ff = ElevatorFeedforward.calculate(0.0);
                main.setVoltage(pidOutput + ff);
            },
            () -> main.setVoltage(0.0) // 離開時停止
        );
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
        SmartDashboard.putBoolean("atgoal", this.atgoal());
        SmartDashboard.putNumber("goal", this.pidController.getGoal().position);
        SmartDashboard.putNumber("Eleencoder", this.encoder());
    }
}
