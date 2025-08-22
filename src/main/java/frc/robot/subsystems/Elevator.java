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
    private final double[] Levelmeter = {5.0, 0.0, 0.00816 * 100, (0.4098 * 100)-5, (1.032 * 100)-5 ,30.98, 50.98}; // 初始， L1 , L2
                                                                                                          // , L3 , L4 ,
                                                                                                          // alage1 ,
                                                                                                          // alage2

    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(200,
            250);
    private final ProfiledPIDController pidController = new ProfiledPIDController(0.75, 0, 0.002, m_constraints);
    private final ElevatorFeedforward ElevatorFeedforward = new ElevatorFeedforward(0.0, 0.0, 0.0, 0.0);//0.13218, 0.0, 0.0081402, 0.0072639

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
    public int Level;

    public Elevator() {
        pidController.setTolerance(3, 0.05);
        this.main.getConfigurator().setPosition(0.0);
        this.follow.getConfigurator().setPosition(0.0);
        this.Config();
        follow.setControl(new Follower(main.getDeviceID(), false));
    }

    // ---------------------新輸出模式---------------------
    public void Config() {
        // in init function
        var talonFXConfigs = new TalonFXConfiguration();

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
            pidController.setGoal(Levelmeter[level]);
    }
    public void setLevelwait(int level) {            
        this.Level = level;
        SmartDashboard.putNumber("Level", level);
    }
    public void waitset(){
        pidController.setGoal(Levelmeter[Level]);
    }
    public Command waitsetcmd(){
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
        SmartDashboard.putBoolean("atgoal", this.atgoal());
        SmartDashboard.putNumber("goal", this.pidController.getGoal().position);
    }
}
