package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
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

    private final TalonFX main = new TalonFX(5);
    private final TalonFX follow = new TalonFX(6);
    private final double ratio = 0.28;
    private final double metersPerRotation = 0.03494 * 2;
    private final double[] Levelmeter = {0.0 , 0.0, 0.00816, 0.4098, 1.032, 0.0, 0.0}; //初始， L1 , L2 , L3 , L4 , alage1 , alage2

    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(0.2,
            0.1);
    private final ProfiledPIDController pidController = new ProfiledPIDController(0.85, 0, 0.0435, m_constraints);
    private final ElevatorFeedforward ElevatorFeedforward = new ElevatorFeedforward(0.0, 0.0, 0.0, 0.0);// TODO

    private final VoltageOut voltagRequire = new VoltageOut(0.0);
    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(2).per(Second), Volts.of(5),
                    null, (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> {
                        this.main.setControl(voltagRequire.withOutput(volts.in(Volts)));
                    },
                    null,
                    this));

    public Elevator() {


        follow.setControl(new Follower(main.getDeviceID(), true));
    }
    public void Config(){
        Slot0Configs slot0Config = new Slot0Configs();
        slot0Config.kP = 70.0; // 依實測調整
        slot0Config.kG = 0.2; // 重力抵銷，依實測

        this.main.getConfigurator().apply(slot0Config); 
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
    
    
    public Command gogoal() {
        return moveToPositionCommand()
            .andThen(keepCommand());
    }
    
    public Command levelCommand(int whatLevel) {
        return runOnce(() -> setLevel(whatLevel));
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
                goalto()
                        .until(() -> pidController.atGoal()))
                .withTimeout(3);
    }

    public Command restandset() {
        return Commands.runOnce(() -> {
            pidController.reset(this.encoder());
        });
    }

    public void keep() {
        this.main.setControl(new PositionVoltage(pidController.getGoal().position / metersPerRotation).withSlot(0));
    }
    public Command keepCommand(){
        return runOnce(() -> this.keep());
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

    // public Command sysIdElevatorTest() { TODO
    // return Commands.sequence(
    // this.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
    // .raceWith(new WaitUntilCommand(() -> this.encoder() > 0.15)),
    // new WaitCommand(1.5),

    // this.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
    // .raceWith(new WaitUntilCommand(() -> this.encoder() < 0.15)),
    // new WaitCommand(1.5),

    // this.sysIdDynamic(SysIdRoutine.Direction.kForward)
    // .raceWith(new WaitUntilCommand(() -> this.encoder() > 0.15)),
    // new WaitCommand(1.5),

    // this.sysIdDynamic(SysIdRoutine.Direction.kReverse)
    // .raceWith(new WaitUntilCommand(() -> this.encoder() < 0.15)));
    // }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("atgoal", this.atgoal());
        SmartDashboard.putNumber("goal", this.pidController.getGoal().position);
    }
}
