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
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Head extends SubsystemBase {
    private final MotionMagicVoltage headvolt = new MotionMagicVoltage(0.0);

    private final TalonFX head = new TalonFX(20);
    private final TalonFX intake = new TalonFX(21);
    private final double metersPerangle = (1/0.0263671875) * (1/360);// 馬達一圈轉0.026圈*360
    private final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(0);
    private final AnalogInput IRSensor = new AnalogInput(0);

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
        slot0Configs.kP = 17.0; // A position error of 2.5 rotations results in 12 V output
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

        head.getConfigurator().apply(talonFXConfigs);
        intake.getConfigurator().apply(talonFXConfigs);
    }

    public void Magicgo(double Position) {
        head.setControl(m_request.withPosition(Position));
    }

    public Command magicgocCommand(double Position){
        return run(() -> this.Magicgo(Position));
    }
    public Command coarlintakeexecute() {
        return Commands.sequence(
                Commands.run(() -> this.intakexcute(4), this),
                new WaitCommand(0.75),
                Commands.run(() -> this.intakexcute(7), this),
                new WaitCommand(0.75)
        );
    }
    public Command alagelintakeexecute() {
        return Commands.run(() -> this.intakexcute(-7),this);
    }
    public Command alageput() {
        return Commands.runEnd(() -> this.intakexcute(7), this::intakestop, this);
    }
    public Command intakebackexecute() {
        return Commands.runEnd(() -> this.intakebackexcute(), this::intakestop, this);
    }
    public Command intakeputcmd() {
        return Commands.runEnd(() -> this.intakeput(), this::intakestop, this);
    }
    public Command intakestopCmd(){
        return Commands.run(() -> this.intakestop(), this);
    }
    public void intakestop() {
        this.intake.setVoltage(0.0);
    }
    public void headstop(){
        this.head.stopMotor();
    }
    public void intakexcute(double volt){
        this.intake.setVoltage(volt);
    }
    public void  intakebackexcute(){
        this.intake.setVoltage(-1);
    }
    public void  intakeput(){
        this.intake.setVoltage(-6);
    }
    public void intakesetVoltage(double volt){
        this.intake.setVoltage(volt);
    }

    public boolean isCoralIn() {
        return this.IRSensor.getVoltage() <= 1.0 ? true : false;
    }
    public boolean isCoralout() {
        return this.IRSensor.getVoltage() >= 1.0 ? true : false;
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("headencoder", this.head.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("IRValue", this.IRSensor.getVoltage());
    }
}
