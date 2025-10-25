package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final TalonFX intake = new TalonFX(21);

    public StatusSignal<Angle> m_positionSig = intake.getPosition();
    public StatusSignal<Voltage> m_voltageSig = intake.getMotorVoltage();


    public Intake() {
        this.configureMotors();
    }

    private void configureMotors() {
        // in init function
        var talonFXConfigs = new TalonFXConfiguration();

        // 電流限制
        talonFXConfigs.CurrentLimits.withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(70.0)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(50.0);

        // Brake 模式
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // 馬達正反轉
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        talonFXConfigs.CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40);

        // 套用設定
        intake.getConfigurator().apply(talonFXConfigs);
    }

    public Command voltage(DoubleSupplier voltage) {
        return this.run(() -> intake.setControl(voltageRequest.withOutput(voltage.getAsDouble())));
    }

    public Command voltage(double voltage) {
        return voltage(() -> voltage);
    }

    public double getVoltage() {
        m_voltageSig.refresh();//要求更新這個訊號的最新值
        return m_voltageSig.getValueAsDouble();
    }

    public Command stop() {
        return voltage(0);
    }

    public Command inCoral() {
        return voltage(8.0);
    }

    public Command inCoralSlow() {
        return voltage(2.5);
    }

    public Command outCoral() {
        return voltage(-6.0);

    }

    public Command outCoralReverse() {
        return voltage(6.0);

    }

    public Command outCoralSlow() {
        return voltage(-4.0);

    }

    public Command inAlgae() {
        return voltage(-10.0);
    }

    public Command outAlgae() {
        return voltage(10);// HandConstants.OUT_ALGAE_VOLTAGE);
    }

    public Command outAlgaeSlow() {
        return voltage(2.0);
    }
}
