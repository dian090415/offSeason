package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

    private final TalonFX Lmain = new TalonFX(1);
    private final TalonFX Lfollow = new TalonFX(2);
    private final TalonFX Rmain = new TalonFX(3);
    private final TalonFX Rfollow = new TalonFX(4);
    private final double ratio = 0.0;// TODO
    private final double metersPerangle = 0.0;// TODO

    public Arm() {
        this.Config();
        Lfollow.setControl(new Follower(Lmain.getDeviceID(), true));
        Rmain.setControl(new Follower(Lmain.getDeviceID(), false));
        Rfollow.setControl(new Follower(Lmain.getDeviceID(), true));
    }

    public void Config() {
        // in init function
        var talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
        
        // set Motion Magic Expo settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0; // Unlimited cruise velocity
        motionMagicConfigs.MotionMagicExpo_kV = 0.12; // kV is around 0.12 V/rps
        motionMagicConfigs.MotionMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)

        // Brake 模式
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // 馬達正反轉
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        Lmain.getConfigurator().apply(talonFXConfigs);
        Rmain.getConfigurator().apply(talonFXConfigs);
        Lfollow.getConfigurator().apply(talonFXConfigs);
        Rfollow.getConfigurator().apply(talonFXConfigs);
    }
    
}
