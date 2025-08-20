package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Driver extends XboxController {

    public Driver() {
        super(0);
    }

    public Trigger Intake(){
        return new Trigger(this::getintakeTrigger);
    }
    
    public Trigger zero(){
        return new Trigger(this::getAButton);
    }

    public boolean getintakeTrigger() {
        return this.getLeftTriggerAxis() >= 0.3 ? true : false;
    }
    public Trigger put(){
        return new Trigger(this::getputTrigger);
    }
    public boolean getputTrigger() {
        return this.getRightTriggerAxis() >= 0.3 ? true : false;
    }
    public Trigger Leftgo(){
        return new Trigger(this::getLeftBumperButton);
    }
    public Trigger Rightgo(){
        return new Trigger(this::getLeftBumperButton);
    }
    public Trigger restgryo(){
        return new Trigger(this::getXButton);
    }
}