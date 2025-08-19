package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Controller extends XboxController {
    public Controller() {
        super(1);
    }

    public Trigger L4() {
        return new Trigger(this::getYButton);
    }
    public Trigger L3() {
        return new Trigger(this::getXButton);
    }
    public Trigger L2(){
        return new Trigger(this::getBButton);
    }
    public Trigger L1(){
        return new Trigger(this::getAButton);
    }
    public Trigger go(){
        return new Trigger(this::getRightBumperButton);
    }
}
