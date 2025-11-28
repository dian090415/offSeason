package frc.robot.subsystems.NewControl;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class NewController extends XboxController {
    
    public NewController(int port) {
        super(port);
    }
    public Trigger LeftAilgn(){
        return new Trigger(this::getLeftBumperButton);
    }
    public Trigger RightAilgn(){
        return new Trigger(this::getRightBumperButton);
    }
    public Trigger zeroHeading() {
        return new Trigger(this::getAButton);
    }
    public Trigger Intake(){
        return new Trigger(this::getintakeTrigger);
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
    public Trigger Intakemodel(){
        return new Trigger(this::getXButton);
    }
    // public Trigger L4() {
    //     return new Trigger(this::getYButton);
    // }
    // public Trigger L3() {
    //     return new Trigger(this::getXButton);
    // }
    // public Trigger L2() {
    //     return new Trigger(this::getBButton);
    // }
    // public Trigger L1() {
    //     return new Trigger(this::getAButton);
    // }
}
