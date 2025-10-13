package frc.robot.subsystems.NewControl;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class NewController extends XboxController {
    
    public NewController(int port) {
        super(port);
    }

    public Trigger zeroHeading() {
        return new Trigger(this::getAButton);
    }
    public Trigger test() {
        return new Trigger(this::getAButton);
    }
    public Trigger Intake(){
        return new Trigger(this::getintakeTrigger);
    }
    public boolean getintakeTrigger() {
        return this.getLeftTriggerAxis() >= 0.3 ? true : false;
    }
}
