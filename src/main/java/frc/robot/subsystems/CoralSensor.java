package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSensor extends SubsystemBase{
        private final AnalogInput IRSensor = new AnalogInput(0);
    public CoralSensor(){

    }
    public boolean isCoralIn() {
        return this.IRSensor.getVoltage() <= 1.0 ? true : false;
    }

    public boolean isCoralout() {
        return this.IRSensor.getVoltage() >= 1.0 ? true : false;
    }
}
