package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSensor extends SubsystemBase{
        private final AnalogInput IRSensor = new AnalogInput(1);
    public CoralSensor(){

    }
    public boolean isCoralIn() {
        return this.IRSensor.getVoltage() <= 3.0 ? true : false;
    }

    public boolean isCoralout() {
        return this.IRSensor.getVoltage() >= 3.0 ? true : false;
    }
    @Override
    public void periodic(){
        SmartDashboard.putBoolean("isCoralIn", isCoralIn());
        SmartDashboard.putNumber("CoralSensor", IRSensor.getVoltage());
    }
}
