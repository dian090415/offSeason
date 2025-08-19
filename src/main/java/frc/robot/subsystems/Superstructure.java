package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import javax.sound.midi.Sequencer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class Superstructure extends SubsystemBase {
    private final Arm arm;
    private final Elevator elevator;
    private final Head head;
    private final double Elevatorsafemeter = 0.0;//TODO
    private final double Elevatorminmeter = 0.0;//TODO
    private final double Elevatormaxmeter = 0.0;//TODO  
    private final double Headsafemeter = 0.0;//TODO
    private final double[] headLevel = {0.0};//TODO 預設
    private final double[] Armchoose = {0.0};//TODO 預設
    public double headmeter = 0.25;
    public double intakedown = 0.0;
    
        public Superstructure(Arm arm, Elevator elevator, Head head) {
            this.elevator = elevator;
            this.arm = arm;
            this.head = head;
        }
    
        public Command movetoLevel() {
            return Commands.sequence(
                this.arm.moveToCommand(0)
            );
        }

        // public Command intakedown(){
        //     return Commands.sequence(
        //         this.arm.moveToCommand(intakedown)
        //         .alongWith(this.head.magicgocCommand(headmeter)),
        //         .this.head.intake()
        //     );
        // }


    //---------------head安全處理-------------
    public double headLevel(int Level){
        if(Math.abs(this.elevator.goal() - this.elevator.encoder()) >= 45){
            return this.headLevel[Level] - 0.25;
        }else if(Math.abs(this.elevator.goal() - this.elevator.encoder()) >= 10){
            return this.headLevel[Level] - 0.06;
        }else{
            return this.headLevel[Level];
        }
    }
    public Command headmeterCommand(int whatLevel){
        return runOnce(() -> this.headLevel(whatLevel));
    }
    public Command waitElevator(){
        return new WaitUntilCommand(() -> this.elevator.atgoal());
    }
    public Command levelCommand(int whatLevel) {
        return Commands.sequence(
            Commands.runOnce(() -> this.elevator.setLevel(whatLevel))
        );
    }
    public void setLevel(int Level){
        this.elevator.setLevel(Level);
    }
    public void stopMotor(){
        this.head.stop();
        this.arm.stop();
        this.elevator.stop();
    }
    public Command stop(){
        return runOnce(() -> this.stopMotor());
    }



}
