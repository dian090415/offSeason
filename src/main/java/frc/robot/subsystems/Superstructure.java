package frc.robot.subsystems;

import java.security.PublicKey;
import java.util.concurrent.BlockingDeque;
import java.util.function.BooleanSupplier;

import javax.sound.midi.Sequencer;

import com.google.gson.annotations.Until;

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
    private final double[] Armchoose = {0.0 ,0.0};//可以L4，不行
    public double headputcoral = 0.0;
    public double headintake = 5.88154296875;
    public double headkeep = 1.0;
    public double armdown = -0.32861328125;
    public double armkeep = 26.4033203125;
    public int Level;
    public Boolean invertarm;
    
        public Superstructure(Arm arm, Elevator elevator, Head head) {
            this.elevator = elevator;
            this.arm = arm;
            this.head = head;
            this.Level = 0;
            this.invertarm = false;
        }
    
        public Command test() {
            return Commands.sequence(
                this.elevator.moveToPositionCommand()
            );
        }

        public Command intakedown(){
            return Commands.sequence(
                Commands.runOnce(() -> this.setLevel(0), this),

                Commands.parallel(
                this.arm.moveToCommand(armdown),
                this.head.magicgocCommand(headintake),
                this.elevator.moveToPositionCommand()
                ).until(() -> this.arm.armatgoal(armdown)),
                this.head.intakeexecute().until(() -> this.head.isCoralIn()),
                this.head.intakebackexecute().withTimeout(0.5)
            );
        }
        public Command allkeep(){
            return Commands.sequence(
                Commands.runOnce(() -> this.setLevel(0), this),
                this.arm.moveToCommand(armkeep)
                .alongWith(this.elevator.moveToPositionCommand())
                .alongWith(this.head.magicgocCommand(headkeep))
            );
        }
        public Command Elevatorgo(){
            if(invertarm){
                return CannotL4levelput();
            }else{
                return CanL4levelput();
            }
        }
        public Command CanL4levelput(){
            return Commands.sequence(
                this.arm.moveToCommand(this.Armchoose[0])
                .alongWith(this.head.magicgocCommand(headintake))
                .alongWith(this.elevator.moveToPositionCommand()),
                new WaitUntilCommand(() -> this.elevator.atgoal()),
                this.head.magicgocCommand(headputcoral)
            );
        }

        public Command CannotL4levelput(){
            return Commands.sequence(
                this.arm.moveToCommand(this.Armchoose[1])
                .alongWith(this.head.magicgocCommand(headintake))
                .alongWith(this.elevator.moveToPositionCommand())
            );
        }
        public Command putCoral() {
            return Commands.sequence(
                this.head.intakebackexecute().until(() -> this.head.isCoralout()),
                this.allkeep()
            );
        }


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
        this.Level = Level;
        this.elevator.setLevel(Level);
    }
    public void stopMotor(){
        this.head.headstop();
        this.arm.stop();
        this.elevator.stop();
        this.head.intakestop();
    }
    public Command stop(){
        return runOnce(() -> this.stopMotor());
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Level", this.Level);
    }
}
