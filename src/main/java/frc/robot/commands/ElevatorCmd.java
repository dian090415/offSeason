package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorCmd extends Command {
	private final Elevator Elevator;
	private final XboxController controller;

	public ElevatorCmd(Elevator Elevator , XboxController controller){
		this.Elevator = Elevator;
		this.controller = controller;
		this.addRequirements(this.Elevator);
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		double speed = MathUtil.applyDeadband(this.controller.getLeftY(),0.05) * 8;
		this.Elevator.setVoltage(speed);
	}

	@Override
	public void end(boolean interrupted) {
		this.Elevator.stop();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}