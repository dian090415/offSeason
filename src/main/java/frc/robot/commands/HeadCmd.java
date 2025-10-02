package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Head;

public class HeadCmd extends Command {
	private final Head Head;
	private final XboxController controller;

	public HeadCmd(Head Head , XboxController controller){
		this.Head = Head;
		this.controller = controller;
		this.addRequirements(this.Head);
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		double speed = MathUtil.applyDeadband(this.controller.getLeftY(),0.05) * 5;
        // this.Head.setVoltage(speed);
		// this.Head.intakesetVoltage(speed);
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}