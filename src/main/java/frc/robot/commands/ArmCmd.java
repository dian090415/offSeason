package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MainPivotS;

public class ArmCmd extends Command {
	private final MainPivotS Arm;
	private final XboxController controller;

	public ArmCmd(MainPivotS Arm , XboxController controller){
		this.Arm = Arm;
		this.controller = controller;
		this.addRequirements(this.Arm);
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		double speed = MathUtil.applyDeadband(this.controller.getLeftY(),0.05) * 8;
        // this.Arm.setVoltage(speed);
	}

	@Override
	public void end(boolean interrupted) {;
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}