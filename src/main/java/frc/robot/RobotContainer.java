
package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.commands.ElevatorCmd;

public class RobotContainer {

  private final Controller controller = new Controller();

  private final Elevator Elevator = new Elevator();

  private final ElevatorCmd ElevatorCmd = new ElevatorCmd(Elevator, controller);

  public RobotContainer() {
    this.Elevator.setDefaultCommand(this.ElevatorCmd);
    this.configBindings();

  }

  public void configBindings() {
    this.controller.L1()
        .onTrue(this.Elevator.levelCommand(1));
    this.controller.L2()
        .onTrue(this.Elevator.levelCommand(2));
    this.controller.L3()
        .onTrue(this.Elevator.levelCommand(3));
    this.controller.L4()
        .onTrue(this.Elevator.levelCommand(4));
    this.controller.go()
        .onTrue(this.Elevator.gogoal());
  }

  public Command getAutonomousCommand() {
    return null;
  }
}