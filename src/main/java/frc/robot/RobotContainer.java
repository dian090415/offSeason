
package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Head;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Drivetrain.SwerveSubsystem;
import frc.robot.commands.ArmCmd;
import frc.robot.commands.DriveCmd;
import frc.robot.commands.ElevatorCmd;
import frc.robot.commands.HeadCmd;

public class RobotContainer {

  private final Controller controller = new Controller();

  private final Elevator Elevator = new Elevator();
  private final SwerveSubsystem Swerve = new SwerveSubsystem();
  private final Arm arm = new Arm();
  private final Head Head = new Head();
  private final Superstructure Superstructure = new Superstructure(arm, Elevator, Head);

  private final ElevatorCmd ElevatorCmd = new ElevatorCmd(Elevator, controller);
  private final HeadCmd HeadCmd = new HeadCmd(Head, controller);
  private final ArmCmd armCmd = new ArmCmd(arm, controller);

  public RobotContainer() {
    this.Swerve.setDefaultCommand(
        new DriveCmd(Swerve,
            () -> -this.controller.getLeftY(),
            () -> this.controller.getLeftX(),
            () -> this.controller.getRightX(),
            () -> false));

    this.Elevator.setDefaultCommand(this.ElevatorCmd);
    this.Head.setDefaultCommand(this.HeadCmd);
    this.arm.setDefaultCommand(this.armCmd);
    this.configBindings();

  }

  public void configBindings() {
    this.controller.L1()
        .onTrue(this.Superstructure.levelCommand(1));
    this.controller.L2()
        .onTrue(this.Superstructure.levelCommand(2));
    this.controller.L3()
        .onTrue(this.Superstructure.levelCommand(3));
    this.controller.L4()
        .onTrue(this.Superstructure.levelCommand(4));
    this.controller.go()
        .onTrue(this.Superstructure.movetoLevel());
      // this.controller.L3()
      //     .onTrue(this.Elevator.startCommand());
      // this.controller.L2()
      //     .onTrue(this.Elevator.stopCommand());
      // this.controller.go()
      //     .onTrue(this.Elevator.sysIdElevatorTest());
  }
  

  public Command getAutonomousCommand() {
    return null;
  }
}