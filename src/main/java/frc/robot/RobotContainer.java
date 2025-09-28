
package frc.robot;

import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.MainPivotS;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Head;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.NewControl.NewController;
import frc.robot.subsystems.NewDrive.drive;
import frc.robot.subsystems.NewDrive.driveIOHardware;
import frc.robot.subsystems.NewVision.Vision;
import frc.robot.commands.ArmCmd;
import frc.robot.commands.ElevatorCmd;
import frc.robot.commands.HeadCmd;
import frc.robot.commands.NewDriveCmd;

public class RobotContainer {

  private final Controller controller = new Controller();


  private final Driver driver = new Driver();
  
  // ------------------------------新東西---------------------------

  private final NewController main_driver = new NewController(0);
  private final NewController co_driver = new NewController(1);
  private final Vision vision = new Vision();
  private final Elevator Elevator = new Elevator();

  private final driveIOHardware driveIO = new driveIOHardware();
  private final drive drive = new drive(driveIO, vision);

  // -----------------------------------------------------------------

  private final VisionSubsystem visionSubsystem = new VisionSubsystem(Swerve);
  private final MainPivotS arm = new MainPivotS();
  private final Head Head = new Head();
  private final Superstructure Superstructure = new Superstructure(arm, Elevator, Head);

  private final ElevatorCmd ElevatorCmd = new ElevatorCmd(Elevator, controller);
  private final HeadCmd HeadCmd = new HeadCmd(Head, controller);
  private final ArmCmd armCmd = new ArmCmd(arm, controller);

  public RobotContainer() {

        this.drive.setDefaultCommand(new NewDriveCmd(drive, main_driver, co_driver));
    // this.Swerve.setDefaultCommand(
    //     new DriveCmd(
    //         Swerve,
    //         () -> -this.driver.getLeftY(),
    //         () -> this.driver.getLeftX(),
    //         () -> this.driver.getRightX(),
    //         () -> true));

    // new Trigger(() -> Math.abs(this.controller.getLeftY()) > 0.1 ||
    //     Math.abs(this.controller.getLeftX()) > 0.1 ||
    //     Math.abs(this.controller.getRightX()) > 0.1)
    //     .whileTrue(new DriveCmd(
    //         Swerve,
    //         () -> -this.controller.getLeftY(),
    //         () -> this.controller.getLeftX(),
    //         () -> this.controller.getRightX(),
    //         () -> true));



    this.configBindings();
    visionSubsystem.periodic();
  }

  public void configBindings() {
    // this.controller.L1()
    // .onTrue(this.Superstructure.levelCommand(1));
    // this.controller.L2()
    // .onTrue(this.Superstructure.levelCommand(2));
    // this.controller.L3()
    // .onTrue(this.Superstructure.levelCommand(3));
    // this.controller.L4()
    // .onTrue(this.Superstructure.levelCommand(4));
    // this.driver.Intake()
    //     .onTrue(this.Superstructure.test())
    //     .onFalse(this.Elevator.hold());
    // this.driver.restgryo()
    // .onTrue(this.Swerve.resetGyro());
    // this.controller.coral()
    // .onTrue(this.Superstructure.setcmd(1));
    // this.controller.alage()
    // .onTrue(this.Superstructure.setcmd(0));
    // this.driver.Leftgo()
    // .onTrue(this.Superstructure.Elevatorgo());
    // this.driver.put()
    // .onTrue(this.Superstructure.putCoral());
    // this.driver.Rightgo()
    // .onTrue(this.Superstructure.alageintakedown())
    // .onFalse(this.Superstructure.alagekeep());
    // this.controller.high()
    // .onTrue(this.Superstructure.highalage())
    // .onFalse(this.Superstructure.alagekeep());
    // this.controller.low()
    // .onTrue(this.Superstructure.lowalage())
    // .onFalse(this.Superstructure.alagekeep());
    // this.driver.net()
    // .onTrue(this.Superstructure.net())
    // .onFalse(this.Superstructure.coralkeep());

    // this.driver.put()
    // .onTrue(this.Superstructure.Elevatorgo());
    // this.controller.go()
    // .onTrue(this.Superstructure.test());
    // this.controller.L3()
    // .onTrue(this.Elevator.startCommand());
    // this.controller.L2()
    // .onTrue(this.Elevator.stopCommand());
    // this.controller.go()
    // .onTrue(this.Elevator.sysIdElevatorTest());
  }

  public Command getAutonomousCommand() {
    return null;
  }
}