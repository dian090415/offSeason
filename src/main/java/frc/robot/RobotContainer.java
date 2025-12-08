
package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import java.security.PublicKey;
import java.util.Map;

import org.photonvision.simulation.VisionSystemSim;

import com.ctre.phoenix6.swerve.jni.SwerveJNI.DriveState;
import com.google.flatbuffers.Constants;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.MainPivotS;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Head;
import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Arm.AbstractArm;
import frc.robot.subsystems.Arm.AbstractArm.ArmPosition;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.NewControl.Controller;
import frc.robot.subsystems.NewControl.NewController;
import frc.robot.subsystems.NewDrive.drive;
import frc.robot.subsystems.NewDrive.driveIOHardware;
import frc.robot.subsystems.NewVision.Vision;
import frc.robot.subsystems.NewVision.VisionFuser;
import frc.robot.subsystems.NewVision.VisionFuser.VisionConstants;
import frc.robot.commands.*;
import frc.robot.Constants.ReefConstants;
import frc.robot.subsystems.CoralSensor;

public class RobotContainer {

  // ------------------------------新東西---------------------------

  private final NewController main_driver = new NewController(0);
  private final NewController co_driver = new NewController(1);
  private final Controller controller = new Controller(2);

  private final Elevator Elevator = new Elevator();

  private final driveIOHardware driveIO = new driveIOHardware();
  private final drive drive = new drive(driveIO);

  private final Vision vision = new Vision(drive);
  private final VisionFuser visionFuser = new VisionFuser(
      VisionConstants.cameraTransforms, drive);

  private final AutoFactory autoFactory;

  private final MainPivotS MainPivotS = new MainPivotS();
  private final Head Head = new Head();
  private final Arm arm = new Arm();
  private final Intake intake = new Intake();
  private final Superstructure superstructure = new Superstructure(arm, intake, drive, vision, visionFuser);

  public Pose2d goalPose2d;

  public boolean ifmechanismreverse;

  public int Level;

  public boolean intakemodel = false;

  private boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
  }

  // -----------------------------------------------------------------

  public RobotContainer() {

    goalPose2d = new Pose2d(0, 0, null);

    autoFactory = new AutoFactory(
        drive::getPose, // A function that returns the current robot pose
        drive::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
        drive::followTrajectory, // The drive subsystem trajectory follower
        isRedAlliance(), // If alliance flipping should be enabled
        drive // The drive subsystem
    );
    autoFactory.cache().clear();

    autoFactory
        .bind("L4 Prepare", this.autoL4Prepare())
        .bind("L4 put", this.autoL4put())
        .bind("all take back", this.autoalltakeback())
        .bind("intake down", this.autointakedown())
        .bind("intake suck", this.autointakesuck())
        .bind("intake stop", this.autointakestop())
        .bind("intake up", this.autointakeup());

    this.drive.setDefaultCommand(new NewDriveCmd(drive, main_driver, co_driver));
    // this.Swerve.setDefaultCommand(
    // new DriveCmd(
    // Swerve,
    // () -> -this.driver.getLeftY(),
    // () -> this.driver.getLeftX(),
    // () -> this.driver.getRightX(),
    // () -> true));

    // new Trigger(() -> Math.abs(this.controller.getLeftY()) > 0.1 ||
    // Math.abs(this.controller.getLeftX()) > 0.1 ||
    // Math.abs(this.controller.getRightX()) > 0.1)
    // .whileTrue(new DriveCmd(
    // Swerve,
    // () -> -this.controller.getLeftY(),
    // () -> this.controller.getLeftX(),
    // () -> this.controller.getRightX(),
    // () -> true));

    this.configBindings();
  }

  public void configBindings() {
    this.co_driver.Intake()
        .onTrue(this.superstructure.intake())
        .onFalse(this.superstructure.intakestop());
    this.co_driver.Intakemodel()
        .onTrue(this.superstructure.setintakemodel());
    this.co_driver.put()
        .whileTrue(this.superstructure.put())
        .onFalse(this.superstructure.STOW());
    this.co_driver.LeftAilgn()
        .whileTrue(this.superstructure.LeftautochoserAlign());
    this.co_driver.RightAilgn()
        .whileTrue(this.superstructure.RightautochoserAlign());
    this.controller.L1()
        .onTrue(this.superstructure.setlevelCommand(1));
    this.controller.L2()
        .onTrue(this.superstructure.setlevelCommand(2));
    this.controller.L3()
        .onTrue(this.superstructure.setlevelCommand(3));
    this.controller.L4()
        .onTrue(this.superstructure.setlevelCommand(4));
    // this.co_driver.L1()
    // .onTrue(this.arm.goToPosition(Arm.Positions.L1));
    // this.co_driver.L2()
    // .onTrue(this.arm.goToPosition(Arm.Positions.L2));
    // this.co_driver.L3()
    // .onTrue(this.arm.goToPosition(Arm.Positions.L3));
    // this.co_driver.L4()
    // .onTrue(this.arm.goToPosition(Arm.Positions.L4));

    this.main_driver.zeroHeading().onTrue(new InstantCommand(() -> drive.zeroHeading()));
    this.co_driver.zeroHeading().onTrue(new InstantCommand(() -> drive.zeroHeading()));
  }

  public VisionFuser getVisionFuser() {
    return this.visionFuser;
  }

  public Command setsafe(){
    return this.superstructure.SetSafe();
  }

  public Command getAutonomousCommand() {
    return autotest();
  }

  // -----------------auto---------------------------------------
  public Command autotest() {
    return Commands.sequence(
        autoFactory.resetOdometry("testone"),
        autoFactory.trajectoryCmd("testone"));
  }

  public Command autoL4Prepare() {
    SmartDashboard.putBoolean("autoL4Preparerun", true);
    return Commands.sequence(this.arm.goToPosition(Arm.Positions.L4));
  }

  public Command autoL4put() {
    return Commands.sequence(
        this.intake.outCoral(),
        Commands.waitSeconds(0.5));
  }

  public Command autoalltakeback() {
    return Commands.parallel(this.intake.stop(), this.arm.goToPosition(Arm.Positions.CORAL_STOW));
  }

  public Command autointakedown() {
    return Commands.sequence(this.arm.goToPosition(Arm.Positions.GROUND_CORAL));
  }

  public Command autointakesuck() {
    return Commands.sequence(this.intake.inCoral());
  }

  public Command autointakestop() {
    return Commands.sequence(this.intake.stop());
  }

  public Command autointakeup() {
    return Commands.sequence(this.arm.goToPosition(Arm.Positions.CORAL_STOW));
  }
}