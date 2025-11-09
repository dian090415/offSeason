
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
  // private final Superstructure Superstructure = new Superstructure(arm,intake);

  private final DriveToPoseCommand driveToPoseCommand;

  private final CoralSensor coralsensor = new CoralSensor();

  public Pose2d goalPose2d;

  public boolean ifmechanismreverse;

  public int Level;

  private boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
  }

  private final Field2d field = new Field2d();

  // -----------------------------------------------------------------

  public RobotContainer() {

    goalPose2d = new Pose2d(0, 0, null);

    this.driveToPoseCommand = new DriveToPoseCommand(
        drive,
        this::alageautoAligngoal, // Supplier<Pose2d>
        drive::getPose // Supplier<Pose2d>
    );

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
        .onTrue(this.intake())
        .onFalse(this.intakestop());
    this.co_driver.LeftAilgn()
        .whileTrue(this.LeftautochoserAlign());
    this.co_driver.RightAilgn()
        .whileTrue(this.RightautochoserAlign());
    this.controller.L1()
        .onTrue(this.setlevelCommand(1));
    this.controller.L2()
        .onTrue(this.setlevelCommand(2));
    this.controller.L3()
        .onTrue(this.setlevelCommand(3));
    this.controller.L4()
        .onTrue(this.setlevelCommand(4));

    this.main_driver.zeroHeading().onTrue(new InstantCommand(() -> drive.zeroHeading()));
    this.co_driver.zeroHeading().onTrue(new InstantCommand(() -> drive.zeroHeading()));
  }

  public Command getAutonomousCommand() {
    return autotest();
  }

  // ------------------Superstructure---------------------------
  public int tagId() {
    if (vision.getLeftTagId() == vision.getRightTagId()) {
      return vision.getLeftTagId();
    } else if (visionFuser.apriltagId() != -1) {
      return visionFuser.apriltagId();
    } else {
      return -1;
    }
  }

  public Pose2d alageautoAligngoal() {
    return ReefConstants.REEF_alage.get(this.tagId());
  }

  public Pose2d LeftautoAligngoal() {
    return ReefConstants.REEF_Left.get(this.tagId());
  }

  public Pose2d RightautoAligngoal() {
    return ReefConstants.REEF_Right.get(this.tagId());
  }

  public Pose2d ifreverse(Pose2d goalpPose2d) {
    Rotation2d currentRot = this.drive.getPose().getRotation(); // 機器人目前角度
    Rotation2d targetRotFlipped = goalpPose2d.getRotation().plus(Rotation2d.fromDegrees(180));

    // 計算兩個角度差的絕對值
    double diffNormal = Math.abs(currentRot.minus(goalpPose2d.getRotation()).getDegrees());
    double diffFlipped = Math.abs(currentRot.minus(targetRotFlipped).getDegrees());

    if (diffNormal <= diffFlipped) {
      this.ifmechanismreverse = false;
      field.getObject("goalGoal").setPose(goalpPose2d);
      SmartDashboard.putData("Field", field);
      return goalpPose2d;
    } else {
      this.ifmechanismreverse = true;
      field.getObject("goalGoal").setPose(new Pose2d(
        goalpPose2d.getX(),
        goalpPose2d.getY(),
        goalpPose2d.getRotation().plus(Rotation2d.fromDegrees(180))));
      SmartDashboard.putData("Field", field);
      return new Pose2d(
          goalpPose2d.getX(),
          goalpPose2d.getY(),
          goalpPose2d.getRotation().plus(Rotation2d.fromDegrees(180)));
    }
  }

  public Command alageautoAlign() {
    Pose2d goal = alageautoAligngoal();
    if (goal == null) {
      goal = getalageClosestReefTagId(); // 沒有偵測到有效 Tag
    }

    Pose2d finalGoal = ifreverse(goal); // lambda裡要用final或effectively final變數

    return new DriveToPoseCommand(
        drive,
        () -> finalGoal,
        drive::getPose);
  }

  public Command LeftautoAlign() {
    Pose2d goal = LeftautoAligngoal();
    if (goal == null) {
      goal = getLeftClosestReefTagId(); // 沒有偵測到有效 Tag
    }

    Pose2d finalGoal = ifreverse(goal); // lambda裡要用final或effectively final變數

    return new DriveToPoseCommand(
        drive,
        () -> finalGoal,
        drive::getPose);
  }

  public Command RightautoAlign() {
    Pose2d goal = RightautoAligngoal();
    if (goal == null) {
      goal = getRightClosestReefTagId(); // 沒有偵測到有效 Tag
    }

    Pose2d finalGoal = ifreverse(goal); // lambda裡要用final或effectively final變數

    return new DriveToPoseCommand(
        drive,
        () -> finalGoal,
        drive::getPose);
  }

  public Command LeftautochoserAlign() {
    return Commands.either(Leftreef(), this.alageautoAlign(), () -> this.coralsensor.isCoralIn());
  }

  public Command RightautochoserAlign() {
    return Commands.either(Rightreef(), this.alageautoAlign(), () -> this.coralsensor.isCoralIn());
  }

  public Command Leftreef() {
    return Commands.parallel(this.Levelreef(), this.LeftautoAlign());
  }

  public Command Rightreef() {
    return Commands.parallel(this.Levelreef(), this.RightautoAlign());
  }

  public Pose2d getalageClosestReefTagId() {
    double minDistance = Double.POSITIVE_INFINITY; // 初始設為無限大
    int closestId = -1; // 預設沒找到

    for (Map.Entry<Integer, Pose2d> entry : ReefConstants.REEF_alage.entrySet()) {
      Pose2d tagPose = entry.getValue();
      double distance = this.drive.getPose().getTranslation().getDistance(tagPose.getTranslation());

      if (distance < minDistance) {
        minDistance = distance;
        closestId = entry.getKey();
      }
    }

    return ReefConstants.REEF_alage.get(closestId);
  }

  public Pose2d getLeftClosestReefTagId() {
    double minDistance = Double.POSITIVE_INFINITY; // 初始設為無限大
    int closestId = -1; // 預設沒找到

    for (Map.Entry<Integer, Pose2d> entry : ReefConstants.REEF_Left.entrySet()) {
      Pose2d tagPose = entry.getValue();
      double distance = this.drive.getPose().getTranslation().getDistance(tagPose.getTranslation());

      if (distance < minDistance) {
        minDistance = distance;
        closestId = entry.getKey();
      }
    }

    return ReefConstants.REEF_Left.get(closestId);
  }

  public Pose2d getRightClosestReefTagId() {
    double minDistance = Double.POSITIVE_INFINITY; // 初始設為無限大
    int closestId = -1; // 預設沒找到

    for (Map.Entry<Integer, Pose2d> entry : ReefConstants.REEF_Right.entrySet()) {
      Pose2d tagPose = entry.getValue();
      double distance = this.drive.getPose().getTranslation().getDistance(tagPose.getTranslation());

      if (distance < minDistance) {
        minDistance = distance;
        closestId = entry.getKey();
      }
    }

    return ReefConstants.REEF_Right.get(closestId);
  }

  public boolean ifclosegoal() {
    if (this.drive.getPose().getTranslation().getDistance(this.goalPose2d.getTranslation()) < 0.35) {
      return true;
    } else {
      return false;
    }
  }

  public Command intake() {
    return Commands.either(
        this.intakestop(),
        Commands.parallel(
            this.arm.goToPosition(Arm.Positions.GROUND_CORAL),
            this.intake.inCoral().until(() -> this.coralsensor.isCoralIn()).andThen(this.intake.stop())),
        () -> this.coralsensor.isCoralIn());
  }

  public Command intakestop() {
    return Commands.parallel(this.arm.goToPosition(Arm.Positions.CORAL_STOW), this.intake.stop());
  }

  public void setlevel(int level) {
    this.Level = level;
  }

  public Command setlevelCommand(int level) {
    return Commands.runOnce(() -> setlevel(level));
  }

  public ArmPosition LevelPosition() {
    switch (Level) {
      case 1:
        return Arm.Positions.L1;
      case 2:
        return ifmechanismreverse ? Arm.Positions.L2_OPP : Arm.Positions.L2;
      case 3:
        return ifmechanismreverse ? Arm.Positions.L3_OPP : Arm.Positions.L3;
      case 4:
        return ifmechanismreverse ? Arm.Positions.L4_OPP : Arm.Positions.L4;
      default:
        return null;
    }
  }

  public Command Levelreef() {
    return Commands.sequence(new WaitUntilCommand(() -> ifclosegoal()), this.arm.goToPosition(LevelPosition()));
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