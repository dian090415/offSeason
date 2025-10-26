
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.MainPivotS;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Head;
import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Arm.AbstractArm;
import frc.robot.subsystems.Arm.AbstractArm.ArmPosition;
import frc.robot.subsystems.Arm.Arm;
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

  private final Elevator Elevator = new Elevator();

  private final driveIOHardware driveIO = new driveIOHardware();
  private final drive drive = new drive(driveIO);

  private final Vision vision = new Vision(drive);
  private final SwerveDrivePoseEstimator poseEstimator = drive.poseEstimator();
  private final VisionFuser visionFuser = new VisionFuser(
      VisionConstants.cameraTransforms, poseEstimator);

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

  private boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
  }

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
        .whileTrue(this.intake())
        .onFalse(this.arm.goToPosition(Arm.Positions.CORAL_STOW));
    this.co_driver.LeftAilgn()
        .whileTrue(this.LeftautochoserAlign());
    this.co_driver.RightAilgn()
        .whileTrue(this.RightautochoserAlign());
    this.main_driver.zeroHeading().onTrue(new InstantCommand(() -> drive.zeroHeading()));
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
      return goalpPose2d;
    } else {
      this.ifmechanismreverse = true;
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
    goalPose2d = ifreverse(goal);

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
    goalPose2d = ifreverse(goal);

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
    goalPose2d = ifreverse(goal);

    return new DriveToPoseCommand(
        drive,
        () -> finalGoal,
        drive::getPose);
  }

  public Command LeftautochoserAlign() {
    return Commands.either(LeftautoAlign(), this.alageautoAlign(), () -> this.coralsensor.isCoralIn());
  }

  public Command RightautochoserAlign() {
    return Commands.either(RightautoAlign(), this.alageautoAlign(), () -> this.coralsensor.isCoralIn());
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
    if (this.drive.getPose().getTranslation().getDistance(this.goalPose2d.getTranslation()) < 35) {
      return true;
    } else {
      return false;
    }
  }

  public Command intake() {
    return Commands.either(Commands.sequence(
        this.arm.goToPosition(Arm.Positions.GROUND_CORAL), this.intake.inCoral()),
        Commands.parallel(this.intake.stop(),
            this.arm.goToPosition(Arm.Positions.CORAL_STOW)),
        () -> this.coralsensor.isCoralIn());
  }

  // -----------------auto---------------------------------------
  public Command autotest() {
    return Commands.sequence(
        autoFactory.resetOdometry("testone"),
        autoFactory.trajectoryCmd("testone"));
  }
}