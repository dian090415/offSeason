// package frc.robot.subsystems;

// import java.util.Map;

// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.Arm.Arm;
// import frc.robot.subsystems.NewVision.VisionFuser;
// import frc.robot.subsystems.NewVision.VisionFuser.VisionConstants;
// import frc.robot.subsystems.NewDrive.*;
// import frc.robot.subsystems.NewVision.Vision;
// import frc.robot.commands.DriveToPoseCommand;

// public class Superstructure extends SubsystemBase {

//     private static class ReefConstants {
//         public static final Map<Integer, Pose2d> REEF_TARGETS = Map.ofEntries(
//             Map.entry(6, new Pose2d(13.257, 3.676, new Rotation2d(2.095))),
//             Map.entry(7, new Pose2d(13.460, 4.020, new Rotation2d(3.139))),
//             Map.entry(8, new Pose2d(13.257, 4.366, new Rotation2d(-2.095))),
//             Map.entry(9, new Pose2d(12.861, 4.366, new Rotation2d(-1.047))),
//             Map.entry(10, new Pose2d(12.657, 4.021, new Rotation2d(0.000))),
//             Map.entry(11, new Pose2d(12.861, 3.677, new Rotation2d(1.047))),
//             Map.entry(17, new Pose2d(4.292, 3.677, new Rotation2d(1.047))),
//             Map.entry(18, new Pose2d(4.090, 4.021, new Rotation2d(0.000))),
//             Map.entry(19, new Pose2d(4.292, 4.366, new Rotation2d(-1.047))),
//             Map.entry(20, new Pose2d(4.689, 4.366, new Rotation2d(-2.095))),
//             Map.entry(21, new Pose2d(4.889, 4.021, new Rotation2d(3.141))),
//             Map.entry(22, new Pose2d(4.690, 3.677, new Rotation2d(2.095)))
//         );
//     }

//     public final Arm arm;
//     public final Intake intake;
//     private final driveIOHardware driveIO = new driveIOHardware();
//     private final drive drive = new drive(driveIO);
//     private final SwerveDrivePoseEstimator poseEstimator = drive.poseEstimator();
//     private final VisionFuser visionFuser = new VisionFuser(
//         VisionConstants.cameraTransforms, poseEstimator);
//     private final Vision vision = new Vision(drive);

//     private final DriveToPoseCommand driveToPoseCommand;

//     public Superstructure(Arm m_arm, Intake m_intake) {
//         this.arm = m_arm;
//         this.intake = m_intake;

//         this.driveToPoseCommand = new DriveToPoseCommand(
//             drive,
//             this::autoAligngoal, // Supplier<Pose2d>
//             drive::getPose       // Supplier<Pose2d>
//         );
//     }

//     public int tagId() {
//         if (vision.getLeftTagId() == vision.getRightTagId()) {
//             return vision.getLeftTagId();
//         } else if (visionFuser.apriltagId() != -1) {
//             return visionFuser.apriltagId();
//         } else {
//             return -1;
//         }
//     }

//     public Pose2d autoAligngoal() {
//         return ReefConstants.REEF_TARGETS.get(this.tagId());
//     }

//     public Command autoAlign() {
//         Pose2d goal = autoAligngoal();
//         if (goal == null) {
//             return Commands.none(); // 沒有偵測到有效 Tag
//         }
//         return new DriveToPoseCommand(
//             drive,
//             this::autoAligngoal,
//             drive::getPose
//         );
//     }
// }