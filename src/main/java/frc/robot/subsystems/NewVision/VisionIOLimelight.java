package frc.robot.subsystems.NewVision;
// package frc.robot.subsystems.vision;

// import java.lang.module.Configuration;
// import java.util.ArrayList;
// import java.util.Collection;
// import java.util.concurrent.TransferQueue;

// import edu.wpi.first.math.Pair;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.networktables.NetworkTablesJNI;
// import frc.robot.configs.CameraConfiguration;
// import frc.robot.constants.FieldConstants;
// import frc.robot.RobotState;

// import org.littletonrobotics.junction.Logger;

// public class VisionIOLimelight implements VisionIO {

//     // Constants

//     private static final double TAG_HEIGHT = Units.inchesToMeters(6.5);
//     private static final double HEIGHT_OF_TAG_OFF_GROUND = Units.inchesToMeters(8.87);

//     public static final double VERTICAL_RESOLUTION_IN_PIXELS = 800;
//     public static final double VERTICAL_FIELD_OF_VIEW_IN_DEGREES = 56.2;
//     public static final double LIMELIGHT_LATENCY_CONSTANT_SECONDS = 0.0;

//     public static final double LIMELIGHT_SCALAR_FACTOR = 38.12 / 35.99; // TODO

//     private final NetworkTableEntry valid, tx, tid, corner, throttleSet;

//     private final ArrayList<Pair<Double, Double>> cornerListPairs;

//     private final String limelightName;
//     private final CameraConfiguration.Location limelightLocation;
//     private final Translation2d cameraToRobotCenter;
//     private final double offsetBetweenApriltagBottomAndCamera;
//     private final Rotation2d limelightYaw;
//     private final double distanceScalarValue;

//     // NetworkTables

//     public VisionIOLimelight(CameraConfiguration conf) {
//         NetworkTableInstance inst = NetworkTableInstance.getDefault();
//         this.limelightYaw = Rotation2d.fromRadians(conf.LimelightMountingYawRad);
//         this.limelightName = conf.getName();
//         this.limelightLocation = conf.location;
//         this.cameraToRobotCenter = conf.getTranslationToRobotCenter();
//         this.offsetBetweenApriltagBottomAndCamera = HEIGHT_OF_TAG_OFF_GROUND - conf.LimelightHeightOffsetMeters;
//         this.distanceScalarValue = conf.LimelightDistanceScalarValue;

//         this.valid = inst.getTable(limelightName).getEntry("tv");
//         this.tx = inst.getTable(limelightName).getEntry("tx");
//         this.tid = inst.getTable(limelightName).getEntry("tid");
//         this.corner = inst.getTable(limelightName).getEntry("tcornxy");
//         this.throttleSet = inst.getTable(limelightName).getEntry("throttle_set");

//         this.cornerListPairs = new ArrayList<Pair<Double, Double>>();
//     }

//     // Synchronized 同步鎖：
//     // 在同一時間下只有單一線程可執行func
//     // 保護程式再多線程下的安全執行
//     // 避免多線程同時修改資源導致 "race condition"

//     @Override
//     public synchronized void updateInputs(VisionIOInputs inputs) {
//         inputs.horizontalAngleToTarget = Rotation2d.fromDegrees(tx.getDouble(HEIGHT_OF_TAG_OFF_GROUND));
//         inputs.hasTarget = valid.getDouble(0.0) == 1.0;
//         inputs.tagId = (int) tid.getDouble(-1);

//         this.cornerListPairs.clear();

//         var x = corner.getDoubleArray(new double[0]);

//         if (x.length >= 8) {
//             for (int i = 0; i < 7; i += 2) {
//                 this.cornerListPairs.add(new Pair<Double, Double>(x[i], x[i + 1]));
//             }
//         }

//         Logger.recordOutput(this.limelightName + "/Number of corners", this.cornerListPairs.size());

//         if (inputs.hasTarget && cornerListPairs.size() >= 4) {
//             inputs.targetHeight = calculateTargetHeightInPixels(this.cornerListPairs);
//             inputs.angleEncompassingTag = calculateAngleEncompassingTagHeight(inputs.targetHeight);
//             inputs.distanceToTagMeters = calculateDistanceToAprilTagInMetersUsingTrigMethod(inputs.angleEncompassingTag)
//                     * this.distanceScalarValue;
//         } else {
//             inputs.targetHeight = -1;
//             inputs.angleEncompassingTag = new Rotation2d();
//             inputs.distanceToTagMeters = -1;
//         }

//         if (inputs.hasTarget && inputs.distanceToTagMeters != -1 && inputs.tagId > 0 && inputs.tagId <= 22) {
//             inputs.robotPoseBasedOffDistanceCalcAndTagLocation = calculateRobotPose(inputs.tagId,
//                     inputs.distanceToTagMeters, inputs.horizontalAngleToTarget);
//         } else {
//             inputs.robotPoseBasedOffDistanceCalcAndTagLocation = null;
//         }

//     }

//     // TODO
//     @Override
//     public void setThrottleValue(int throttleValue) {
//         throttleSet.setNumber(throttleValue);
//     }

//     static double calculateTargetHeightInPixels(Collection<Pair<Double, Double>> sortedCorners) {
//         return sortedCorners.stream().mapToDouble(Pair::getSecond).max().orElse(-1)
//                 - sortedCorners.stream().mapToDouble(Pair::getSecond).min().orElse(-1);
//     }

//     // 排序Tag中的四個角 左上 右上 左下 右下

//     public static Collection<Pair<Double, Double>> sortCorners(Collection<Pair<Double, Double>> entries) {
//         ArrayList<Pair<Double, Double>> finalCornerList = new ArrayList<Pair<Double, Double>>();

//         for (int i = 0; i < 4; i++) {
//             finalCornerList.add(Pair.of(0.0, 0.0));
//         }

//         double CentroidX = entries.stream().mapToDouble(Pair::getFirst).average().orElse(0.0);
//         double CentroidY = entries.stream().mapToDouble(Pair::getFirst).average().orElse(0.0);

//         for (int i = 0; i < 4; i++) {
//             if (entries.stream().toList().get(i).getFirst() - CentroidX > 0) {
//                 if (entries.stream().toList().get(i).getSecond() - CentroidY > 0) {
//                     finalCornerList.set(1, entries.stream().toList().get(i));
//                 } else {
//                     finalCornerList.set(2, entries.stream().toList().get(i));
//                 }
//             } else if (entries.stream().toList().get(i).getFirst() - CentroidX < 0) {
//                 if (entries.stream().toList().get(i).getSecond() - CentroidY < 0) {
//                     finalCornerList.set(3, entries.stream().toList().get(i));
//                 } else {
//                     finalCornerList.set(0, entries.stream().toList().get(i));
//                 }
//             }
//         }

//         return finalCornerList;
//     }

//     // 將pixel高度轉換為視場夾角

//     static Rotation2d calculateAngleEncompassingTagHeight(double pixelHeight) {
//         return Rotation2d.fromDegrees(pixelHeight * VERTICAL_FIELD_OF_VIEW_IN_DEGREES / VERTICAL_RESOLUTION_IN_PIXELS);
//     }

//     // 計算距離

//     double calculateDistanceToAprilTagInMetersUsingTrigMethod(Rotation2d angle) {
//         var tanOfTheta1 = angle.getTan();
//         var sqrtTerm = Math.sqrt(Math.abs(Math.pow(TAG_HEIGHT, 2)
//                 - (4
//                         * tanOfTheta1
//                         * (offsetBetweenApriltagBottomAndCamera + TAG_HEIGHT)
//                         * tanOfTheta1
//                         * offsetBetweenApriltagBottomAndCamera)));
//         return Math.abs((TAG_HEIGHT + sqrtTerm) / (2 * tanOfTheta1));
//     }

//     // 利用 Odometry + Limelight + tagLocation 計算座標 + 延遲補償。尼瑪，我直接一個大問號
//     // TODO

//     Pose2d calculateRobotPose(final int tagId, double distanceToTagMeters, Rotation2d horizontalAngleToTarget) {
//         var tagPose = FieldConstants.getTagPose(tagId).toPose2d(); // Apriltag 座標

//         var robotRotation = RobotState.getInstance().getRobotPoseFromSwerveDriveOdometry().getRotation(); // Odometry Rotation TODO

//         var robotRotationWithLimelightCorrection = robotRotation.plus(limelightYaw); // 將Rotation加上Limelight角度

//         var scaledTx = Rotation2d.fromDegrees(
//                 -horizontalAngleToTarget.div(LIMELIGHT_SCALAR_FACTOR).getDegrees()); // WTF誤差補償 TODO

//         var cameraToRobotCenter = this.cameraToRobotCenter.rotateBy(
//                 RobotState.getInstance().getRobotPoseFromSwerveDriveOdometry().getRotation()); //將相機偏移轉換至機器中心

//         var angleToTag = scaledTx.plus(robotRotationWithLimelightCorrection);       //計算機器朝Tag角度

//         var translation = new Translation2d(distanceToTagMeters, angleToTag);      //場地座標 (Translation2d) 
//         var translatedPose = tagPose.getTranslation().minus(translation);          //場地座標 (Pose2d)
//                                                                                    //以上皆為相機位置

//         var fieldRelativeRobotTranslation = translatedPose.plus(cameraToRobotCenter);   //加上相機偏移

//         // 延遲補償 (尼瑪到底是三小)

//         var fieldRelativeChassisSpeeds = RobotState.getInstance().getRobotChassisSpeeds(); // 取得當前底盤速度，用於推測延遲後位置

//         var latencyCompensatedFieldRelativeTranslation = new Translation2d(
//                 fieldRelativeRobotTranslation.getX()
//                         + (fieldRelativeChassisSpeeds.vxMetersPerSecond * LIMELIGHT_LATENCY_CONSTANT_SECONDS), // 計算延遲後座標
//                 fieldRelativeRobotTranslation.getY()
//                         + (fieldRelativeChassisSpeeds.vyMetersPerSecond * LIMELIGHT_LATENCY_CONSTANT_SECONDS));

//         var latencyCompensatedPose = new Pose2d(latencyCompensatedFieldRelativeTranslation, robotRotation); // 延遲補償後pose
//         var uncompensatedPose = new Pose2d(fieldRelativeRobotTranslation, robotRotation);                   //原始pose



//         Logger.recordOutput(limelightName + "/latencyCompensatedPose", latencyCompensatedPose);
//         Logger.recordOutput(limelightName + "/uncompensatedPose", uncompensatedPose);

//         return latencyCompensatedPose;
//     }

//     @Override
//     public CameraConfiguration.Location getLimelightLocation() {
//         return limelightLocation;
//     }

//     @Override
//     public String getLimelightName() {
//         return limelightName;
//     }
// }
