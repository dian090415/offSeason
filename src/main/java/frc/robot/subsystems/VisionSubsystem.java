package frc.robot.subsystems;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drivetrain.SwerveSubsystem;
import lombok.val;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera LeftOV;
    private final PhotonCamera RightOV;
    private final AprilTagFieldLayout fieldLayout;

    private double x, y, z, yaw;

    private final PIDController xPID = new PIDController(1.0, 0, 0);
    private final PIDController yPID = new PIDController(1.0, 0, 0);
    private final PIDController yawPID = new PIDController(1.0, 0, 0);

    // 相機在機器人上的安裝位置（以公尺為單位）
    private final Transform3d robotToRightOVCam = new Transform3d(
            new Translation3d(0.20979456, -0.13607890, 0.15952705), // X, Y, Z
            new Rotation3d(0.0, 0.0, Math.toRadians(30)) // Pitch, Roll, Yaw
    );
    private final Transform3d robotToLeftOVCam = new Transform3d(
            new Translation3d(0.20979456, 0.13607890, 0.15952705), // X, Y, Z
            new Rotation3d(0.0, 0.0, Math.toRadians(-30)) // Pitch, Roll, Yaw
    );

    private final SwerveSubsystem swerve; // Swerve 子系統實體引用

    // ✅ Pose3d 發佈器（NetworkTables）
    private final StructPublisher<Pose3d> posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Vision/RobotPose", Pose3d.struct)
            .publish();

    public VisionSubsystem(SwerveSubsystem swerve) {
        this.swerve = swerve;
        this.RightOV = new PhotonCamera("RightOV");
        this.LeftOV = new PhotonCamera("LeftOV");
        this.fieldLayout = AprilTagFields.k2025ReefscapeWelded.loadAprilTagLayoutField();
    }

    @Override
    public void periodic() {
        this.TagToRobot();
        if(goodLeftOV() == null){
            return;
        }
        // swerve.resetOdometry(goodOV());
    }

    public Pose2d goodLeftOV() {
        PhotonPipelineResult Leftresult = LeftOV.getLatestResult();

        if (!Leftresult.hasTargets())
            return null;

        PhotonTrackedTarget target = Leftresult.getBestTarget();
        int tagID = target.getFiducialId();

        Optional<Pose3d> tagPoseOpt = fieldLayout.getTagPose(tagID);
        if (tagPoseOpt.isEmpty())
            return null;

        Pose3d tagPose = tagPoseOpt.get(); // Tag 在場地的 Pose
        Transform3d cameraToTarget = target.getBestCameraToTarget(); // Tag → Camera

        Pose3d LeftOVfieldToRobot = tagPose
                .transformBy(cameraToTarget.inverse())
                .transformBy(robotToLeftOVCam.inverse());

        return new Pose2d(
                LeftOVfieldToRobot.getX(),
                LeftOVfieldToRobot.getY(),
                LeftOVfieldToRobot.getRotation().toRotation2d() // 只取 Z 軸旋轉 (Yaw)
        );
    }

    public Pose2d goodRightOV() {
        PhotonPipelineResult RightOVresult = RightOV.getLatestResult();

        if (!RightOVresult.hasTargets())
            return null;

        PhotonTrackedTarget target = RightOVresult.getBestTarget();
        int tagID = target.getFiducialId();

        Optional<Pose3d> tagPoseOpt = fieldLayout.getTagPose(tagID);
        if (tagPoseOpt.isEmpty())
            return null;

        Pose3d tagPose = tagPoseOpt.get(); // Tag 在場地的 Pose
        Transform3d cameraToTarget = target.getBestCameraToTarget(); // Tag → Camera

        Pose3d RIghtOVfieldToRobot = tagPose
                .transformBy(cameraToTarget.inverse())
                .transformBy(robotToRightOVCam.inverse());
        return new Pose2d(
            RIghtOVfieldToRobot.getX(),
            RIghtOVfieldToRobot.getY(),
            RIghtOVfieldToRobot.getRotation().toRotation2d() // 只取 Z 軸旋轉 (Yaw)
        );
    }
    public Pose2d goodOV(){
        if (goodLeftOV() != null && goodRightOV() != null) {
            return new Pose2d(
                (this.goodLeftOV().getX()+this.goodRightOV().getX())/2,
                (this.goodLeftOV().getY()+this.goodRightOV().getY())/2,
                this.swerve.getRotation2d()
            );
        } else if (goodLeftOV() != null) {
            return new Pose2d(
                this.goodLeftOV().getX(),
                this.goodLeftOV().getY(),
                this.swerve.getRotation2d()
            );
        } else if (goodRightOV() != null) {
            return new Pose2d(
                this.goodRightOV().getX(),
                this.goodRightOV().getY(),
                this.swerve.getRotation2d()
            );
        }else{
            return null;
        }
    }

    // /** Robot 相對於 Tag 的位置（Tag 為原點） */
    public void TagToRobot() {
        var result = RightOV.getLatestResult();
        // if (!result.hasTargets()) 
        // return;
        if (result.hasTargets()) {
            SmartDashboard.putBoolean("camera?", result.hasTargets());

            PhotonTrackedTarget target = result.getBestTarget();
            Transform3d cameraToTarget = target.getBestCameraToTarget(); // Tag → Camera

             // Tag → Robot
            Transform3d tagToRobot = cameraToTarget.inverse().plus(robotToRightOVCam.inverse());
        
            x = tagToRobot.getX(); // 機器人相對於 Tag 的 X（前後距離）
            y = tagToRobot.getY(); // 左右
            z = tagToRobot.getZ(); // 高度
            yaw = tagToRobot.getRotation().getZ(); // 旋轉角
        }else {
            x = 0;
            y = 0;
            z = 0;
            yaw = 0;
        }
    
    // SmartDashboard.putBoolean("camera?", result.hasTargets());

    // PhotonTrackedTarget target = result.getBestTarget();
    // Transform3d cameraToTarget = target.getBestCameraToTarget(); // Tag → Camera

    // // Tag → Robot
    // Transform3d tagToRobot = cameraToTarget.inverse().plus(robotToRightOVCam.inverse());

    // double x = tagToRobot.getX(); // 機器人相對於 Tag 的 X（前後距離）
    // double y = tagToRobot.getY(); // 左右
    // double z = tagToRobot.getZ(); // 高度
    // double yaw = tagToRobot.getRotation().getZ(); // 旋轉角

    SmartDashboard.putNumber("RobotFromTag_X", x);
    SmartDashboard.putNumber("RobotFromTag_Y", y);
    SmartDashboard.putNumber("RobotFromTag_YawDeg", Math.toDegrees(yaw));
    // SmartDashboard.putNumber("cameraToTarget_X", cameraToTarget.getX());
    // SmartDashboard.putNumber("cameraToTarget_Y", cameraToTarget.getY());

    }



    // /** 額外提供 2D Pose（若需要） */
    // public Pose2d getRobotPose2d() {
    // PhotonPipelineResult result = LeftOV.getLatestResult();

    // if (!result.hasTargets())
    // return null;

    // PhotonTrackedTarget target = result.getBestTarget();
    // int tagID = target.getFiducialId();
    // Optional<Pose3d> tagPoseOpt = fieldLayout.getTagPose(tagID);
    // if (tagPoseOpt.isEmpty())
    // return null;

    // Pose3d tagPose = tagPoseOpt.get();
    // Transform3d cameraToTarget = target.getBestCameraToTarget();

    // Pose3d fieldToRobot = tagPose
    // .transformBy(cameraToTarget.inverse())
    // .transformBy(robotToCam.inverse());

    // return new Pose2d(
    // fieldToRobot.getX(),
    // fieldToRobot.getY(),
    // fieldToRobot.getRotation().toRotation2d());
    // }
}
