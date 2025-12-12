package frc.robot.subsystems.NewVision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Vision.LimelightHelpers;
import frc.robot.subsystems.NewDrive.drive;
import frc.robot.subsystems.NewVision.VisionFuser.VisionConstants;

public class Vision extends SubsystemBase {
    
    private final drive drive;
    private final String leftName = "limelight-left";
    private final String rightName = "limelight-right";

    // 用來儲存看到的 Tag ID，供外部讀取 (Auto 或 Debug 用)
    private int leftTagId = -1;
    private int rightTagId = -1;

    public Vision(drive drive) {
        this.drive = drive;
    }

    @Override
    public void periodic() {
        // 每一週期處理兩顆鏡頭
        processLimelight(leftName);
        processLimelight(rightName);
        
        // Debug 顯示
        SmartDashboard.putNumber("Vision/LeftID", leftTagId);
        SmartDashboard.putNumber("Vision/RightID", rightTagId);
    }

    /** 核心邏輯：處理單顆 Limelight 的數據 */
    private void processLimelight(String llName) {
        // ---------------------------------------------------------
        // 1. 餵 Gyro 數據給 MegaTag 2 (MT2 核心)
        // ---------------------------------------------------------
        // 這裡假設 drive.getHeading() 回傳 Rotation2d
        // 這是讓單 Tag 變準的關鍵！
        LimelightHelpers.SetRobotOrientation(
            llName, 
            drive.getRotation2d().getDegrees(), 
            drive.getGyroYawRate(), 0, 0, 0, 0
        );

        // ---------------------------------------------------------
        // 2. 讀取 MT2 估算結果
        // ---------------------------------------------------------
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName);
        
        // 更新 Tag ID (取主目標)
        if (llName.equals(leftName)) {
            leftTagId = (int) LimelightHelpers.getFiducialID(leftName);
        } else {
            rightTagId = (int) LimelightHelpers.getFiducialID(rightName);
        }

        // ---------------------------------------------------------
        // 3. 過濾無效數據 (Filters)
        // ---------------------------------------------------------
        
        // 沒看到 Tag 則跳過
        if (mt2.tagCount == 0) return;

        // 機器人旋轉太快時 (大於 maxYawRate度/秒)，視覺會有殘影，不使用數據
        // (假設 drive.getGyroYawRate() 回傳 deg/s)
        if (Math.abs(drive.getGyroYawRate()) > VisionConstants.maxYawRate) return;

        // 檢查座標是否跑出場地外 (X: 0~16.54m, Y: 0~8.21m)
        if (mt2.pose.getX() < 0 || mt2.pose.getX() > 16.54 || 
            mt2.pose.getY() < 0 || mt2.pose.getY() > 8.21) return;

        // ---------------------------------------------------------
        // 4. 計算標準差 (Trust Level)
        // ---------------------------------------------------------
        double xyStds;
        double degStds;
        double avgDist = mt2.avgTagDist;

        if (mt2.tagCount >= 2) {
            // 多 Tag：非常信任
            xyStds = 0.5; 
            degStds = 6.0; 
        } else {
            // 單 Tag：信任度隨距離遞減 (距離越遠，標準差越大)
            // 這裡使用距離的平方來快速降低遠距離的權重
            xyStds = 1.0 * (avgDist * avgDist); 
            degStds = 999.0; // 單 Tag 完全不信任 MT2 算出的角度，只用它的 X/Y
        }

        // ---------------------------------------------------------
        // 5. 送入 Drive Subsystem
        // ---------------------------------------------------------
        // 這裡需要你的 Drive 支援接收標準差 (Vector<N3>)
        drive.addVisionMeasurement(
            mt2.pose,             // 視覺算出的 Pose2d
            mt2.timestampSeconds, // 這是正確的拍攝時間 (Latency Compensated)
            VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds))
        );

        double[] pose = { 
            mt2.pose.getX(),
            mt2.pose.getY(),
            mt2.pose.getRotation().getRadians()
        };

        SmartDashboard.putNumberArray("LLPose", pose);
    }

    // 提供給外部使用的 Getter
    public int getLeftTagId() { return leftTagId; }
    public int getRightTagId() { return rightTagId; }
}