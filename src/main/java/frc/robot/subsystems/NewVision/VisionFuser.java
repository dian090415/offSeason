package frc.robot.subsystems.NewVision;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.NewDrive.drive;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.DriverStation;
import com.ctre.phoenix6.Utils;

/**
 * VisionFuser
 * - 多相機 AprilTag Pose 估計 -> 權重融合 -> 送入
 * SwerveDrivePoseEstimator.addVisionMeasurement
 *
 * 設定/使用：
 * - 構造時傳入：cameraTransforms (cameraName -> Transform3d), poseEstimator 實例
 * - 週期呼叫 update(); 會讀取每台 PhotonCamera 的 unread results, 使用
 * estimator.update(result)
 * - 合併後（若有至少一個可信觀測）呼叫 poseEstimator.addVisionMeasurement(fusedPose,
 * visionTimestamp)
 */
public class VisionFuser extends SubsystemBase {
  private int tagId = -1;

  /** 輔助：示範 VisionConstants.SIM_CAMERA_PROPERTIES 的最小 stub（你可以在別處定義） */
  public class VisionConstants {
    public static final Map<String, Transform3d> cameraTransforms = Map.of(
        "RightOV", new Transform3d(
            // 位置不變 (車尾右側)
            new Translation3d(-0.20979456, -0.13607890, 0.15952705),
            // 🛠️ 修改這裡：原本是 180-30，改成 180+30 (即 -150度)
            new Rotation3d(0.0, 0.0, Math.toRadians(180 + 30))),
        "LeftOV", new Transform3d(
            // 位置不變 (車尾左側)
            new Translation3d(-0.20979456, 0.13607890, 0.15952705),
            // 🛠️ 修改這裡：原本是 -180+30，改成 -180-30 (即 150度)
            new Rotation3d(0.0, 0.0, Math.toRadians(-180 - 30))));
  }

  private static class CamWrapper {
    final String name;
    final PhotonCamera cam;
    final PhotonPoseEstimator estimator;

    CamWrapper(String name, PhotonCamera cam, PhotonPoseEstimator estimator) {
      this.name = name;
      this.cam = cam;
      this.estimator = estimator;
    }
  }

  private final List<CamWrapper> cams = new ArrayList<>();
  private drive drive;

  // thresholds & tuning:
  private final double borderPixels = 15.0; // 拒絕貼邊緣的角點（避免畸變/遮擋）
  private final double minDistanceSingleTagMeters = Units.feetToMeters(4.0); // 單tag可信最小距離
  private final double maxSingleTagDistanceMeters = Units.feetToMeters(6.0); // 單tag最遠可接受距離
  private final double multiTagTrustedDistance = Units.feetToMeters(10.0); // 多tag可信距離
  private final double minWeight = 1e-6; // 避免除以 0

  /**
   * @param cameraTransforms map: cameraName -> Transform3d (camera-to-robot
   *                         transform)
   * @param poseEstimator    SwerveDrivePoseEstimator 實例（你用來融合 odometry + vision）
   */
  public VisionFuser(Map<String, Transform3d> cameraTransforms, drive drive) {
    this.drive = drive;

    // 載入官方場地 tag 資訊 (photon pose estimator 需要 field layout)
    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // 為每顆 camera 建立 PhotonCamera + PhotonPoseEstimator
    VisionConstants.cameraTransforms.forEach((name, transform) -> {
      PhotonCamera cam = new PhotonCamera(name);
      PhotonPoseEstimator estimator = new PhotonPoseEstimator(
          fieldLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          transform);
      // 多 tag fallback: 遇不到 multi-tag 結果時使用最低 ambiguous 方案
      estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      cams.add(new CamWrapper(name, cam, estimator));
    });
  }

  /**
   * - 讀 unread results (每個 camera)
   * - 用 PhotonPoseEstimator.update(result) 產生 Pose3d（camera frame -> robot frame）
   * - 根據距離、被使用 tag 數、邊界檢查等決定是否接受此觀測，以及計算權重
   * - 權重平均合併 (x, y, theta)
   * - 轉成 Pose2d 與 timestamp，丟給 poseEstimator.addVisionMeasurement()
   */
  @Override
  public void periodic() {
    this.vision();
  }

  public void vision() {
    // 遍歷每一台相機 (不再需要收集 List 做平均，直接處理直接送)
    for (CamWrapper cw : cams) {
      // 讀取這台相機的所有未讀結果
      for (PhotonPipelineResult result : cw.cam.getAllUnreadResults()) {

        // 1. 基礎檢查與更新
        var poseOpt = cw.estimator.update(result);

        if (result.hasTargets()) {
          tagId = result.getBestTarget().getFiducialId();
        }

        if (poseOpt.isEmpty())
          continue;

        // 機器人旋轉太快時 (大於 720度/秒)，視覺會有殘影，不使用數據
        if (Math.abs(drive.getGyroYawRate()) > 720)
          continue;

        var est = poseOpt.get();
        Pose3d cameraRobotPose3d = est.estimatedPose;
        double resultTimeSec = est.timestampSeconds;

        // 2. 過濾邏輯 (Filter)

        // Z 軸高度檢查
        if (!filterByZ(cameraRobotPose3d))
          continue;

        // 邊緣檢查 (Corner Edge Check)
        boolean cornerNearEdge = false;
        var targets = result.getTargets();
        for (var tgt : targets) {
          var corners = tgt.detectedCorners;
          if (corners != null) {
            for (var corner : corners) {
              if (corner == null)
                continue;
              if (Math.abs(corner.x - 0.0) < borderPixels || Math.abs(corner.y - 0.0) < borderPixels ||
                  Math.abs(corner.x - cw.cam.getCameraMatrix().get().getNumCols()) < borderPixels || // 簡化寫法，或維持原樣
                  Math.abs(corner.y - cw.cam.getCameraMatrix().get().getNumRows()) < borderPixels) { // 這裡假設你有拿到解析度，若無維持原判斷即可
                // 註：若不想改原本的寬高判斷，維持原本寫法即可，這邊示意
                if (Math.abs(corner.x - 0.0) < borderPixels || Math.abs(corner.y - 0.0) < borderPixels) {
                  cornerNearEdge = true;
                  break;
                }
                // 注意：上面這幾行如果你原本的寫法有 width/height 變數，請繼續使用你原本的寫法
                // 為了保持你的註解與邏輯，我還原你原本的邊緣檢查邏輯如下：
                // (假設 camera resolution 寫死或已知，這邊簡化為不檢查右下邊界以免報錯，或者你保留原本程式碼)
              }
            }
          }
          if (cornerNearEdge)
            break;
        }
        if (cornerNearEdge)
          continue;

        // 計算距離與 Tag 數量
        double avgDist = 0.0;
        int usedTags = 0;
        for (var tgt : poseOpt.get().targetsUsed) {
          double d = tgt.getBestCameraToTarget().getTranslation().getNorm();
          avgDist += d;
          usedTags++;
        }

        // Ambiguity 檢查 (重要！)
        var bestTarget = result.getBestTarget();
        if (usedTags == 1 && bestTarget != null && bestTarget.getPoseAmbiguity() > 0.2) {
          continue; // 單 Tag 太模糊，丟棄
        }

        if (usedTags == 0)
          continue;
        avgDist /= usedTags;

        // 距離過濾
        if (usedTags < 2 && avgDist > maxSingleTagDistanceMeters)
          continue;

        // 3. 計算標準差 (Trust) - 這取代了原本的 Weight 計算
        Vector<N3> stdDevs;
        if (usedTags >= 2) {
          // 【多 Tag】非常信任：X/Y 10cm, 角度 5度
          stdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));
        } else {
          // 【單 Tag】不信任：誤差隨距離平方增長
          double distError = 0.5 * avgDist * avgDist;
          // 角度給予無限大 (999999)，代表「完全不相信單 Tag 的角度」，只相信 Gyro
          stdDevs = VecBuilder.fill(distError, distError, 999999);
        }

        // 4. 直接送出數據 (Send to Pose Estimator)
        // 不需要再存 list 做平均了，直接餵給 Estimator
        Pose2d robotPose2d = cameraRobotPose3d.toPose2d();
        double fpgatime = Utils.fpgaToCurrentTime(resultTimeSec);

        // 呼叫 drive 的方法 (請確認 NewDrive 有支援接收 stdDevs)
        drive.addVisionMeasurement(robotPose2d, fpgatime, stdDevs);
      }
    }
    // 迴圈結束，工作完成。PoseEstimator 會自動處理融合。
  }

  // public void vision() {
  // List<Pose2d> poses = new ArrayList<>();
  // List<Double> weights = new ArrayList<>();// 存放對應的權重（用來融合時加權平均）
  // List<Double> timestamps = new ArrayList<>();// 存放對應的時間戳（確保融合時能對齊時間）
  // List<Vector<N3>> stdDevs = new ArrayList<>();// 新增這一行：用來存每一台相機算出來的標準差

  // for (CamWrapper cw : cams) {
  // for (PhotonPipelineResult result : cw.cam.getAllUnreadResults()) {// -
  // 相機清單，每個 CamWrapper 包含一顆相機與它的 Pose Estimator
  // // 先讓 estimator 以目前已知 odometry 做 reference pose（photon 建議）
  // // estimator.update(result) 會回傳 Optional<EstimatedRobotPose>
  // var poseOpt = cw.estimator.update(result);// 取出這顆相機所有「未處理過」的結果（PhotonVision
  // 會累積結果，這裡逐一處理）
  // // 讀取apriltagid
  // if (result.hasTargets()) {// 檢查這一幀有沒有偵測到任何 AprilTag。
  // tagId = result.getBestTarget().getFiducialId();// PhotonVision
  // 會挑一個「最佳目標」（通常是最近、最清晰的）取得這個目標的 AprilTag ID（整數
  // }
  // if (poseOpt.isEmpty()) {
  // continue;
  // }
  // if (Math.abs(drive.getGyroYawRate()) > 720)// 機器人旋轉太快時 (大於
  // 720度/秒)，視覺會有殘影，不使用數據
  // continue;

  // var est = poseOpt.get(); // 具有 .estimatedPose (Pose3d) 與 .timestampSeconds
  // Pose3d cameraRobotPose3d = est.estimatedPose; //
  // est.estimatedPose：估測到的機器人姿態（3D，包含 X/Y/Z 與旋轉）
  // double resultTimeSec = est.timestampSeconds; // Photon 的 timestamp
  // 秒數（coprocessor 時間基底）

  // // basic checks --------------------------------------------------
  // if (!filterByZ(cameraRobotPose3d)) {
  // // 高度超過可接受範圍（可能是誤檢）
  // continue;
  // }

  // // 檢查檢測到的每個 tag 的角點是否貼邊（若有貼邊則跳過該 frame）
  // boolean cornerNearEdge = false;// 建立一個旗標（flag），表示「這個 frame（影像）是否有任何被檢測到的 tag
  // 的角點靠近影像邊緣」。預設為 false。
  // var targets = result.getTargets();// 回傳檢測到的所有 target（每個 target 代表一個 AprilTag
  // 或偵測到的標記物）。我們要逐個檢查每個 target 的角點位置。
  // for (var tgt : targets) {// 開始迭代每一個被偵測到的 target
  // var corners = tgt.detectedCorners;// detectedCorners 通常是 target
  // 在影像中的四個角點（通常為像素座標），可能是 List<Point>
  // // 或類似結構。注意它可能為 null（取決於 Photon 的版本或資料是否完整），
  // // 所以要做 null 檢查
  // if (corners != null) {// 只有當角點資料存在時才進入下一層迭代
  // for (var corner : corners) {// 迭代該 target 的每一個角點（通常是四個）
  // if (corner == null)
  // continue;// 防守式程式（defensive）檢查：如果某個角點為 null，跳過它（避免 NPE）
  // // Photon's corner.x/y 是 pixel（0..width/height）
  // // 提醒你 corner 的座標是以像素為單位（畫面左上為 0,0，右下大約為 width, height）。不同相機解析度或模擬參數，這些值會不同。
  // /*
  // * 這四個子條件分別檢查角點是否「接近影像左邊/右邊/上邊/下邊」：
  // *
  // * Math.abs(corner.x - 0.0) < borderPixels → x 靠近左邊（x ≈ 0）
  // *
  // * Math.abs(corner.x - width) < borderPixels → x 靠近右邊（x ≈ width）
  // *
  // * Math.abs(corner.y - 0.0) < borderPixels → y 靠近上邊
  // *
  // * Math.abs(corner.y - height) < borderPixels → y 靠近下邊
  // */
  // // borderPixels 是允許的邊界厚度（像素），如果角點在這個厚度以內就判為「靠邊」。一旦發現靠邊，將 cornerNearEdge =
  // true 並
  // // break（跳出角點迴圈）。
  // if (Math.abs(corner.x - 0.0) < borderPixels || Math.abs(corner.y - 0.0) <
  // borderPixels) {
  // cornerNearEdge = true;
  // break;
  // }
  // }
  // }
  // if (cornerNearEdge)
  // break;// 跳出最外層的 targets 迴圈（已經知道這frame有靠邊角點，沒必要再檢查其他 target）
  // }
  // if (cornerNearEdge) {
  // continue;// 如果這一幀影像有任何角點靠邊 → 丟棄這一幀，不使用它的姿態估測結果
  // }

  // // 計算距離與可信度條件 --------------------------------------------
  // double avgDist = 0.0;// 用來累積並計算「平均距離」
  // int usedTags = 0;// 統計這一幀影像中實際被用來解算姿態的標籤數量
  // double closest = Double.POSITIVE_INFINITY;// 記錄「最近的標籤距離」，初始設為無限大
  // for (var tgt : poseOpt.get().targetsUsed) {// poseOpt.get().targetsUsed：這是
  // PhotonPoseEstimator
  // // 在這一幀中實際用來計算姿態的標籤集合。
  // double d = tgt.getBestCameraToTarget().getTranslation().getNorm();//
  // 計算相機到該標籤的距離（單位：公尺）
  // avgDist += d;// - 把距離加總到 avgDist
  // usedTags++;// 並增加 usedTags 計數
  // if (d < closest)
  // closest = d;// 如果這個距離比目前記錄的 closest 還小，就更新最近距離
  // }
  // // 在讀取 poseOpt 之後加入：
  // var bestTarget = result.getBestTarget();
  // if (usedTags == 1 && bestTarget.getPoseAmbiguity() > 0.2) {
  // // 單 Tag 且模糊度太高 -> 丟棄
  // continue;
  // }
  // if (usedTags == 0)
  // continue;// 如果沒有任何標籤被用來解算，這一幀就直接跳過
  // avgDist /= usedTags;// 否則，計算平均距離 avgDist

  // // 單 tag 時要求距離不能太遠（因為單標籤會有姿態不確定性）
  // if (usedTags < 2) {
  // if (avgDist > maxSingleTagDistanceMeters) {// 如果總距離大於最大距離限制 -> 捨棄這一幀
  // continue;
  // }
  // } else {
  // // 若多 tag，但距離全部都很遠，有可能不穩 → 可以拒絕或降低權重（這邊只降低權重）
  // }

  // double p = 1.0;/*- p = 1.0 → 權重與距離成反比。
  // - 標籤數越多 → 權重越大。
  // - 距離越近 → 權重越大。 */

  // double weight = usedTags * (1.0 / Math.max(avgDist, 0.5));
  // // 權重計算：可以根據距離、被用到的 tag 數、corner 數、標籤 id 是否在 reef 區域等決定
  // // 簡單策略： weight = (nTags) * (1 / distance^p), p=1..2
  // /*- 權重的基本公式：
  // \text{weight} = \text{標籤數量} \times \frac{1}{\text{平均距離}^p}- p = 1.0 →
  // 權重與距離成反比。
  // 標籤數越多 → 權重越大。
  // 距離越近 → 權重越大。
  // Math.max(avgDist, 0.5) → 避免除以零或距離過小造成權重爆炸 */

  // // 若最接近的 tag 非常近，給額外加權
  // if (closest < Units.metersToInches(2.0) / 39.37) {// 如果最近的標籤距離小於 2 英尺（約 0.6
  // 公尺），代表相機有看到非常近的標籤。
  // weight *= 1.5;// 近距離標籤通常更可靠，因此額外乘上 1.5 倍權重。
  // }

  // // 若 detector 回傳置信度（Photon 的 target.confidence），可乘上去（此處用 best target
  // confidence）
  // // PhotonVision 的 target 可能會有「置信度 (confidence)」欄位（不同版本支援不同）
  // if (result.getTargets().size() > 0 /* 這一幀有偵測到至少一個 AprilTag */ &&
  // result.getBestTarget() != null/*
  // * PhotonVision
  // * 幫你挑了一個「最佳目標」
  // */) {
  // // 貌似調權重使用的
  // weight *= Math.max(0.1, result.getBestTarget().getFiducialId() >= 0 ? 1.0 :
  // 1.0); // placeholder: 可用
  // // target.confidence//max 取
  // // X 和 0.1 之間的最大值
  // }
  // // --- 新增邏輯開始：計算動態標準差 ---
  // Vector<N3> currentStdDev;

  // if (usedTags >= 2) {
  // // 【情況 1：多 Tag】非常信任
  // // X, Y 給予 0.1 (公尺) 的誤差容許度
  // // 角度給予 5 度 的誤差容許度 (多 Tag 算角度很準)
  // currentStdDev = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));
  // } else {
  // // 【情況 2：單 Tag】不信任，且距離越遠越不準
  // // 誤差隨距離平方增長
  // double distError = 0.5 * avgDist * avgDist; // 係數 0.5 可自行調整

  // // 角度給予無限大 (99999)，代表「完全不相信單 Tag 的角度」，只相信 Gyro
  // currentStdDev = VecBuilder.fill(distError, distError, 999999);
  // }

  // // 把 3D pose 轉成 2D pose（忽略 Z）
  // Pose2d robotPose2d = cameraRobotPose3d.toPose2d();

  // // 收集
  // // 把這一幀的姿態、權重、時間戳存進清單
  // poses.add(robotPose2d);// 姿態
  // weights.add(weight);// 權重
  // stdDevs.add(currentStdDev);// 信任值
  // // Utils.fpgaToCurrentTime(resultTimeSec)：把 PhotonVision 的時間戳（協處理器時間）轉換成 FPGA
  // // 時間，確保與 WPILib 的 PoseEstimator 使用同一個時間基準
  // // convert camera / photon timestamp -> FPGA time so poseEstimator uses same
  // // clock
  // double fpgatime = Utils.fpgaToCurrentTime(resultTimeSec); // 如有問題，可改成
  // Timer.getFPGATimestamp() - latency
  // timestamps.add(fpgatime);// 時間戳存
  // }
  // }

  // // 若沒拿到任何可信觀測，直接 return
  // if (poses.isEmpty()) {
  // return;// - 如果沒有任何可信的觀測結果（poses 為空），就直接結束，不做融合
  // }

  // // 權重平均（x, y, angle）——角度以向量平均處理（避免 2π 跳變）
  // double sumW = 0.0;// 權重總和
  // double sumX = 0.0;// 加權後的 X 座標總和
  // double sumY = 0.0;//// 加權後的 Y 座標總和
  // double sumCosA = 0.0;// 角度的 cos 與 sin 加權總和（用來做向量平均)
  // double sumSinA = 0.0;//// 角度的 cos 與 sin 加權總和（用來做向量平均)
  // double latestTimestamp = 0.0;// 記錄最新（最大的）時間戳

  // for (int i = 0; i < poses.size(); i++) {
  // double w = Math.max(weights.get(i), minWeight);// w = 這個觀測的權重，至少要大於
  // minWeight（避免權重為 0）
  // Pose2d p = poses.get(i);// p = 這個觀測的姿態（Pose2d）
  // sumW += w;// 同時累積權重總和
  // sumX += p.getX() * w;// 把每個觀測的 X、Y 乘上權重後加總
  // sumY += p.getY() * w;// 把每個觀測的 X、Y 乘上權重後加總
  // /*- 角度不能直接數值平均，因為會有「2π 跳變」問題。
  // 例如：179° 和 -179° 的平均應該是 180°（或 -180°），但直接平均會得到 0°，完全錯誤。
  // - 解法：把角度轉成單位向量 (cosθ, sinθ)，再做加權平均。
  // - 最後再用 atan2(sumSinA, sumCosA) 算回平均角度。*/
  // sumCosA += Math.cos(p.getRotation().getRadians()) * w;
  // sumSinA += Math.sin(p.getRotation().getRadians()) * w;
  // // 使用最晚的 timestamp（或也可以做加權時間）
  // latestTimestamp = Math.max(latestTimestamp, timestamps.get(i));//
  // 這樣融合後的姿態會對應到最新的觀測時間
  // }

  // double fusedX = sumX / sumW;// - sumX、sumY 是前面迴圈累積的「加權後座標總和」。
  // double fusedY = sumY / sumW;// - sumW 是權重總和
  // double fusedTheta = Math.atan2(sumSinA / sumW, sumCosA / sumW);// - 這裡用
  // atan2(sumSinA / sumW, sumCosA / sumW)
  // // 算回平均角度。

  // Pose2d fused = new Pose2d(fusedX, fusedY, new Rotation2d(fusedTheta));// -
  // 把加權平均後的 X、Y、角度組合成一個新的 Pose2d

  // // --- 新增邏輯：挑選最佳標準差 ---
  // // 預設一個很大的值
  // double bestXStd = 999.0;
  // double bestYStd = 999.0;
  // double bestThetaStd = 999.0;

  // // 遍歷所有相機的標準差，找出最小值（最可信的）
  // for (Vector<N3> s : stdDevs) {
  // if (s.get(0, 0) < bestXStd)
  // bestXStd = s.get(0, 0);
  // if (s.get(1, 0) < bestYStd)
  // bestYStd = s.get(1, 0);
  // if (s.get(2, 0) < bestThetaStd)
  // bestThetaStd = s.get(2, 0);
  // }

  // // 建立最終送出的標準差向量
  // Vector<N3> finalStdDevs = VecBuilder.fill(bestXStd, bestYStd, bestThetaStd);

  // // debug publish
  // SmartDashboard.putNumber("VisionFuser/numObservations", poses.size());
  // SmartDashboard.putNumber("VisionFuser/fusedX", fusedX);
  // SmartDashboard.putNumber("VisionFuser/fusedY", fusedY);
  // SmartDashboard.putNumber("VisionFuser/fusedThetaDeg", fusedTheta * 180.0 /
  // Math.PI);

  // // 最後把 fused pose 與對應時間戳回饋給 pose estimator
  // // SwerveDrivePoseEstimator.addVisionMeasurement(Pose2d visionPose, double
  // // timestamp)
  // /*
  // * poseEstimator.addVisionMeasurement(fused, latestTimestamp);- 把融合後的姿態 fused
  // * 和對應的時間戳 latestTimestamp 傳給 SwerveDrivePoseEstimator。
  // * - SwerveDrivePoseEstimator 會把這個「視覺觀測」和「里程計 (odometry)」融合，得到更準確的機器人位置
  // */
  // // optional: 若你不使用 SwerveDrivePoseEstimator，可以改成呼叫你的
  // // drive.addVisionMeasurement(fused, latestTimestamp)

  // drive.addVisionMeasurement(fused, latestTimestamp, finalStdDevs);
  // }

  /** 濾掉明顯錯誤的高度（例如相機誤估到天花板） */
  private boolean filterByZ(Pose3d pose3d) {
    double z = pose3d.getZ();
    // 若相機報出的機器人 z > 0.6m 代表不合理（你的場地、相機角度會影響門檻）
    return Math.abs(z) < 0.5;
  }

  public int apriltagId() {
    return tagId;
  }

  /*---------------------------------auto第一次刷新位置使用------------------------------------------------------- */
  /**
   * 用於 Auto 初始化：強制將機器人座標「重置」為視覺看到的最準確位置。
   * 建議在 DisabledPeriodic 或 AutonomousInit 且機器人靜止時呼叫。
   * 
   * @return 是否成功重置 (有看到 Tag 才會回傳 true)
   */
  public boolean resetPoseToVision() {
    Pose2d bestPose = null;
    double minStdDev = 999.0; // 用來比較誰比較準，數值越小越準
    int bestTagCount = 0;

    for (CamWrapper cw : cams) {
      // 讀取最新結果
      var result = cw.cam.getLatestResult();
      if (!result.hasTargets())
        continue;

      var poseOpt = cw.estimator.update(result);
      if (poseOpt.isEmpty())
        continue;

      var est = poseOpt.get();
      Pose3d pose3d = est.estimatedPose;

      // 1. 基本過濾：高度是否合理
      if (Math.abs(pose3d.getZ()) > 0.5)
        continue;

      // 2. 計算 Tag 數量與平均距離
      int usedTags = 0;
      double avgDist = 0.0;
      for (var tgt : est.targetsUsed) {
        avgDist += tgt.getBestCameraToTarget().getTranslation().getNorm();
        usedTags++;
      }
      if (usedTags == 0)
        continue;
      avgDist /= usedTags;

      // 3. 過濾模糊的單 Tag
      // 如果只有 1 個 Tag，且模糊度太高 (>0.2)，這個數據不安全，不要用來重置
      if (usedTags == 1) {
        var bestTarget = result.getBestTarget();
        if (bestTarget != null && bestTarget.getPoseAmbiguity() > 0.2)
          continue;
      }

      // 4. 評分機制：找出「最可信」的 Pose
      // 邏輯：多 Tag 優先於單 Tag。同 Tag 數時，距離近者優先。
      // 這裡我們簡單算出一個「信任分數 (stdDev)」，越小越好
      double currentScore;
      if (usedTags >= 2) {
        currentScore = 0.1 + avgDist * 0.1; // 多 Tag 分數很低 (很好)
      } else {
        currentScore = 10.0 + avgDist * 2.0; // 單 Tag 分數較高 (較差)
      }

      // 如果這台相機比目前的最佳結果還準，就更新
      if (usedTags > bestTagCount || (usedTags == bestTagCount && currentScore < minStdDev)) {
        minStdDev = currentScore;
        bestTagCount = usedTags;
        bestPose = pose3d.toPose2d();
      }
    }

    // 5. 如果有找到任何可信的 Pose，就重置 Drive 的里程計
    if (bestPose != null) {
      // ⚠️ 呼叫 Drive 的 resetOdometry (硬重置)
      // 注意：這會把機器人的座標直接改掉，請確保機器人是靜止的
      drive.resetOdometry(bestPose);
      return true;
    }

    return false; // 沒看到任何 Tag，重置失敗
  }
  // public void forcevision() {
  // List<Pose2d> poses = new ArrayList<>();
  // List<Double> weights = new ArrayList<>();// 存放對應的權重（用來融合時加權平均）
  // List<Double> timestamps = new ArrayList<>();// 存放對應的時間戳（確保融合時能對齊時間）

  // for (CamWrapper cw : cams) {
  // for (PhotonPipelineResult result : cw.cam.getAllUnreadResults()) {// -
  // 相機清單，每個 CamWrapper 包含一顆相機與它的 Pose Estimator
  // // 先讓 estimator 以目前已知 odometry 做 reference pose（photon 建議）
  // // estimator.update(result) 會回傳 Optional<EstimatedRobotPose>
  // var poseOpt = cw.estimator.update(result);// 取出這顆相機所有「未處理過」的結果（PhotonVision
  // 會累積結果，這裡逐一處理）
  // // 讀取apriltagid
  // if (result.hasTargets()) {// 檢查這一幀有沒有偵測到任何 AprilTag。
  // tagId = result.getBestTarget().getFiducialId();// PhotonVision
  // 會挑一個「最佳目標」（通常是最近、最清晰的）取得這個目標的 AprilTag ID（整數
  // }
  // if (poseOpt.isEmpty()) {
  // continue;
  // }

  // var est = poseOpt.get(); // 具有 .estimatedPose (Pose3d) 與 .timestampSeconds
  // Pose3d cameraRobotPose3d = est.estimatedPose; //
  // est.estimatedPose：估測到的機器人姿態（3D，包含 X/Y/Z 與旋轉）
  // double resultTimeSec = est.timestampSeconds; // Photon 的 timestamp
  // 秒數（coprocessor 時間基底）

  // // basic checks --------------------------------------------------
  // if (!filterByZ(cameraRobotPose3d)) {
  // // 高度超過可接受範圍（可能是誤檢）
  // continue;
  // }

  // // 檢查檢測到的每個 tag 的角點是否貼邊（若有貼邊則跳過該 frame）
  // boolean cornerNearEdge = false;// 建立一個旗標（flag），表示「這個 frame（影像）是否有任何被檢測到的 tag
  // 的角點靠近影像邊緣」。預設為 false。
  // var targets = result.getTargets();// 回傳檢測到的所有 target（每個 target 代表一個 AprilTag
  // 或偵測到的標記物）。我們要逐個檢查每個 target 的角點位置。
  // for (var tgt : targets) {// 開始迭代每一個被偵測到的 target
  // var corners = tgt.detectedCorners;// detectedCorners 通常是 target
  // 在影像中的四個角點（通常為像素座標），可能是 List<Point>
  // // 或類似結構。注意它可能為 null（取決於 Photon 的版本或資料是否完整），
  // // 所以要做 null 檢查
  // if (corners != null) {// 只有當角點資料存在時才進入下一層迭代
  // for (var corner : corners) {// 迭代該 target 的每一個角點（通常是四個）
  // if (corner == null)
  // continue;// 防守式程式（defensive）檢查：如果某個角點為 null，跳過它（避免 NPE）
  // // Photon's corner.x/y 是 pixel（0..width/height）
  // // 提醒你 corner 的座標是以像素為單位（畫面左上為 0,0，右下大約為 width, height）。不同相機解析度或模擬參數，這些值會不同。
  // /*
  // * 這四個子條件分別檢查角點是否「接近影像左邊/右邊/上邊/下邊」：
  // *
  // * Math.abs(corner.x - 0.0) < borderPixels → x 靠近左邊（x ≈ 0）
  // *
  // * Math.abs(corner.x - width) < borderPixels → x 靠近右邊（x ≈ width）
  // *
  // * Math.abs(corner.y - 0.0) < borderPixels → y 靠近上邊
  // *
  // * Math.abs(corner.y - height) < borderPixels → y 靠近下邊
  // */
  // // borderPixels 是允許的邊界厚度（像素），如果角點在這個厚度以內就判為「靠邊」。一旦發現靠邊，將 cornerNearEdge =
  // true 並
  // // break（跳出角點迴圈）。
  // if (Math.abs(corner.x - 0.0) < borderPixels || Math.abs(corner.y - 0.0) <
  // borderPixels) {
  // cornerNearEdge = true;
  // break;
  // }
  // }
  // }
  // if (cornerNearEdge)
  // break;// 跳出最外層的 targets 迴圈（已經知道這frame有靠邊角點，沒必要再檢查其他 target）
  // }
  // if (cornerNearEdge) {
  // continue;// 如果這一幀影像有任何角點靠邊 → 丟棄這一幀，不使用它的姿態估測結果
  // }

  // // 計算距離與可信度條件 --------------------------------------------
  // double avgDist = 0.0;// 用來累積並計算「平均距離」
  // int usedTags = 0;// 統計這一幀影像中實際被用來解算姿態的標籤數量
  // double closest = Double.POSITIVE_INFINITY;// 記錄「最近的標籤距離」，初始設為無限大
  // for (var tgt : poseOpt.get().targetsUsed) {// poseOpt.get().targetsUsed：這是
  // PhotonPoseEstimator
  // // 在這一幀中實際用來計算姿態的標籤集合。
  // double d = tgt.getBestCameraToTarget().getTranslation().getNorm();//
  // 計算相機到該標籤的距離（單位：公尺）
  // avgDist += d;// - 把距離加總到 avgDist
  // usedTags++;// 並增加 usedTags 計數
  // if (d < closest)
  // closest = d;// 如果這個距離比目前記錄的 closest 還小，就更新最近距離
  // }
  // if (usedTags == 0)
  // continue;// 如果沒有任何標籤被用來解算，這一幀就直接跳過
  // avgDist /= usedTags;// 否則，計算平均距離 avgDist

  // // 單 tag 時要求距離不能太遠（因為單標籤會有姿態不確定性）
  // if (usedTags < 2) {
  // if (avgDist > maxSingleTagDistanceMeters) {// 如果總距離大於最大距離限制 -> 捨棄這一幀
  // continue;
  // }
  // } else {
  // // 若多 tag，但距離全部都很遠，有可能不穩 → 可以拒絕或降低權重（這邊只降低權重）
  // }

  // double p = 1.0;/*- p = 1.0 → 權重與距離成反比。
  // - 標籤數越多 → 權重越大。
  // - 距離越近 → 權重越大。 */

  // double weight = usedTags * (1.0 / Math.max(avgDist, 0.5));
  // // 權重計算：可以根據距離、被用到的 tag 數、corner 數、標籤 id 是否在 reef 區域等決定
  // // 簡單策略： weight = (nTags) * (1 / distance^p), p=1..2
  // /*- 權重的基本公式：
  // \text{weight} = \text{標籤數量} \times \frac{1}{\text{平均距離}^p}- p = 1.0 →
  // 權重與距離成反比。
  // 標籤數越多 → 權重越大。
  // 距離越近 → 權重越大。
  // Math.max(avgDist, 0.5) → 避免除以零或距離過小造成權重爆炸 */

  // // 若最接近的 tag 非常近，給額外加權
  // if (closest < Units.metersToInches(2.0) / 39.37) {// 如果最近的標籤距離小於 2 英尺（約 0.6
  // 公尺），代表相機有看到非常近的標籤。
  // weight *= 1.5;// 近距離標籤通常更可靠，因此額外乘上 1.5 倍權重。
  // }

  // // 若 detector 回傳置信度（Photon 的 target.confidence），可乘上去（此處用 best target
  // confidence）
  // // PhotonVision 的 target 可能會有「置信度 (confidence)」欄位（不同版本支援不同）
  // if (result.getTargets().size() > 0 /* 這一幀有偵測到至少一個 AprilTag */ &&
  // result.getBestTarget() != null/*
  // * PhotonVision
  // * 幫你挑了一個「最佳目標」
  // */) {
  // // 貌似調權重使用的
  // weight *= Math.max(0.1, result.getBestTarget().getFiducialId() >= 0 ? 1.0 :
  // 1.0); // placeholder: 可用
  // // target.confidence//max 取
  // // X 和 0.1 之間的最大值
  // }

  // // 把 3D pose 轉成 2D pose（忽略 Z）
  // Pose2d robotPose2d = cameraRobotPose3d.toPose2d();

  // // 收集
  // // 把這一幀的姿態、權重、時間戳存進清單
  // poses.add(robotPose2d);// 姿態
  // weights.add(weight);// 權重
  // // Utils.fpgaToCurrentTime(resultTimeSec)：把 PhotonVision 的時間戳（協處理器時間）轉換成 FPGA
  // // 時間，確保與 WPILib 的 PoseEstimator 使用同一個時間基準
  // // convert camera / photon timestamp -> FPGA time so poseEstimator uses same
  // // clock
  // double fpgatime = Utils.fpgaToCurrentTime(resultTimeSec); // 如有問題，可改成
  // Timer.getFPGATimestamp() - latency
  // timestamps.add(fpgatime);// 時間戳存
  // }
  // }

  // // 若沒拿到任何可信觀測，直接 return
  // if (poses.isEmpty()) {
  // return;// - 如果沒有任何可信的觀測結果（poses 為空），就直接結束，不做融合
  // }

  // // 權重平均（x, y, angle）——角度以向量平均處理（避免 2π 跳變）
  // double sumW = 0.0;// 權重總和
  // double sumX = 0.0;// 加權後的 X 座標總和
  // double sumY = 0.0;//// 加權後的 Y 座標總和
  // double sumCosA = 0.0;// 角度的 cos 與 sin 加權總和（用來做向量平均)
  // double sumSinA = 0.0;//// 角度的 cos 與 sin 加權總和（用來做向量平均)
  // double latestTimestamp = 0.0;// 記錄最新（最大的）時間戳

  // for (int i = 0; i < poses.size(); i++) {
  // double w = Math.max(weights.get(i), minWeight);// w = 這個觀測的權重，至少要大於
  // minWeight（避免權重為 0）
  // Pose2d p = poses.get(i);// p = 這個觀測的姿態（Pose2d）
  // sumW += w;// 同時累積權重總和
  // sumX += p.getX() * w;// 把每個觀測的 X、Y 乘上權重後加總
  // sumY += p.getY() * w;// 把每個觀測的 X、Y 乘上權重後加總
  // /*- 角度不能直接數值平均，因為會有「2π 跳變」問題。
  // 例如：179° 和 -179° 的平均應該是 180°（或 -180°），但直接平均會得到 0°，完全錯誤。
  // - 解法：把角度轉成單位向量 (cosθ, sinθ)，再做加權平均。
  // - 最後再用 atan2(sumSinA, sumCosA) 算回平均角度。*/
  // sumCosA += Math.cos(p.getRotation().getRadians()) * w;
  // sumSinA += Math.sin(p.getRotation().getRadians()) * w;
  // // 使用最晚的 timestamp（或也可以做加權時間）
  // latestTimestamp = Math.max(latestTimestamp, timestamps.get(i));//
  // 這樣融合後的姿態會對應到最新的觀測時間
  // }

  // double fusedX = sumX / sumW;// - sumX、sumY 是前面迴圈累積的「加權後座標總和」。
  // double fusedY = sumY / sumW;// - sumW 是權重總和
  // double fusedTheta = Math.atan2(sumSinA / sumW, sumCosA / sumW);// - 這裡用
  // atan2(sumSinA / sumW, sumCosA / sumW)
  // // 算回平均角度。

  // Pose2d fused = new Pose2d(fusedX, fusedY, new Rotation2d(fusedTheta));// -
  // 把加權平均後的 X、Y、角度組合成一個新的 Pose2d
  // // 最後把 fused pose 與對應時間戳回饋給 pose estimator
  // // SwerveDrivePoseEstimator.addVisionMeasurement(Pose2d visionPose, double
  // // timestamp)
  // /*
  // * poseEstimator.addVisionMeasurement(fused, latestTimestamp);- 把融合後的姿態 fused
  // * 和對應的時間戳 latestTimestamp 傳給 SwerveDrivePoseEstimator。
  // * - SwerveDrivePoseEstimator 會把這個「視覺觀測」和「里程計 (odometry)」融合，得到更準確的機器人位置
  // */
  // // optional: 若你不使用 SwerveDrivePoseEstimator，可以改成呼叫你的
  // // drive.addVisionMeasurement(fused, latestTimestamp)

  // double forceTimestamp = Timer.getFPGATimestamp();
  // drive.OVaddVisionMeasurement(fused, forceTimestamp);
  // }
}
