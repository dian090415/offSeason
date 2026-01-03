package frc.robot.subsystems.NewDrive;

import org.littletonrobotics.junction.Logger;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.subsystems.NewVision.Vision;
import frc.robot.subsystems.NewVision.VisionFuser;
import frc.robot.util.Swerve.SwerveSetpoint;
import frc.robot.util.Swerve.SwerveSetpointGenerator;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj.DriverStation;

public class drive extends SubsystemBase {

    private static drive autodrive;

    private final driveIO io;

    private final SwerveDrivePoseEstimator poseEstimator;

    private final Field2d field;

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DriveConstants.autoLocations);

    private final SwerveSetpointGenerator swerveSetpointGenerator;

    private SwerveSetpoint currentSetpoint = new SwerveSetpoint(
            new ChassisSpeeds(),
            new SwerveModuleState[] {
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState()
            });

    private final PIDController xController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController headingController = new PIDController(7.5, 0.0, 0.0);

    public drive(driveIO io) {

        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
                this::getPose, // 告訴 AutoBuilder 機器人在哪 (Pose2d)
                this::resetOdometry, // 告訴 AutoBuilder 如何重設位置
                this::getRobotRelativeSpeeds, // ❌ 你原本沒有這個方法，請看下面第 3 點新增
                (speeds, feedforwards) -> autorunVelocity(speeds), // ✅ 使用你寫好的 robot-relative 驅動方法
                new PPHolonomicDriveController( 
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID (請依照實際情況調整)
                        new PIDConstants(5.0, 0.0, 0.0)  // Rotation PID (請依照實際情況調整)
                ),
                config, 
                () -> {
                    // 自動判斷是否為紅隊 (路徑翻轉)
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Subsystem 參考
        );

        this.io = io;

        Pose2d initialPose = new Pose2d(0, 0, io.getRotation2d());

        this.poseEstimator = new SwerveDrivePoseEstimator(
                this.kinematics,
                io.getRotation2d(),
                this.io.getModulePositions(),
                initialPose);

        this.swerveSetpointGenerator = new SwerveSetpointGenerator(
                this.kinematics,
                DriveConstants.moduleLocations);

        this.field = new Field2d();

        io.zeroHeading();
        io.resetEncoders();
        headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    // 👇 新增這個方法給 AutoBuilder 用
    public ChassisSpeeds getRobotRelativeSpeeds() {
        // 將 4 顆輪子的狀態轉回底盤的 X, Y, Omega 速度
        return kinematics.toChassisSpeeds(io.getModuleStates());
    }

    public SwerveDrivePoseEstimator poseEstimator() {
        return poseEstimator;
    }

    public static drive getinDrive() {
        if (autodrive == null) {
            autodrive = new drive(null);
        }
        return autodrive;
    }

    public double getVelocity() {
        // 1. 從 IO 取得 4 顆輪子的當前狀態 (包含速度與角度)
        SwerveModuleState[] states = io.getModuleStates();

        // 2. 使用正向運動學 (Forward Kinematics) 將輪子速度轉換為底盤速度 (ChassisSpeeds)
        // kinematics 會算出 Vx (X軸速度), Vy (Y軸速度), Omega (旋轉速度)
        var chassisSpeeds = kinematics.toChassisSpeeds(states);

        // 3. 使用畢氏定理計算合速度 (√(vx² + vy²))
        // Math.hypot 是計算直角三角形斜邊的快速函式
        return Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    }

    public double getGyroYawRate() {
        return this.io.getGyroYawRate();
    }

    public Rotation2d getRotation2d() {
        return this.io.getRotation2d();
    }

    public void followTrajectory(SwerveSample sample) {
        // Get the current pose of the robot
        Pose2d pose = poseEstimator.getEstimatedPosition();

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
                sample.vx + xController.calculate(pose.getX(), sample.x),
                sample.vy + yController.calculate(pose.getY(), sample.y),
                sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading));

        // Apply the generated speeds
        runVelocity(speeds);
    }

    public void runVelocity(ChassisSpeeds speeds) {

        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, io.getRotation2d());

        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(fieldSpeeds, 0.02);

        SwerveModuleState[] SetPointStatesUnoptimized = kinematics.toSwerveModuleStates(discreteSpeeds);

        currentSetpoint = swerveSetpointGenerator.generateSetpoint(
                DriveConstants.moduleLimitsFree,
                currentSetpoint,
                discreteSpeeds,
                0.02);

        SwerveModuleState[] setpointStates = currentSetpoint.moduleStates();

        Logger.recordOutput("Drive/SwerveStates/SetpointsUnoptimized", SetPointStatesUnoptimized);
        Logger.recordOutput("Drive/SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("Drive/SwerveChassisSpeeds/Setpoints", currentSetpoint.chassisSpeeds());

        this.io.setModuleStates(setpointStates);
    }

    public void autorunVelocity(ChassisSpeeds robotRelativeSpeeds) { // 參數名稱改一下比較清楚

        // ❌ 刪除這行！不要在這裡做座標轉換
        // ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds,
        // io.getRotation2d());

        // ✅ 直接使用傳進來的 speeds (假設已經是機器人相對座標)
        // 這裡做 discretize 是對的 (修正飄移)
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        // ... 後面的邏輯保持不變 ...
        SwerveModuleState[] SetPointStatesUnoptimized = kinematics.toSwerveModuleStates(discreteSpeeds);

        currentSetpoint = swerveSetpointGenerator.generateSetpoint(
                DriveConstants.moduleLimitsFree,
                currentSetpoint,
                discreteSpeeds,
                0.02);

        SwerveModuleState[] setpointStates = currentSetpoint.moduleStates();

        Logger.recordOutput("Drive/SwerveStates/SetpointsUnoptimized", SetPointStatesUnoptimized);
        Logger.recordOutput("Drive/SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("Drive/SwerveChassisSpeeds/Setpoints", currentSetpoint.chassisSpeeds());

        this.io.setModuleStates(setpointStates);
    }

    public void autoAlign(double[] speeds) {
        this.runVelocity(new ChassisSpeeds(speeds[0], speeds[1], speeds[2]));
    }

    public void stop() {
        this.io.stopModules();
    }

    public void zeroHeading() {
        this.io.zeroHeading();
    }

    public Command resetHeading() {
        return run(() -> this.zeroHeading());

    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPose(pose);
    }

    public void OVaddVisionMeasurement(Pose2d pose, double timestampSeconds) {
        this.poseEstimator.addVisionMeasurement(pose, timestampSeconds);
    }

    public void addVisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        this.poseEstimator.addVisionMeasurement(getPose(), getGyroYawRate(), visionMeasurementStdDevs);
    }

    @Override
    public void periodic() {

        // field.setRobotPose(poseEstimator.getEstimatedPosition());

        poseEstimator.update(
                io.getRotation2d(),
                this.io.getModulePositions());

        double[] pose = { poseEstimator.getEstimatedPosition().getX(),
                poseEstimator.getEstimatedPosition().getY(),
                Math.toRadians(io.getHeading()) };

        SmartDashboard.putNumberArray("Pose", pose);

        SmartDashboard.putNumber("heading", io.getHeading());

        SmartDashboard.putNumber("turnRate", io.getTurnRate());
    }
    @Override
    public void simulationPeriodic() {
        // 1. 算出當前底盤的理論速度
        var speeds = kinematics.toChassisSpeeds(io.getModuleStates());
        
        // 2. 算出 20ms 內轉了多少度
        double angleChange = Math.toDegrees(speeds.omegaRadiansPerSecond * 0.02);
        
        // 3. 傳給 IO 更新 Gyro (IO 內部會把它反轉寫入)
        io.updateSimGyro(angleChange);
    }
}
