package frc.robot.subsystems.Drivetrain;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.Swerve.SwerveSetpoint;
import frc.robot.util.Swerve.SwerveSetpointGenerator;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

public class SwerveSubsystem extends SubsystemBase {

    private final PIDController xController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController headingController = new PIDController(7.5, 0.0, 0.0);
    private final Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("New Path");

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DriveConstants.moduleLocations);

    private final SwerveSetpointGenerator swerveSetpointGenerator;

    private SwerveSetpoint currentSetpoint = new SwerveSetpoint(
            new ChassisSpeeds(),
            new SwerveModuleState[] {
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState()
            });

    private final SwerveModule FL, FR, BL, BR; // 四個全向輪模組定義
    private final AHRS gyro; // 陀螺儀，用於獲取機器朝向和角速度

    private final SwerveDriveOdometry odometer; // 里程計(自動模式)
    private SlewRateLimiter xLimiter, yLimiter, turningLimiter; // 限速器，用於限制加速度和角速度

    private final Field2d Field = new Field2d(); // 用於顯示機器位置和路徑的Field2d物件

    public SwerveSubsystem() {
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        swerveSetpointGenerator = new SwerveSetpointGenerator(kinematics, DriveConstants.moduleLocations);

        // 初始化陀螺儀，並在啟動後1秒重置朝向
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();

            } catch (Exception e) {
            }
        }).start(); // 非同步執行，避免阻塞主線程

        // 定義每一個模組
        FL = new SwerveModule(
                DriveConstants.kFLDriveMotorPort, // 駕駛馬達編號
                DriveConstants.kFLTurningMotorPort, // 轉向馬達編號
                DriveConstants.kFLDriveEncoderReversed, // 駕駛馬達是否反轉
                DriveConstants.kFLTurningEncoderReversed, // 轉向馬達是否反轉
                DriveConstants.kFLDriveAbsoluteEncoderPort, // 絕對編碼器編號
                DriveConstants.kFLDriveAbsoluteEncoderOffsetRad, // 絕對編碼器偏移量(在Phoenix裡zeroHeading後可設為0)
                DriveConstants.kFLDriveAbsoluteEncoderReversed); // 絕對編碼器是否反轉

        FR = new SwerveModule(
                DriveConstants.kFRDriveMotorPort,
                DriveConstants.kFRTurningMotorPort,
                DriveConstants.kFRDriveEncoderReversed,
                DriveConstants.kFRTurningEncoderReversed,
                DriveConstants.kFRDriveAbsoluteEncoderPort,
                DriveConstants.kFRDriveAbsoluteEncoderOffsetRad,
                DriveConstants.kFRDriveAbsoluteEncoderReversed);

        BL = new SwerveModule(
                DriveConstants.kBLDriveMotorPort,
                DriveConstants.kBLTurningMotorPort,
                DriveConstants.kBLDriveEncoderReversed,
                DriveConstants.kBLTurningEncoderReversed,
                DriveConstants.kBLDriveAbsoluteEncoderPort,
                DriveConstants.kBLDriveAbsoluteEncoderOffsetRad,
                DriveConstants.kBLDriveAbsoluteEncoderReversed);

        BR = new SwerveModule(
                DriveConstants.kBRDriveMotorPort,
                DriveConstants.kBRTurningMotorPort,
                DriveConstants.kBRDriveEncoderReversed,
                DriveConstants.kBRTurningEncoderReversed,
                DriveConstants.kBRDriveAbsoluteEncoderPort,
                DriveConstants.kBRDriveAbsoluteEncoderOffsetRad,
                DriveConstants.kBRDriveAbsoluteEncoderReversed);
        //

        gyro = new AHRS(NavXComType.kMXP_SPI); // 陀螺儀

        resetEncoder(); // 重置所有模組編碼器

        odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
                getRotation2d(), getModulePosition()); // 里程計

        setOdometry();
    }

    public void followTrajectory(SwerveSample sample) {
        // Get the current pose of the robot
        Pose2d pose = getPose();

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
                sample.vx + xController.calculate(pose.getX(), sample.x),
                sample.vy + yController.calculate(pose.getY(), sample.y),
                sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading));

        // Apply the generated speeds
        runVelocity(speeds);
    }

    // 重置機器朝向
    public void zeroHeading() {
        gyro.reset();
    }

    // 將陀螺儀數值轉換為0 ~ 360
    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    // 將陀螺儀數值轉換為弧度
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Rotation2d turngetRotation2d() {
        return Rotation2d.fromDegrees(-getHeading());
    }
    // __________自動模式__________

    // 取得模組位置 (陀螺儀角度、模組位置)

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    // 重設里程計

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getModulePosition(), pose);
    }

    public void setOdometry() {
        odometer.resetPose(trajectory.get().getInitialPose(false).get());
    }

    // 取得全向輪Kinematics

    public SwerveDriveKinematics getKinematics() {
        return DriveConstants.kDriveKinematics;
    }

    // 自動模式建構器(PathPlanner需要)

    // ________________________________

    // 設定模組狀態 (給個別模組設定目標速度和角度)
    public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldRelative) {
        ChassisSpeeds targetSpeeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rotation);
        // 直接呼叫 runVelocity，裡面會處理所有模組狀態計算與馬達控制
        this.runVelocity(targetSpeeds);
    }

    public void runVelocity(ChassisSpeeds speeds) {

        // (1) 離散化速度：把速度調整成固定頻率 (讓控制更平滑、避免跳動太快)
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);

        // (2) 計算未優化的模組狀態（四個輪子的速度跟角度）
        SwerveModuleState[] setpointStatesUnoptimized = kinematics.toSwerveModuleStates(discreteSpeeds);

        // (3) 用 SwerveSetpointGenerator 生成平滑、符合限制的目標狀態
        currentSetpoint = swerveSetpointGenerator.generateSetpoint(
                DriveConstants.moduleLimitsFree,
                currentSetpoint,
                discreteSpeeds,
                0.02);
        SwerveModuleState[] setpointStates = currentSetpoint.moduleStates();

        Logger.recordOutput("Drive/SwerveStates/SetpointsUnoptimized", setpointStatesUnoptimized);
        Logger.recordOutput("Drive/SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("Drive/SwerveChassisSpeeds/Setpoints", currentSetpoint.chassisSpeeds());

        setStates(setpointStates);

    }

    public void setStates(SwerveModuleState[] targetStates) {

        // 去飽和化輪速 (確保每個輪子的速度不超過最大物理速度)

        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates,
                Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        // 設定每個模組的目標狀態 (速度和角度)

        FL.setDesiredState(targetStates[0]);
        FR.setDesiredState(targetStates[1]);
        BL.setDesiredState(targetStates[2]);
        BR.setDesiredState(targetStates[3]);

    }

    // 以機器相對速度驅動 (給定速度和角度)

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {

        // 將機器相對速度離散化 (將連續速度轉換為離散時間步長的速度)

        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        // 將機器相對速度轉換為模組狀態 (每個模組的速度和角度)

        SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);

        // 設定模組狀態 (將計算出的模組狀態應用到每個模組)

        setStates(targetStates);
    }

    // 以場地相對速度驅動 (給定速度和角度，並考慮機器當前朝向)

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {

        // 將場地相對速度轉換為機器相對速度

        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));

    }

    // 取得機器相對速度 (將模組狀態轉換為機器速度)

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    // 取得所有模組狀態 (每個模組的速度和角度)

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                FL.getState(),
                FR.getState(),
                BL.getState(),
                BR.getState()
        };
    }

    // ---------

    // 搖桿以外的控制方式(Apriltag校準等)
    public void driveRobot(double xSpeed, double ySpeed, double turningSpeed, boolean fieldRelative) {
        // 1. 限制加速度
        xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSec);
        yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSec);
        turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSec);
        // 1. 套用SlewRateLimiter
        double xSpd = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMeterPerSec;
        double ySpd = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMeterPerSec;
        double turnSpd = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSec;

        // 2. 計算 ChassisSpeeds
        ChassisSpeeds chassisSpeeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpd, ySpd, turnSpd, getRotation2d())
                : new ChassisSpeeds(xSpd, ySpd, turnSpd);

        // 3. 轉換成模組狀態並設定
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(moduleStates);
    }
    // ---------

    // 定期更新里程計和顯示數據

    @Override
    public void periodic() {
        Pose2d pose = getPose();
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putNumber("odo", odometer.getPoseMeters().getRotation().getDegrees());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        odometer.update(turngetRotation2d(), getModulePosition());
        Field.setRobotPose(pose);
        SmartDashboard.putData("Field", Field);
        SmartDashboard.putNumber("fieldX", Field.getRobotPose().getX());
        SmartDashboard.putNumber("fieldY", Field.getRobotPose().getY());
    }

    // 停止所有模組

    public void stopModules() {
        FL.stop();
        FR.stop();
        BL.stop();
        BR.stop();
    }

    // 重置所有模組編碼器

    public void resetEncoder() {
        FL.resetEncoders();
        FR.resetEncoders();
        BL.resetEncoders();
        BR.resetEncoders();
    }

    // 分別讓全向輪作動 (每個摩快要動的方向、速度可能不同)

    public void setModuleStates(SwerveModuleState[] desiredState) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredState, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        FL.setDesiredState(desiredState[0]);
        FR.setDesiredState(desiredState[1]);
        BL.setDesiredState(desiredState[2]);
        BR.setDesiredState(desiredState[3]);
    }

    // 給里程計提供每個全向輪數據 (里程計要把四顆模組的數據整合起來)

    public SwerveModulePosition[] getModulePosition() {

        return new SwerveModulePosition[] {
                FL.getPosition(),
                FR.getPosition(),
                BL.getPosition(),
                BR.getPosition()
        };
    }
}
