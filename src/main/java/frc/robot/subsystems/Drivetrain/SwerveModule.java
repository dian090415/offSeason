package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ModuleConstants;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase;

public class SwerveModule {

    private final SparkFlex driveMotor; // 馬達定義
    private final SparkMax turningMotor;

    private final SparkFlexConfig FlexConf = new SparkFlexConfig(); // 馬達設定
    private final SparkMaxConfig MaxConf = new SparkMaxConfig();

    private final RelativeEncoder driveEncoder; // 馬達編碼器(馬達內部，斷電即丟失數據)
    private final RelativeEncoder turningEncoder;

    private final SparkClosedLoopController driveController; // PID控制器 (Spark系->SparkClosedLoop CTRE系在Config可設定
                                                             // 其餘則用PIDController(萬能)
    private final SparkClosedLoopController turningController;

    private final CANcoder absoluteEncoder; // 絕對編碼器，負責初始化階段取得絕對朝向以校準馬達編碼器

    private final boolean absoluteEncoderReversed; // 絕對編碼器是否反轉

    double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps; // kFF (SparkClosedLoop用)

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReverdsed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderReversed = absoluteEncoderReversed;

        absoluteEncoder = new CANcoder(absoluteEncoderId);

        driveMotor = new SparkFlex(driveMotorId, MotorType.kBrushless); // NEO為無刷，千萬不要設錯，不然會燒掉
        turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);

        FlexConf// 駕駛馬達之設定(SparkFlex)
                .idleMode(IdleMode.kBrake) // 煞車模式，在不給動力時有阻力維持停止，確保機器不會滑行
                .smartCurrentLimit(50) // 電流限制，調高雖然能提升性能，但馬達會承受不住燒掉，詳細數值要問全向輪供應商
                .inverted(driveMotorReverdsed) // 是否反轉
                .apply(FlexConf); // 套用設定
        FlexConf.encoder// 編碼器設定
                .positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter) // 馬達數值轉換 (主要是齒比與單位)
                .velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec)
                .apply(FlexConf.encoder); // 套用
        FlexConf.closedLoop// PID控制器設定控制器設定
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder) // 反饋傳感器
                .pid(0.13, 0.0, 0.0) // PID數值
                .velocityFF(drivingVelocityFeedForward) // FF數值 (反饋反饋)
                .outputRange(-1, 1) // 輸出範圍 (馬達輸入只有-1 ~ 1)
                .apply(FlexConf.closedLoop); // 套用

        // 同上
        MaxConf
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(30)
                .inverted(turningMotorReversed)
                .apply(MaxConf);
        MaxConf.encoder
                .positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad)
                .velocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec)
                .apply(MaxConf.encoder);
        MaxConf.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(1.6, 0.0005, 0)
                .outputRange(-1, 1)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(-Math.PI, Math.PI)
                .apply(MaxConf.closedLoop);

        //
        // 馬達.configure(配置，只重設安全參數，寫入馬達記憶體) (不用這個沒法套用配置)
        driveMotor.configure(FlexConf, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turningMotor.configure(MaxConf, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // 讀馬達裡面的編碼器
        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        // 讀馬達裡面的PID控制器
        driveController = driveMotor.getClosedLoopController();
        turningController = turningMotor.getClosedLoopController();
        resetEncoders();
    }
    // 取得編碼器數值

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    // 取得絕對編碼器數值(角度)

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
        angle *= absoluteEncoderReversed ? -1.0d : 1.0d;
        SmartDashboard.putNumber("absoluteAngle", angle);
        return angle;
    }

    // 編碼器歸零

    public void resetEncoders() {
        driveEncoder.setPosition(0); // 歸零
        turningEncoder.setPosition(getAbsoluteEncoderRad()); // 不能歸零，會失去目前朝向
    }

    // 取得全向輪狀態(速度、角度)

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(),
                Rotation2d.fromRadians(getTurningPosition()));
    }

    // 設定全向輪狀態(速度、角度)

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = new SwerveModuleState();
        state.angle = Rotation2d.fromRadians(getTurningPosition());
        desiredState.optimize(state.angle); // 若目標朝向與目前插值>=180deg，改為旋轉另一方向並反轉車輪

        driveController.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity);
        turningController.setReference(desiredState.angle.getRadians(), ControlType.kPosition); // 套用PID處裡後數據置馬達
        SmartDashboard.putNumber("turnspeed", turningEncoder.getVelocity());
    }

    // 取得全向輪數值(整顆)

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                -driveEncoder.getPosition(),
                Rotation2d.fromRadians(-getTurningPosition()));
    }

    // 停止機器

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
        try {
            System.out.println("Robot Stopped!!!");
        } catch (Exception e) {
            System.err.println("Error stopping robot: " + e.getMessage());
        }
    }
}
