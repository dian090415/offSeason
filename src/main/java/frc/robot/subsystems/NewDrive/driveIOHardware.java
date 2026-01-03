package frc.robot.subsystems.NewDrive;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase; // 記得 import 這個
import frc.robot.Constants.DriveConstants;
import frc.robot.util.Swerve.Module.Module;

public class driveIOHardware implements driveIO {

    private final Module FL, FR, BL, BR;
    private final AHRS gyro;

    // 👇 【新增】這是我們的「虛擬 Gyro」，只在模擬時使用
    private double simHeading = 0.0;

    public driveIOHardware() {
        this.FL = new Module(0, true);
        this.FR = new Module(1, false);
        this.BL = new Module(2, true);
        this.BR = new Module(3, false);

        this.gyro = new AHRS(NavXComType.kMXP_SPI);
    }

    @Override
    public void zeroHeading() {
        this.gyro.reset();
        this.simHeading = 0.0; // 模擬時也要歸零
    }

    @Override
    public double getHeading() {
        // 👇 【關鍵修改】
        // 如果是真車，照舊用負號修正 NavX
        if (RobotBase.isReal()) {
            return -this.gyro.getAngle();
        } 
        // 如果是模擬，直接回傳我們自己算出來的正確角度 (不用負號)
        else {
            return simHeading;
        }
    }

    @Override
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(this.getHeading());
    }

    @Override
    public Rotation2d getOdometry2d() {
        return Rotation2d.fromDegrees(this.getHeading());
    }

    @Override
    public double getTurnRate() {
        if (RobotBase.isReal()) {
            return gyro.getRate();
        } else {
            return 0.0; // 模擬時暫時回傳 0 或另外計算，通常不影響 PathPlanner
        }
    }

    @Override
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                FL.getState(),
                FR.getState(),
                BL.getState(),
                BR.getState()
        };
    }

    @Override
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                FL.getPosition(),
                FR.getPosition(),
                BL.getPosition(),
                BR.getPosition()
        };
    }

    @Override
    public void setModuleStates(SwerveModuleState[] state) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                state,
                DriveConstants.kMaxSpeedMeterPerSecond);

        FL.setDesiredState(state[0]);
        FR.setDesiredState(state[1]);
        BL.setDesiredState(state[2]);
        BR.setDesiredState(state[3]);
    }

    @Override
    public void stopModules() {
        FL.Stop();
        FR.Stop();
        BL.Stop();
        BR.Stop();
    }

    @Override
    public void resetEncoders() {
        FL.resetEncoders();
        FR.resetEncoders();
        BL.resetEncoders();
        BR.resetEncoders();
    }

    @Override
    public double getGyroYawRate() {
        return this.gyro.getRate();
    }

    // 👇 【關鍵修改】直接更新變數，不透過 SimDevice
    @Override
    public void updateSimGyro(double angleChange) {
        // 因為傳進來的 angleChange 是由 Kinematics 算出的 (逆時針為正)
        // 而我們的 getHeading 在模擬時是直接回傳 simHeading
        // 所以這裡直接「加」上去即可，不用負號
        this.simHeading += angleChange;
    }
}