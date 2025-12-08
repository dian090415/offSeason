package frc.robot.subsystems.NewDrive;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.Swerve.Module.Module;

public class driveIOHardware implements driveIO {

    private final Module FL, FR, BL, BR;
    private final AHRS gyro;

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
    }

    @Override
    public double getHeading() {
        return -this.gyro.getAngle();
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
        return gyro.getRate();
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
}
