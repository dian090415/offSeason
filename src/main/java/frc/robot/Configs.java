package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Configs {
    public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {
        double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                / ModuleConstants.kDrivingMotorReduction;
        double turningFactor = ModuleConstants.kTurningGearRaitio * 2 * Math.PI;
        double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

        drivingConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50);
        drivingConfig.encoder
                .positionConversionFactor(drivingFactor)
                .velocityConversionFactor(drivingFactor / 60.0);
        drivingConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.13, 0.000, 0)
                .velocityFF(drivingVelocityFeedForward)
                .outputRange(-1, 1);

        turningConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40);
        turningConfig.encoder
                .positionConversionFactor(turningFactor)
                .velocityConversionFactor(turningFactor / 60.0);
        turningConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(1.65, 0.0007, 0)
                .outputRange(-1, 1)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(-Math.PI, Math.PI);
    }

}
