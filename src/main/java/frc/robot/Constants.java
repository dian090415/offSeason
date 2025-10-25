// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Per;
import frc.robot.util.Swerve.ModuleLimits;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Degrees;

import java.util.Map;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;

public final class Constants {

    public static final class ModuleConstants {
        public static final double WilliamConstant = 1.042;
        public static final double kDrivingMotorFreeSpeedRps = MotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.10068;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        public static final double kDrivingMotorReduction = 5.95 * WilliamConstant;
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
            / kDrivingMotorReduction;
        public static final double kTurningGearRaitio = 1 / 19.6;
    }

    public static final class DriveConstants {

        public static final double kMaxSpeedMeterPerSecond = 2;
        public static final double kMaxAngularSpeedRadiansPerSecond = 1 * 1.8 * Math.PI;

        public static final double kMaxAccerationUnitsPerSecond = 5;

        public static final int[] kDriveMotorID = { 1, 2, 3, 4 };
        public static final int[] kTurnMotorID = { 5, 6, 7, 8 };
        public static final int[] kCANcoderID = { 9, 10, 11, 12 };

        public static final Translation2d[] moduleLocations = new Translation2d[] {
                new Translation2d(0.278, 0.278),
                new Translation2d(0.278, -0.278),
                new Translation2d(-0.278, 0.278),
                new Translation2d(-0.278, -0.278)
        };

        public static final Translation2d[] autoLocations = new Translation2d[] {
            new Translation2d(-0.278, -0.278),
            new Translation2d(-0.278, 0.278),
            new Translation2d(0.278, -0.278),
            new Translation2d(0.278, 0.278)
        };

        public static final ModuleLimits moduleLimitsFree = new ModuleLimits(kMaxSpeedMeterPerSecond,
                kMaxAccerationUnitsPerSecond, Units.degreesToRadians(1080.0));
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final int[] kOperatorReefSelecterButtonId = {
                11, 12, 16, 15, 14, 13, 10, 9, 5, 6, 7, 8
        };

        public static final double kDeadband = 0.06;
    }

    public static final class MotorConstants {
        public static final double kFreeSpeedRpm = 6784;
    }

    public static final class robotConstants {
        public static final double bumper = 10.0;
    }

    public static final class ElevatorConstants {
        public static final Per<AngleUnit, DistanceUnit> MOTOR_ROTATIONS_PER_METER_UNIT = Rotations.of(1)
                .div(Inches.of(13.0 / 50.0 * (Math.PI * 1.4397) * 2));// div馬達圈數/電梯移動的距離
        public static final double MOTOR_ROTATIONS_PER_METER = MOTOR_ROTATIONS_PER_METER_UNIT.in(Rotations.per(Meter));// 其實就是捲捲毛算給我的

        public static final Distance MIN_LENGTH = Inches.of(27.0);
        public static final Distance MIN_PADDED_LENGTH = MIN_LENGTH.plus(Inches.of(0.5));// 在最小長度的基礎上，加上 0.5 英吋作為安全墊
        public static final Distance MAX_LENGTH = Inches.of(66.0);
        public static final double MIN_LENGTH_ROTATIONS = MIN_LENGTH.in(Meters)
                * MOTOR_ROTATIONS_PER_METER;
        public static final double MAX_LENGTH_ROTATIONS = MAX_LENGTH.in(Meters)
                * MOTOR_ROTATIONS_PER_METER;

    }

    public static final class WristConstants {
        public static final double K_S = 0;
        public static final double K_V = 4.548;
        public static final double K_A = 0.2 * 0.45 / 0.25;
        public static final double MOTOR_ROTATIONS_PER_ARM_ROTATION = 48.0 / 9.0 * 40.0 / 15.0 * 40.0 / 15.0;
        public static final Angle CCW_LIMIT = Degrees.of(146.8);
        public static final Angle CW_LIMIT = Degrees.of(-70);
        public static final Angle K_G_ANGLE = Degrees.of(35.06);// Rotations.of(-0.072);
        public static final Angle K_G_ANGLE_WITH_CORAL = Degrees.of(45);
        public static final double K_G = 0.45;

    }
        public static class ReefConstants {
        public static final Map<Integer, Pose2d> REEF_alage = Map.ofEntries(
            Map.entry(6, new Pose2d(13.75, 2.75, new Rotation2d(2.095))),
            Map.entry(7, new Pose2d(14.45, 4.020, new Rotation2d(3.139))),
            Map.entry(8, new Pose2d(13.75, 5.2, new Rotation2d(-2.095))),
            Map.entry(9, new Pose2d(12.4, 5.25, new Rotation2d(-1.047))),
            Map.entry(10, new Pose2d(11.65, 4.02, new Rotation2d(0.000))),
            Map.entry(11, new Pose2d(12.365, 2.8, new Rotation2d(1.047))),
            Map.entry(17, new Pose2d(3.8, 2.85, new Rotation2d(1.047))),
            Map.entry(18, new Pose2d(3.15, 4.021, new Rotation2d(0.000))),
            Map.entry(19, new Pose2d(3.8, 5.2, new Rotation2d(-1.047))),
            Map.entry(20, new Pose2d(5.15, 5.2, new Rotation2d(-2.095))),
            Map.entry(21, new Pose2d(5.85, 4.021, new Rotation2d(3.141))),
            Map.entry(22, new Pose2d(5.15, 2.85, new Rotation2d(2.095)))
        );
        public static final Map<Integer, Pose2d> REEF_Left = Map.ofEntries(
            Map.entry(6, new Pose2d(13.61, 2.755, new Rotation2d(2.095))),
            Map.entry(7, new Pose2d(14.45, 3.8575, new Rotation2d(3.139))),
            Map.entry(8, new Pose2d(13.893, 5.126, new Rotation2d(-2.095))),
            Map.entry(9, new Pose2d(12.521, 5.2989, new Rotation2d(-1.047))),
            Map.entry(10, new Pose2d(11.6972, 4.1906, new Rotation2d(0.000))),
            Map.entry(11, new Pose2d(12.2325, 2.913, new Rotation2d(1.047))),
            Map.entry(17, new Pose2d(3.6641 , 2.931, new Rotation2d(1.047))),
            Map.entry(18, new Pose2d(3.1193, 4.19, new Rotation2d(0.000))),
            Map.entry(19, new Pose2d(3.9454, 5.2894, new Rotation2d(-1.047))),
            Map.entry(20, new Pose2d(5.3164, 5.1214, new Rotation2d(-2.095))),
            Map.entry(21, new Pose2d(5.85, 3.8575, new Rotation2d(3.141))),
            Map.entry(22, new Pose2d(5.0255, 2.766, new Rotation2d(2.095)))
        );
        public static final Map<Integer, Pose2d> REEF_Right = Map.ofEntries(
            Map.entry(6, new Pose2d(13.8961, 2.9261, new Rotation2d(2.095))),
            Map.entry(7, new Pose2d(14.45, 4.185, new Rotation2d(3.139))),
            Map.entry(8, new Pose2d(13.6008, 5.28, new Rotation2d(-2.095))),
            Map.entry(9, new Pose2d(12.2388, 5.1266, new Rotation2d(-1.047))),
            Map.entry(10, new Pose2d(11.6972, 3.855, new Rotation2d(0.000))),
            Map.entry(11, new Pose2d(12.5246, 2.7644, new Rotation2d(1.047))),
            Map.entry(17, new Pose2d(3.9518 , 2.7648, new Rotation2d(1.047))),
            Map.entry(18, new Pose2d(3.1193, 3.855, new Rotation2d(0.000))),
            Map.entry(19, new Pose2d(3.662, 5.1251, new Rotation2d(-1.047))),
            Map.entry(20, new Pose2d(5.0252, 5.2832, new Rotation2d(-2.095))),
            Map.entry(21, new Pose2d(5.85, 4.185, new Rotation2d(3.141))),
            Map.entry(22, new Pose2d(5.3237, 2.9167, new Rotation2d(2.095)))
        );
    }
}
