package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;
import frc.robot.util.swerve.CommonReductions;
import frc.robot.util.swerve.ModuleConfig;
import frc.robot.util.swerve.TunableSwerveConstants;

/**
 * Constants file.
 */
public final class Constants {

    public static class Swerve {

        public static final Frequency ODOMETRY_FREQUENCY = Hertz.of(200);

        public static class CurrentLimits {

            public static final Current DRIVE_CURRENT_LIMIT = Amps.of(35.0);
            public static final Current DRIVE_CURRENT_LOWER_LIMIT = Amps.of(60.0);
            public static final Time DRIVE_CURRENT_LOWER_TIME_THRESHOLD = Seconds.of(0.1);
            public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

            public static final Current ANGLE_CURRENT_LIMIT = Amps.of(25);
            public static final Current ANGLE_CURRENT_THRESHOLD = Amps.of(40);
            public static final Time ANGLE_CURRENT_THRESHOLD_TIME = Seconds.of(0.1);
            public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;
        }

        // @anchor:tunable
        // @formatter:off
        public static final TunableSwerveConstants TUNABLE = TunableSwerveConstants.builder()
            .wheelRadius(Inches.of(3.87 / 2))
            .maxSpeed(MetersPerSecond.of(4.0))
            .maxAcceleration(MetersPerSecondPerSecond.of(4.0))
            .maxAngularVelocity(RotationsPerSecond.of(1.0))
            .maxSterringVelocity(RotationsPerSecond.of(20.0))
            .drivekS(1.0)
            .drivekT(0.0)
            .drivekV(1.51)
            .drivekA(0.27)
            .drivekP(0.12)
            .drivekD(0.0)
            .anglekP(100.0)
            .anglekD(0.0)
            .build();
        // @formatter:on

        public static final Distance TRACK_WIDTH = Inches.of(24.229);
        public static final Distance WHEEL_BASE = Inches.of(24.229);
        public static final Distance DRIVE_BASE_RADIUS =
            Meters.of(Math.hypot(TRACK_WIDTH.in(Meters) / 2, WHEEL_BASE.in(Meters) / 2));

        public static final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[] {
            new Translation2d(WHEEL_BASE.in(Meters) / 2, TRACK_WIDTH.in(Meters) / 2),
            new Translation2d(WHEEL_BASE.in(Meters) / 2, -TRACK_WIDTH.in(Meters) / 2),
            new Translation2d(-WHEEL_BASE.in(Meters) / 2, TRACK_WIDTH.in(Meters) / 2),
            new Translation2d(-WHEEL_BASE.in(Meters) / 2, -TRACK_WIDTH.in(Meters) / 2)};

        public static final SwerveDriveKinematics SWERVE_KINEMATICS =
            new SwerveDriveKinematics(MODULE_TRANSLATIONS);

        public static final InvertedValue ANGLE_MOTOR_INVERT = InvertedValue.Clockwise_Positive;
        public static final InvertedValue DRIVE_MOTOR_INVERT =
            InvertedValue.CounterClockwise_Positive;
        public static final SensorDirectionValue CANCODER_INVERT =
            SensorDirectionValue.CounterClockwise_Positive;
        public static final NeutralModeValue ANGLE_NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;

        public static final ModuleConfig[] MODULE_CONFIGS = new ModuleConfig[] {
            // @anchor:modules
            // @formatter:off
                // Front Left Module
                ModuleConfig.builder()
                    .driveMotorId(2)
                    .angleMotorId(1)
                    .canCoderId(1)
                    .angleOffset(Rotation2d.fromRotations(0.008789))
                    .build(),
                    
                // Front Right Module
                ModuleConfig.builder()
                    .driveMotorId(9)
                    .angleMotorId(8)
                    .canCoderId(2)
                    .angleOffset(Rotation2d.fromRotations(-0.301758))
                    .build(),
                    
                // Back Left Module
                ModuleConfig.builder()
                    .driveMotorId(0)
                    .angleMotorId(19)
                    .canCoderId(4)
                    .angleOffset(Rotation2d.fromRotations(-0.451172))
                    .build(),
                    
                // Back Right Module
                ModuleConfig.builder()
                    .driveMotorId(11)
                    .angleMotorId(10)
                    .canCoderId(3)
                    .angleOffset(Rotation2d.fromRotations(0.321777))
                    .build(),

            // @formatter:on
        };

        public static final double DRIVE_REDUCTION = CommonReductions.Mk4i.L1.reduction;
        public static final double ANGLE_REDUCTION = CommonReductions.Mk4i.TURN.reduction;

        /**
         * Maximum number of odometry readings between periodic calls. If main code runs at a
         * nominal 50Hz, and the odometry thread runs at a nominal 200Hz, then 4 should be
         * sufficient. However we use a larger number in cases of massive loop overruns, which could
         * result in an effective rate of as low as 10Hz.
         */
        public static final int MAX_ODOMETRY_SUBTICK_MEASUREMENTS = 20;

        /**
         * These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with wear, tipping, etc
         */
        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

    }

}
