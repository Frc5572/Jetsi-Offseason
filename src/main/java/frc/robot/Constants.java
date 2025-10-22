package frc.robot;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import frc.robot.subsystems.swerve.Mk4iReduction;
import frc.robot.subsystems.swerve.ModuleConfig;
import frc.robot.subsystems.swerve.TunableSwerveConstants;

/**
 * Constants file.
 */
public final class Constants {

    public static class Swerve {

        public static final Frequency ODOMETRY_FREQUENCY = Hertz.of(200);

        // @formatter:off
        public static final TunableSwerveConstants TUNABLE = TunableSwerveConstants.builder()
            .wheelRadius(Inches.of(3.87 / 2))
            .maxSpeed(MetersPerSecond.of(4.0))
            .maxAngularVelocity(RotationsPerSecond.of(1.0))
            .drivekS(1.0)
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

        public static final double DRIVE_REDUCTION = Mk4iReduction.L1.reduction;
        public static final double ANGLE_REDUCTION = Mk4iReduction.TURN.reduction;

    }

}
