package frc.lib.util.swerve;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;

/** Per-module constants */
public class ModuleConstants {

    /** Constants for vendor swerve modules */
    public static enum ModuleKind {
        Mk4i(5.36, 150.0 / 7.0, InvertedValue.Clockwise_Positive,
            InvertedValue.CounterClockwise_Positive,
            SensorDirectionValue.CounterClockwise_Positive), Mk4n(5.36, 18.75,
                InvertedValue.Clockwise_Positive, InvertedValue.CounterClockwise_Positive,
                SensorDirectionValue.CounterClockwise_Positive);

        public final double driveGearRatio;
        public final double angleGearRatio;
        public final InvertedValue angleMotorDirection;
        public final InvertedValue driveMotorDirection;
        public final SensorDirectionValue canCoderDirection;

        ModuleKind(double driveGearRatio_, double angleGearRatio_,
            InvertedValue angleMotorDirection_, InvertedValue driveMotorDirection_,
            SensorDirectionValue canCoderDirection_) {
            driveGearRatio = driveGearRatio_;
            angleGearRatio = angleGearRatio_;
            angleMotorDirection = angleMotorDirection_;
            driveMotorDirection = driveMotorDirection_;
            canCoderDirection = canCoderDirection_;
        }


    }

    public final ModuleKind kind;

    /** CAN ID for the drive motor */
    public final int driveMotorId;

    /** CAN ID for the angle motor */
    public final int angleMotorId;

    /** CAN ID for the CANCoder */
    public final int canCoderId;

    /** Reported angle when wheel is straight */
    public final Rotation2d angleOffset;

    /** Per-module constants */

    public ModuleConstants(int driveMotorId, int angleMotorId, int canCoderId,
        Rotation2d angleOffset, ModuleKind kind) {
        this.driveMotorId = driveMotorId;
        this.angleMotorId = angleMotorId;
        this.canCoderId = canCoderId;
        this.angleOffset = angleOffset;
        this.kind = kind;
    }

}
