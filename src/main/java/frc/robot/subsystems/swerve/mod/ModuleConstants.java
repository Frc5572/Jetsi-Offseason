package frc.robot.subsystems.swerve.mod;

import edu.wpi.first.math.geometry.Rotation2d;

/** Per-module constants */
public class ModuleConstants {

    /** CAN ID for the drive motor */
    public final int driveMotorId;

    /** CAN ID for the angle motor */
    public final int angleMotorId;

    /** CAN ID for the CANCoder */
    public final int canCoderId;

    /** Reported angle when wheel is straight */
    public final Rotation2d angleOffset;

    private ModuleConstants(int driveMotorId, int angleMotorId, int canCoderId,
        Rotation2d angleOffset) {
        this.driveMotorId = driveMotorId;
        this.angleMotorId = angleMotorId;
        this.canCoderId = canCoderId;
        this.angleOffset = angleOffset;
    }

    /** Create builder */
    public static Builder1 driveMotorId(int v) {
        return new Builder1(v);
    }

    /** Internal builder */
    public static class Builder1 {
        private final int driveMotorId_;

        private Builder1(int driveMotorId) {
            driveMotorId_ = driveMotorId;
        }

        /** Continue builder */
        public Builder2 angleMotorId(int v) {
            return new Builder2(driveMotorId_, v);
        }
    }

    /** Internal builder */
    public static class Builder2 {
        private final int driveMotorId_;
        private final int angleMotorId_;

        private Builder2(int driveMotorId, int angleMotorId) {
            driveMotorId_ = driveMotorId;
            angleMotorId_ = angleMotorId;
        }

        /** Continue builder */
        public Builder3 canCoderId(int v) {
            return new Builder3(driveMotorId_, angleMotorId_, v);
        }
    }

    /** Internal builder */
    public static class Builder3 {
        private final int driveMotorId_;
        private final int angleMotorId_;
        private final int canCoderId_;

        private Builder3(int driveMotorId, int angleMotorId, int canCoderId) {
            driveMotorId_ = driveMotorId;
            angleMotorId_ = angleMotorId;
            canCoderId_ = canCoderId;
        }

        /** Finish builder */
        public ModuleConstants angleOffset(Rotation2d v) {
            return new ModuleConstants(driveMotorId_, angleMotorId_, canCoderId_, v);
        }
    }



}
