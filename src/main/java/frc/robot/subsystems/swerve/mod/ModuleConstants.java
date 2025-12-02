package frc.robot.subsystems.swerve.mod;

import edu.wpi.first.math.geometry.Rotation2d;

public class ModuleConstants {

    public final int driveMotorId;
    public final int angleMotorId;
    public final int canCoderId;
    public final Rotation2d angleOffset;

    private ModuleConstants(int driveMotorId, int angleMotorId, int canCoderId,
        Rotation2d angleOffset) {
        this.driveMotorId = driveMotorId;
        this.angleMotorId = angleMotorId;
        this.canCoderId = canCoderId;
        this.angleOffset = angleOffset;
    }

    public static Builder1 driveMotorId(int v) {
        return new Builder1(v);
    }

    public static class Builder1 {
        private final int driveMotorId_;

        private Builder1(int driveMotorId) {
            driveMotorId_ = driveMotorId;
        }

        public Builder2 angleMotorId(int v) {
            return new Builder2(driveMotorId_, v);
        }
    }

    public static class Builder2 {
        private final int driveMotorId_;
        private final int angleMotorId_;

        private Builder2(int driveMotorId, int angleMotorId) {
            driveMotorId_ = driveMotorId;
            angleMotorId_ = angleMotorId;
        }

        public Builder3 canCoderId(int v) {
            return new Builder3(driveMotorId_, angleMotorId_, v);
        }
    }

    public static class Builder3 {
        private final int driveMotorId_;
        private final int angleMotorId_;
        private final int canCoderId_;

        private Builder3(int driveMotorId, int angleMotorId, int canCoderId) {
            driveMotorId_ = driveMotorId;
            angleMotorId_ = angleMotorId;
            canCoderId_ = canCoderId;
        }

        public ModuleConstants angleOffset(Rotation2d v) {
            return new ModuleConstants(driveMotorId_, angleMotorId_, canCoderId_, v);
        }
    }



}
