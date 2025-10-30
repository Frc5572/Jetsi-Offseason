package frc.robot.subsystems.swerve.module;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public interface ModuleIO {

    @AutoLog
    public static class ModuleInputs {
        public boolean driveConnected;
        public double drivePositionRad;
        public double driveVelocityRadPerSec;
        public double driveAppliedVolts;

        public boolean angleConnected;
        public Rotation2d anglePosition = new Rotation2d();
        public double angleVelocityRadsPerSec;
        public double angleAppliedVolts;

        public boolean absoluteAngleConnected;
        public Rotation2d angleAbsolutePosition = new Rotation2d();

        public double[] odometryDrivePositionsRad =
            new double[Constants.Swerve.MAX_ODOMETRY_SUBTICK_MEASUREMENTS];
        public Rotation2d[] odometryTurnPositions =
            new Rotation2d[Constants.Swerve.MAX_ODOMETRY_SUBTICK_MEASUREMENTS];
    }

    public void updateInputs(ModuleInputs inputs);

    public void driveOpenLoop(double output);

    public void turnOpenLoop(double output);

    public void driveVelocity(double radPerSec, double ff);

    public void turnPosition(Rotation2d heading);

    public void setDrivePID(double kP, double kI, double kD, double kS, double kV, double kA);

    public void setAnglePID(double kP, double kI, double kD);

    public void setBrakeMode(boolean enabled);

}
