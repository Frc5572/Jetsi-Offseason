package frc.robot.subsystems.swerve.mod;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.util.PhoenixOdometryThread;
import frc.robot.util.GenerateEmptyIO;

@GenerateEmptyIO({int.class, PhoenixOdometryThread.class})
public interface SwerveModuleIO {

    @AutoLog
    public static class SwerveModuleInputs {
        public boolean driveConnected;
        public double drivePositionRad;
        public double driveVelocityRadPerSec;
        public double driveAppliedVolts;
        public double driveSupplyCurrentAmps;
        public double driveStatorCurrentAmps;

        public boolean angleConnected;
        public Rotation2d anglePosition = Rotation2d.kZero;
        public double angleVelocityRadPerSec;
        public double angleAppliedVolts;
        public double angleSupplyCurrentAmps;
        public double angleStatorCurrentAmps;

        public boolean absoluteAngleConnected;
        public Rotation2d angleAbsolutePosition = Rotation2d.kZero;

        public double[] odometryDrivePositionsRad = new double[0];
        public double[] odometryDriveVelocityRadsPerSec = new double[0];
        public Rotation2d[] odometryAnglePositions = new Rotation2d[0];
    }

    public void updateInputs(SwerveModuleInputs inputs);

    public void runDriveOpenLoop(double output);

    public void runAngleOpenLoop(double output);

    public void runDriveVelocity(double velocityRadPerSec, double feedforward);

    public void runAnglePosition(Rotation2d rotation);

    public void setDrivePID(double kP, double kI, double kD, double kS, double kV, double kA);

    public void setAnglePID(double kP, double kI, double kD);

    public void setDriveBrakeMode(boolean enabled);

    public void setAngleBrakeMode(boolean enabled);

}
