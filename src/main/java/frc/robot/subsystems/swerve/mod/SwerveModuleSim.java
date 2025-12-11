package frc.robot.subsystems.swerve.mod;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.util.PhoenixOdometryThread;

/** Simulation implementation for Swerve Module */
public class SwerveModuleSim implements SwerveModuleIO {

    /** Simulation implementation for Swerve Module */
    public SwerveModuleSim(int index, PhoenixOdometryThread odometryThread) {

    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {

        throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
    }

    @Override
    public void runDriveOpenLoop(double output) {

        throw new UnsupportedOperationException("Unimplemented method 'runDriveOpenLoop'");
    }

    @Override
    public void runAngleOpenLoop(double output) {

        throw new UnsupportedOperationException("Unimplemented method 'runAngleOpenLoop'");
    }

    @Override
    public void runDriveVelocity(double velocityRadPerSec, double feedforward) {

        throw new UnsupportedOperationException("Unimplemented method 'runDriveVelocity'");
    }

    @Override
    public void runAnglePosition(Rotation2d rotation) {

        throw new UnsupportedOperationException("Unimplemented method 'runAnglePosition'");
    }

    @Override
    public void setDrivePID(double kP, double kI, double kD, double kS, double kV, double kA) {

        throw new UnsupportedOperationException("Unimplemented method 'setDrivePID'");
    }

    @Override
    public void setAnglePID(double kP, double kI, double kD) {

        throw new UnsupportedOperationException("Unimplemented method 'setAnglePID'");
    }

    @Override
    public void setDriveBrakeMode(boolean enabled) {

        throw new UnsupportedOperationException("Unimplemented method 'setDriveBrakeMode'");
    }

    @Override
    public void setAngleBrakeMode(boolean enabled) {

        throw new UnsupportedOperationException("Unimplemented method 'setAngleBrakeMode'");
    }

}
