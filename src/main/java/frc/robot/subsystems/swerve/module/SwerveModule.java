package frc.robot.subsystems.swerve.module;

import static edu.wpi.first.units.Units.Meters;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveModule {

    private final ModuleIO io;
    public final ModuleInputsAutoLogged inputs = new ModuleInputsAutoLogged();

    private final String logKey;

    public SwerveModule(ModuleIO io, int index) {
        this.io = io;
        this.logKey = "Swerve/Module" + index;
    }

    public void updateInputs() {
        this.io.updateInputs(this.inputs);
        Logger.processInputs(this.logKey, this.inputs);
    }

    private final SwerveModulePosition[] odometryPositions =
        new SwerveModulePosition[Constants.Swerve.MAX_ODOMETRY_SUBTICK_MEASUREMENTS];
    private SwerveModulePosition latest;

    public void periodic() {
        if (Constants.Swerve.TUNABLE.isAnglePIDdirty()) {
            this.io.setAnglePID(Constants.Swerve.TUNABLE.getAnglekP(), 0.0,
                Constants.Swerve.TUNABLE.getAnglekD());
        }

        if (Constants.Swerve.TUNABLE.isDrivePIDdirty()) {
            this.io.setDrivePID(Constants.Swerve.TUNABLE.getDrivekP(), 0.0,
                Constants.Swerve.TUNABLE.getDrivekD(), Constants.Swerve.TUNABLE.getDrivekS(),
                Constants.Swerve.TUNABLE.getDrivekV(), Constants.Swerve.TUNABLE.getDrivekA());
        }

        for (int i = 0; i < Constants.Swerve.MAX_ODOMETRY_SUBTICK_MEASUREMENTS; i++) {
            if (this.inputs.odometryTurnPositions[i] == null) {
                odometryPositions[i] = null;
            } else {
                odometryPositions[i] = new SwerveModulePosition(
                    this.inputs.odometryDrivePositionsRad[i]
                        * Constants.Swerve.TUNABLE.getWheelRadius().in(Meters),
                    this.inputs.odometryTurnPositions[i]);
            }
        }

        this.latest = new SwerveModulePosition(
            this.inputs.drivePositionRad * Constants.Swerve.TUNABLE.getWheelRadius().in(Meters),
            this.inputs.anglePosition);
    }

    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    public SwerveModulePosition getOdometryPosition() {
        return latest;
    }

    public double getVelocityMetersPerSec() {
        return inputs.drivePositionRad * Constants.Swerve.TUNABLE.getWheelRadius().in(Meters);
    }

    public Rotation2d getAngle() {
        return inputs.anglePosition;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    public void runSetpoint(SwerveModuleState state, double wheelTorqueNm) {
        double speedRadPerSec =
            state.speedMetersPerSecond / Constants.Swerve.TUNABLE.getWheelRadius().in(Meters);
        io.driveVelocity(speedRadPerSec, wheelTorqueNm * Constants.Swerve.TUNABLE.getDrivekT());

    }

    public void runCharacterization(double output) {
        io.driveOpenLoop(output);
        io.turnPosition(Rotation2d.kZero);
    }

}
