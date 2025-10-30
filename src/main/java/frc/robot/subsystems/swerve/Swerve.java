package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Meters;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BiFunction;
import java.util.function.Supplier;
import java.util.stream.IntStream;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroInputsAutoLogged;
import frc.robot.subsystems.swerve.module.ModuleIO;
import frc.robot.subsystems.swerve.module.SwerveModule;
import frc.robot.util.swerve.DrivetrainState;
import frc.robot.util.swerve.PhoenixOdometryThread;
import frc.robot.util.swerve.SwerveSetpoint;
import frc.robot.util.swerve.SwerveSetpointGenerator;

public class Swerve extends SubsystemBase {

    private final GyroIO io;
    private final GyroInputsAutoLogged inputs = new GyroInputsAutoLogged();
    private final SwerveModule[] modules;
    private final PhoenixOdometryThread odometryThread;
    private final Lock odometryLock = new ReentrantLock();
    private final Queue<Double> timestampQueue;
    private final TimestampsAutoLogged timestamps = new TimestampsAutoLogged();

    public final DrivetrainState state;

    private SwerveSetpoint currentSetpoint =
        new SwerveSetpoint(new ChassisSpeeds(), new SwerveModuleState[] {new SwerveModuleState(),
            new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()});
    private final SwerveSetpointGenerator swerveSetpointGenerator;

    private final SwerveModulePosition[] workingPositions = new SwerveModulePosition[4];

    public Swerve(GyroIO io, BiFunction<Integer, PhoenixOdometryThread, ModuleIO> modFn) {
        odometryThread = new PhoenixOdometryThread(odometryLock);
        this.modules =
            IntStream.range(0, 4).mapToObj(i -> new SwerveModule(modFn.apply(i, odometryThread), i))
                .toArray(SwerveModule[]::new);
        this.io = io;
        // Start odometry thread (if any signal are registered)
        this.timestampQueue = odometryThread.makeTimestampQueue();
        this.odometryThread.start();

        // Get initial data
        odometryLock.lock();
        this.io.updateInputs(this.inputs);
        Logger.processInputs("Swerve/Gyro", this.inputs);
        for (var mod : modules) {
            mod.updateInputs();
            mod.periodic();
        }
        odometryLock.unlock();

        for (int i = 0; i < 4; i++) {
            workingPositions[i] = modules[i].getOdometryPosition();
        }

        state = new DrivetrainState(workingPositions, inputs.yaw, Timer.getTimestamp());

        swerveSetpointGenerator = new SwerveSetpointGenerator(Constants.Swerve.SWERVE_KINEMATICS,
            Constants.Swerve.MODULE_TRANSLATIONS);
    }

    private boolean velocityMode;

    @Override
    public void periodic() {
        odometryLock.lock();
        this.io.updateInputs(this.inputs);
        Logger.processInputs("Swerve/Gyro", this.inputs);
        for (var mod : modules) {
            mod.updateInputs();
        }
        timestamps.fromQueue(timestampQueue);
        Logger.processInputs("Swerve/Timestamps", timestamps);
        odometryLock.unlock();

        for (var mod : modules) {
            mod.periodic();
        }

        for (int t = 0; t < timestamps.length; t++) {
            for (int i = 0; i < 4; i++) {
                workingPositions[i] = modules[i].getOdometryPositions()[t];
            }
            state.addSwerveObservation(workingPositions, inputs.odometryYaw[t],
                timestamps.timestamps[t]);
        }

        if (!velocityMode) {
            currentSetpoint = new SwerveSetpoint(getChassisSpeeds(), getModuleStates());
        }

        Constants.Swerve.TUNABLE.clean();
    }

    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    private ChassisSpeeds getChassisSpeeds() {
        return Constants.Swerve.SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    private void runVelocity(ChassisSpeeds speeds) {
        velocityMode = true;

        ChassisSpeeds discreteSpeeds =
            ChassisSpeeds.discretize(speeds, LoggedRobot.defaultPeriodSecs);
        SwerveModuleState[] setpointStatesUnoptimized =
            Constants.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(discreteSpeeds);
        currentSetpoint = swerveSetpointGenerator.generateSetpoint(currentSetpoint, discreteSpeeds,
            LoggedRobot.defaultPeriodSecs);
        SwerveModuleState[] setpointStates = currentSetpoint.moduleStates();

        Logger.recordOutput("Swerve/SwerveStates/SetpointsUnoptimized", setpointStatesUnoptimized);
        Logger.recordOutput("Swerve/SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("Swerve/SwerveChassisSpeeds/Setpoints",
            currentSetpoint.chassisSpeeds());

        SwerveModuleState[] moduleStates = getModuleStates();
        for (int i = 0; i < 4; i++) {
            Rotation2d wheelAngle = moduleStates[i].angle;
            setpointStates[i].optimize(wheelAngle);
            setpointStates[i].cosineScale(wheelAngle);

            modules[i].runSetpoint(setpointStates[i], 0.0);
        }
    }

    private void runCharacterization(double output) {
        velocityMode = false;
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

    public Command drive(Supplier<ChassisSpeeds> speedSupplier, boolean isFieldRelative) {
        return this.run(() -> {
            ChassisSpeeds speeds = speedSupplier.get();
            if (isFieldRelative) {
                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, inputs.yaw);
            }
            runVelocity(speeds);
        });
    }

    public Command wheelRadiusCharacterization() {
        SlewRateLimiter limiter = new SlewRateLimiter(0.05);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
            // Drive control
            Commands.sequence(
                // Reset acceleration
                Commands.runOnce(() -> limiter.reset(0.0)),
                // Turn in place
                this.run(() -> {
                    double speed = limiter.calculate(0.25);
                    runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                })),
            // Measurement
            Commands.sequence(
                // Wait for modules to slip while getting into turning position
                Commands.waitSeconds(1.0), Commands.runOnce(() -> {
                    state.positions = getWheelRadiusCharacterizationPositions();
                    state.lastAngle = inputs.yaw;
                    state.gyroDelta = 0.0;
                }), Commands.run(() -> {
                    var rotation = inputs.yaw;
                    state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                    state.lastAngle = rotation;

                    double[] positions = getWheelRadiusCharacterizationPositions();
                    double wheelDelta = 0.0;
                    for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                    }
                    double wheelRadius =
                        (state.gyroDelta * Constants.Swerve.DRIVE_BASE_RADIUS.in(Meters))
                            / wheelDelta;

                    Logger.recordOutput("Swerve/WheelDelta", wheelDelta);
                    Logger.recordOutput("Swerve/WheelRadius", wheelRadius);
                }).finallyDo(() -> {
                    double[] positions = getWheelRadiusCharacterizationPositions();
                    double wheelDelta = 0.0;
                    for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                    }
                    double wheelRadius =
                        (state.gyroDelta * Constants.Swerve.DRIVE_BASE_RADIUS.in(Meters))
                            / wheelDelta;

                    Logger.recordOutput("Swerve/WheelDelta", wheelDelta);
                    Logger.recordOutput("Swerve/WheelRadius", wheelRadius);

                    Constants.Swerve.TUNABLE.setWheelRadius(Meters.of(wheelRadius));
                })));
    }

    private double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].inputs.drivePositionRad;
        }
        return values;
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = Rotation2d.kZero;
        double gyroDelta = 0.0;
    }

    public Command feedforwardCharacterization() {
        List<Double> velocitySamples = new ArrayList<>();
        List<Double> voltageSamples = new ArrayList<>();
        Timer timer = new Timer();
        return Commands.sequence(Commands.runOnce(() -> {
            velocitySamples.clear();
            voltageSamples.clear();
        }), this.run(() -> this.runCharacterization(0.0)).withTimeout(2.0),
            Commands.runOnce(timer::restart), this.run(() -> {
                double voltage = timer.get() * 0.1;
                this.runCharacterization(voltage);
                velocitySamples.add(getFFCharacterizationVelocity());
                voltageSamples.add(voltage);
            }).finallyDo(() -> {
                int n = velocitySamples.size();
                double sumX = 0.0;
                double sumY = 0.0;
                double sumXY = 0.0;
                double sumX2 = 0.0;
                for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                }
                double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                Constants.Swerve.TUNABLE.setDrivekS(kS);
                Constants.Swerve.TUNABLE.setDrivekV(kV);
            }));
    }

    private double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += Units.radiansToRotations(modules[i].inputs.driveVelocityRadPerSec) / 4.0;
        }
        return output;
    }

}
