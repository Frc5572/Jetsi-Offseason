package frc.robot.subsystems.swerve;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BiFunction;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.IntStream;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroInputsAutoLogged;
import frc.robot.subsystems.swerve.mod.SwerveModule;
import frc.robot.subsystems.swerve.mod.SwerveModuleIO;
import frc.robot.subsystems.swerve.util.PhoenixOdometryThread;
import frc.robot.subsystems.swerve.util.SwerveRateLimiter;
import frc.robot.subsystems.swerve.util.SwerveState;

/** Swerve Subsystem */
public class Swerve extends SubsystemBase {

    private final Lock odometryLock = new ReentrantLock();
    private final PhoenixOdometryThread odometryThread;
    private final SwerveModule[] modules;
    private final GyroIO gyro;
    private final GyroInputsAutoLogged gyroInputs = new GyroInputsAutoLogged();
    private final SwerveIO io;
    private final SwerveInputsAutoLogged inputs = new SwerveInputsAutoLogged();

    private final SwerveRateLimiter limiter = new SwerveRateLimiter();

    public final SwerveState state;

    /** Swerve Subsystem */
    public Swerve(Function<PhoenixOdometryThread, SwerveIO> swerveIo,
        Function<PhoenixOdometryThread, GyroIO> gyroIo,
        BiFunction<Integer, PhoenixOdometryThread, SwerveModuleIO> moduleIoFn) {
        super("Swerve");
        this.odometryThread = new PhoenixOdometryThread(this.odometryLock);
        this.gyro = gyroIo.apply(this.odometryThread);
        this.modules = IntStream.range(0, Constants.Swerve.modulesConstants.length)
            .mapToObj(i -> new SwerveModule(i, moduleIoFn.apply(i, this.odometryThread)))
            .toArray(SwerveModule[]::new);
        this.io = swerveIo.apply(odometryThread);
        this.odometryThread.start();
        this.odometryLock.lock();
        SwerveModulePosition[] initPositions = new SwerveModulePosition[modules.length];
        try {
            Arrays.stream(modules).map(mod -> {
                mod.updateInputs();
                return mod.getPosition();
            }).toArray(_i -> initPositions);
        } finally {
            this.odometryLock.unlock();
        }
        this.state = new SwerveState(initPositions);
    }

    @Override
    public void periodic() {
        this.odometryLock.lock();

        for (int i = 0; i < modules.length; i++) {
            this.modules[i].updateInputs();
        }

        this.gyro.updateInputs(this.gyroInputs);
        Logger.processInputs("Swerve/Gyro", this.gyroInputs);

        this.io.updateInputs(this.inputs);
        Logger.processInputs("Swerve/Timestamps", this.inputs);

        this.odometryLock.unlock();

        for (int i = 0; i < modules.length; i++) {
            this.modules[i].periodic();
        }

        double[] sampleTimestamps = this.inputs.timestamps;
        SwerveModulePosition[] wheelPositions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < sampleTimestamps.length; i++) {
            for (int j = 0; j < modules.length; j++) {
                wheelPositions[j] = modules[j].getOdometryPosition(i);
            }
            state.addOdometryObservation(wheelPositions,
                Optional.ofNullable(
                    gyroInputs.connected ? Rotation2d.fromRadians(gyroInputs.yawRads[i]) : null),
                sampleTimestamps[i]);
        }
        SwerveModuleState[] wheelStates = new SwerveModuleState[modules.length];
        for (int j = 0; j < modules.length; j++) {
            wheelStates[j] = modules[j].getState();
        }
        ChassisSpeeds currentSpeeds =
            Constants.Swerve.swerveKinematics.toChassisSpeeds(wheelStates);
        limiter.update(currentSpeeds);

        Logger.recordOutput("Swerve/GlobalPoseEstimate", state.getGlobalPoseEstimate());
    }

    /*
     *
     * Commands
     *
     */

    /** Drive using local robot coordinate frame */
    public Command driveRobotRelative(Supplier<ChassisSpeeds> driveSpeeds) {
        return this.run(() -> {
            ChassisSpeeds speeds = driveSpeeds.get();
            speeds = limiter.limit(speeds);
            setModuleStates(speeds);
        });
    }

    /** Use user-defined field heading for relative heading */
    public Command driveUserRelative(Supplier<ChassisSpeeds> driveSpeeds) {
        return driveRobotRelative(() -> ChassisSpeeds.fromFieldRelativeSpeeds(driveSpeeds.get(),
            getFieldRelativeHeading()));
    }

    /** Use state field heading for relative heading */
    public Command driveFieldRelative(Supplier<ChassisSpeeds> driveSpeeds) {
        return driveRobotRelative(() -> ChassisSpeeds.fromFieldRelativeSpeeds(driveSpeeds.get(),
            state.getGlobalPoseEstimate().getRotation()));
    }

    private static final double ffStartDelay = 2.0;
    private static final double ffRampRate = 0.1;

    /** Sysid routine to determine kS and kV */
    public Command feedforwardCharacterization() {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
            // Reset data
            Commands.runOnce(() -> {
                velocitySamples.clear();
                voltageSamples.clear();
            }),

            // Allow modules to orient
            Commands.run(() -> this.runCharacterization(0.0), this).withTimeout(ffStartDelay),

            // Start timer
            Commands.runOnce(timer::restart),

            // Accelerate and gather data
            Commands.run(() -> {
                double voltage = timer.get() * ffRampRate;
                this.runCharacterization(voltage);
                velocitySamples.add(this.getFFCharacterizationVelocity());
                voltageSamples.add(voltage);
            }, this)

                // When cancelled, calculate and print results
                .finallyDo(() -> {
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

                    NumberFormat formatter = new DecimalFormat("#0.00000");
                    System.out.println("********** Drive FF Characterization Results **********");
                    System.out.println("\tkS: " + formatter.format(kS));
                    System.out.println("\tkV: " + formatter.format(kV));
                    Logger.recordOutput("Sysid/FF/kS", kS);
                    Logger.recordOutput("Sysid/FF/kV", kV);
                }));
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = Rotation2d.kZero;
        double gyroDelta = 0.0;
    }

    private static final double wheelRadiusMaxVelocity = 0.25;
    private static final double wheelRadiusRampRate = 0.05;

    /** Sysid routine to determine wheel radius */
    public Command wheelRadiusCharacterization() {
        SlewRateLimiter limiter = new SlewRateLimiter(wheelRadiusRampRate);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        final double driveBaseRadius =
            Math.hypot(Constants.Swerve.trackWidth / 2.0, Constants.Swerve.wheelBase / 2.0);

        return Commands.parallel(
            // Drive control sequence
            Commands.sequence(
                // Reset acceleration limiter
                Commands.runOnce(() -> limiter.reset(0.0)),

                // Turn in place, accelerating up to full speed
                Commands.run(() -> {
                    double speed = limiter.calculate(wheelRadiusMaxVelocity);
                    this.setModuleStates(new ChassisSpeeds(0.0, 0.0, speed));
                }, this)),

            // Measurement sequence
            Commands.sequence(
                // Wait for modules to fully orient before starting measurement
                Commands.waitSeconds(1.0),

                // Record starting measurement
                Commands.runOnce(() -> {
                    state.positions = this.getWheelRadiusCharacterizationPositions();
                    state.lastAngle = this.gyroInputs.yaw;
                    state.gyroDelta = 0.0;
                }),

                // Update gyro delta
                Commands.run(() -> {
                    var rotation = this.gyroInputs.yaw;
                    state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                    state.lastAngle = rotation;

                    double[] positions = this.getWheelRadiusCharacterizationPositions();
                    double wheelDelta = 0.0;
                    for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                    }
                    double wheelRadius = (state.gyroDelta * driveBaseRadius) / wheelDelta;

                    Logger.recordOutput("Drive/WheelDelta", wheelDelta);
                    Logger.recordOutput("Drive/WheelRadius", wheelRadius);
                })

                    // When cancelled, calculate and print results
                    .finallyDo(() -> {
                        double[] positions = this.getWheelRadiusCharacterizationPositions();
                        double wheelDelta = 0.0;
                        for (int i = 0; i < 4; i++) {
                            wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                        }
                        double wheelRadius = (state.gyroDelta * driveBaseRadius) / wheelDelta;

                        NumberFormat formatter =
                            new DecimalFormat("#0.000000000000000000000000000");
                        System.out
                            .println("********** Wheel Radius Characterization Results **********");
                        System.out
                            .println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                        System.out.println(
                            "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                        System.out.println(
                            "\tWheel Radius: " + formatter.format(wheelRadius) + " meters, "
                                + formatter.format(Units.metersToInches(wheelRadius)) + " inches");
                        Logger.recordOutput("Sysid/FF/radius", wheelRadius);
                    })));
    }

    /** Set heading for driving */
    public Command setFieldRelativeOffset(Supplier<Rotation2d> knownHeading) {
        return Commands.runOnce(() -> {
            fieldOffset = gyroInputs.yaw.getRotations() - knownHeading.get().getRotations();
        });
    }

    /** Set heading for driving to zero */
    public Command setFieldRelativeOffset() {
        return setFieldRelativeOffset(() -> Rotation2d.kZero);
    }

    /** Set heading for driving to the current global estimated pose */
    public Command resetFieldRelativeOffsetBasedOnPose() {
        return setFieldRelativeOffset(() -> {
            return state.getGlobalPoseEstimate().getRotation()
                .plus(shouldFlipPath() ? Rotation2d.kZero : Rotation2d.k180deg);
        });
    }

    /*
     *
     * Utility functions
     *
     */

    private void runCharacterization(double output) {
        for (int i = 0; i < modules.length; i++) {
            modules[i].runCharacterization(output);
        }
    }

    private double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < modules.length; i++) {
            output += modules[i].getFFCharacterizationVelocity() / modules.length;
        }
        return output;
    }

    private double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[modules.length];
        for (int i = 0; i < modules.length; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    private double fieldOffset = 0.0;

    /** State-independent heading for driving */
    public Rotation2d getFieldRelativeHeading() {
        return Rotation2d.fromRotations(gyroInputs.yaw.getRotations() - fieldOffset);
    }

    private void setModuleStates(ChassisSpeeds chassisSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(swerveModuleStates);
    }

    private void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(desiredStates[i]);
        }
    }

    /**
     * Determine whether or not to flight the auto path
     *
     * @return True if flip path to Red Alliance, False if Blue
     */
    private static boolean shouldFlipPath() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            return ally.get() == Alliance.Red;
        }
        return false;
    }
}
