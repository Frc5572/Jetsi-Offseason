package frc.robot.subsystems.swerve.util;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.junction.Logger;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public final class TuningCommands {

    private TuningCommands() {}

    private static final double ffStartDelay = 2.0;
    private static final double ffRampRate = 0.1;
    private static final double wheelRadiusMaxVelocity = 0.25;
    private static final double wheelRadiusRampRate = 0.05;

    public static Command feedforwardCharacterization(Swerve swerve, DoubleConsumer runCharacterization, DoubleSupplier getFFCharacterizationVelocity) {
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
            Commands.run(() -> runCharacterization.accept(0.0), swerve).withTimeout(ffStartDelay),

            // Start timer
            Commands.runOnce(timer::restart),

            // Accelerate and gather data
            Commands.run(() -> {
                    double voltage = timer.get() * ffRampRate;
                    runCharacterization.accept(voltage);
                    velocitySamples.add(getFFCharacterizationVelocity.getAsDouble());
                    voltageSamples.add(voltage);
                }, swerve)

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

    public static Command wheelRadiusCharacterization(Swerve swerve, Consumer<ChassisSpeeds> setModuleStates,
                                                      Supplier<double[]> getWheelRadiusCharacterizationPositions,
                                                      Supplier<Rotation2d> gyroYaw) {
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
                    setModuleStates.accept(new ChassisSpeeds(0.0, 0.0, speed));
                }, swerve)),

            // Measurement sequence
            Commands.sequence(
                // Wait for modules to fully orient before starting measurement
                Commands.waitSeconds(1.0),

                // Record starting measurement
                Commands.runOnce(() -> {
                    state.positions = getWheelRadiusCharacterizationPositions.get();
                    state.lastAngle = gyroYaw.get();
                    state.gyroDelta = 0.0;
                }),

                // Update gyro delta
                Commands.run(() -> {
                        var rotation = gyroYaw.get();
                        state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                        state.lastAngle = rotation;

                        double[] positions = getWheelRadiusCharacterizationPositions.get();
                        double wheelDelta = 0.0;
                        for (int i = 0; i < 4; i++) {
                            wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                        }
                        double wheelRadius = (state.gyroDelta * driveBaseRadius) / wheelDelta;

                        Logger.recordOutput("Drive/WheelDelta", wheelDelta);
                        Logger.recordOutput("Drive/WheelRadius", wheelRadius);
                    })));
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = Rotation2d.kZero;
        double gyroDelta = 0.0;
    }
}
