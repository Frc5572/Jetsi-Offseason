package frc.robot.subsystems.swerve;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BiFunction;
import java.util.function.Function;
import java.util.stream.IntStream;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroInputsAutoLogged;
import frc.robot.subsystems.swerve.mod.SwerveModule;
import frc.robot.subsystems.swerve.mod.SwerveModuleIO;
import frc.robot.subsystems.swerve.util.PhoenixOdometryThread;
import frc.robot.subsystems.swerve.util.SwerveRateLimiter;
import frc.robot.subsystems.swerve.util.SwerveState;

/**
 * Swerve Subsystem
 */
public class Swerve extends SubsystemBase {

    private final Lock odometryLock = new ReentrantLock();
    private final PhoenixOdometryThread odometryThread;
    private final SwerveModule[] modules;
    private final GyroIO gyro;
    private final GyroInputsAutoLogged gyroInputs = new GyroInputsAutoLogged();

    private final SwerveRateLimiter limiter = new SwerveRateLimiter();

    public final SwerveState state;

    public Swerve(Function<PhoenixOdometryThread, GyroIO> gyroIo,
        BiFunction<Integer, PhoenixOdometryThread, SwerveModuleIO> moduleIoFn) {
        super("Swerve");
        this.odometryThread = new PhoenixOdometryThread(this.odometryLock);
        this.gyro = gyroIo.apply(this.odometryThread);
        this.modules = IntStream.range(0, Constants.Swerve.modulesConstants.length)
            .mapToObj(i -> new SwerveModule(i, moduleIoFn.apply(i, this.odometryThread)))
            .toArray(SwerveModule[]::new);
        this.odometryThread.start();
        this.state = new SwerveState();
    }

    @Override
    public void periodic() {
        this.odometryLock.lock();
        for (int i = 0; i < modules.length; i++) {
            this.modules[i].updateInputs();
        }
        this.gyro.updateInputs(this.gyroInputs);
        Logger.processInputs("Swerve/Gyro", this.gyroInputs);
        this.odometryLock.unlock();

        for (int i = 0; i < modules.length; i++) {
            this.modules[i].periodic();
        }
    }
}
