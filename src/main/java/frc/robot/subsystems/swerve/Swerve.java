package frc.robot.subsystems.swerve;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BiFunction;
import java.util.stream.IntStream;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroInputsAutoLogged;
import frc.robot.subsystems.swerve.mod.SwerveModule;
import frc.robot.subsystems.swerve.mod.SwerveModuleIO;

/**
 * Swerve Subsystem
 */
public class Swerve extends SubsystemBase {

    private final Lock odometryLock = new ReentrantLock();
    private final PhoenixOdometryThread odometryThread;
    private final SwerveModule[] modules;
    private final GyroIO gyro;
    private final GyroInputsAutoLogged gyroInputs = new GyroInputsAutoLogged();

    public Swerve(GyroIO gyroIo,
        BiFunction<Integer, PhoenixOdometryThread, SwerveModuleIO> moduleIoFn) {
        super("Swerve");
        this.gyro = gyroIo;
        this.odometryThread = new PhoenixOdometryThread(odometryLock);
        this.modules = IntStream
            .range(0, Constants.Swerve.modulesConstants.length).mapToObj(i -> new SwerveModule(i,
                Constants.Swerve.modulesConstants[i], moduleIoFn.apply(i, this.odometryThread)))
            .toArray(SwerveModule[]::new);
    }

    @Override
    public void periodic() {
        odometryLock.lock();
        for (int i = 0; i < modules.length; i++) {
            modules[i].updateInputs();
        }
        this.gyro.updateInputs(this.gyroInputs);
        Logger.processInputs("Swerve/Gyro", this.gyroInputs);
        odometryLock.unlock();

        for (int i = 0; i < modules.length; i++) {
            modules[i].periodic();
        }
    }
}
