package frc.robot.subsystems.swerve.module;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.util.PhoenixUtil.tryUntilOk;
import java.util.Queue;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.swerve.ModuleConfig;
import frc.robot.util.swerve.PhoenixOdometryThread;

public class ModuleReal implements ModuleIO {

    private final TalonFX angleMotor;
    private final TalonFX driveMotor;
    private final CANcoder angleEncoder;
    private final TalonFXConfiguration angleConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    private final CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

    private final Rotation2d angleOffset;

    /* Drive Signals */
    private final StatusSignal<Angle> drivePosition;
    private final Queue<Double> drivePositionQueue;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Voltage> driveAppliedVolts;

    /* Angle Signals */
    private final StatusSignal<Angle> anglePosition;
    private final Queue<Double> anglePositionQueue;
    private final StatusSignal<AngularVelocity> angleVelocity;
    private final StatusSignal<Voltage> angleAppliedVolts;
    private final StatusSignal<Angle> angleAbsolutePosition;

    public ModuleReal(int index, PhoenixOdometryThread odometryThread) {
        ModuleConfig config = Constants.Swerve.MODULE_CONFIGS[index];
        this.angleOffset = config.angleOffset;

        this.angleEncoder = new CANcoder(config.canCoderId, "canivore");
        this.driveMotor = new TalonFX(config.driveMotorId, "canivore");
        this.angleMotor = new TalonFX(config.angleMotorId, "canivore");

        configAngleEncoder();
        configAngleMotor();
        configDriveMotor();

        drivePosition = driveMotor.getPosition();
        drivePositionQueue = odometryThread.registerSignal(driveMotor.getPosition().clone());
        driveVelocity = driveMotor.getVelocity();
        driveAppliedVolts = driveMotor.getMotorVoltage();

        anglePosition = angleMotor.getPosition();
        anglePositionQueue = odometryThread.registerSignal(angleMotor.getPosition().clone());
        angleVelocity = angleMotor.getVelocity();
        angleAppliedVolts = angleMotor.getMotorVoltage();

        angleAbsolutePosition = angleEncoder.getAbsolutePosition();

        BaseStatusSignal.setUpdateFrequencyForAll(Constants.Swerve.ODOMETRY_FREQUENCY,
            drivePosition, anglePosition, angleAbsolutePosition);
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, driveVelocity, driveAppliedVolts,
            angleVelocity, angleAppliedVolts);
        tryUntilOk(5,
            () -> ParentDevice.optimizeBusUtilizationForAll(driveMotor, angleMotor, angleEncoder));

        PhoenixUtil.registerSignals(true, drivePosition, driveVelocity, driveAppliedVolts,
            anglePosition, angleVelocity, angleAbsolutePosition, angleAppliedVolts);
    }

    private void configAngleMotor() {
        /* Angle Motor Config */
        /* Motor Inverts and Neutral Mode */
        angleConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        angleConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        /* Gear Ratio and Wrapping Config */
        angleConfig.Feedback.FeedbackRemoteSensorID = angleEncoder.getDeviceID();
        angleConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        angleConfig.Feedback.SensorToMechanismRatio = 1.0;
        angleConfig.Feedback.RotorToSensorRatio = Constants.Swerve.ANGLE_REDUCTION;
        angleConfig.ClosedLoopGeneral.ContinuousWrap = true;

        /* Current Limiting */
        angleConfig.CurrentLimits.SupplyCurrentLimitEnable =
            Constants.Swerve.CurrentLimits.ANGLE_ENABLE_CURRENT_LIMIT;
        angleConfig.CurrentLimits.SupplyCurrentLimit =
            Constants.Swerve.CurrentLimits.ANGLE_CURRENT_LIMIT.in(Amps);
        angleConfig.CurrentLimits.SupplyCurrentLowerLimit =
            Constants.Swerve.CurrentLimits.ANGLE_CURRENT_THRESHOLD.in(Amps);
        angleConfig.CurrentLimits.SupplyCurrentLowerTime =
            Constants.Swerve.CurrentLimits.ANGLE_CURRENT_THRESHOLD_TIME.in(Seconds);

        /* PID Config */
        angleConfig.Slot0.kP = Constants.Swerve.TUNABLE.getAnglekP();
        angleConfig.Slot0.kI = 0.0;
        angleConfig.Slot0.kD = Constants.Swerve.TUNABLE.getAnglekD();

        tryUntilOk(5, () -> angleMotor.getConfigurator().apply(angleConfig));
    }

    private void configDriveMotor() {
        /* Drive Motor Config */
        /* Motor Inverts and Neutral Mode */
        driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        /* Gear Ratio Config */
        driveConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.DRIVE_REDUCTION;

        /* Current Limiting */
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable =
            Constants.Swerve.CurrentLimits.DRIVE_ENABLE_CURRENT_LIMIT;
        driveConfig.CurrentLimits.SupplyCurrentLimit =
            Constants.Swerve.CurrentLimits.DRIVE_CURRENT_LIMIT.in(Amps);
        driveConfig.CurrentLimits.SupplyCurrentLowerLimit =
            Constants.Swerve.CurrentLimits.DRIVE_CURRENT_LOWER_LIMIT.in(Amps);
        driveConfig.CurrentLimits.SupplyCurrentLowerTime =
            Constants.Swerve.CurrentLimits.DRIVE_CURRENT_LOWER_TIME_THRESHOLD.in(Seconds);

        /* PID Config */
        driveConfig.Slot0.kP = Constants.Swerve.TUNABLE.getDrivekP();
        driveConfig.Slot0.kI = 0.0;
        driveConfig.Slot0.kD = Constants.Swerve.TUNABLE.getDrivekD();
        driveConfig.Slot0.kS = Constants.Swerve.TUNABLE.getDrivekS();
        driveConfig.Slot0.kV = Constants.Swerve.TUNABLE.getDrivekV();
        driveConfig.Slot0.kA = Constants.Swerve.TUNABLE.getDrivekA();

        /* Open and Closed Loop Ramping */
        driveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.OPEN_LOOP_RAMP;
        driveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.OPEN_LOOP_RAMP;

        driveConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod =
            Constants.Swerve.CLOSED_LOOP_RAMP;
        driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Swerve.CLOSED_LOOP_RAMP;

        tryUntilOk(5, () -> driveMotor.getConfigurator().apply(driveConfig));
        tryUntilOk(5, () -> driveMotor.getConfigurator().setPosition(0.0));
    }

    private void configAngleEncoder() {
        /* Angle Encoder Config */
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        encoderConfig.MagnetSensor.MagnetOffset = -angleOffset.getRotations();

        tryUntilOk(5, () -> angleEncoder.getConfigurator().apply(encoderConfig));
    }

    @Override
    public void updateInputs(ModuleInputs inputs) {

        inputs.driveConnected =
            BaseStatusSignal.isAllGood(drivePosition, driveVelocity, driveAppliedVolts);
        inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
        inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
        inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();

        inputs.angleConnected =
            BaseStatusSignal.isAllGood(anglePosition, angleVelocity, angleAppliedVolts);
        inputs.anglePosition = Rotation2d.fromRotations(anglePosition.getValueAsDouble());
        inputs.angleVelocityRadsPerSec = Units.rotationsToRadians(angleVelocity.getValueAsDouble());
        inputs.angleAppliedVolts = angleAppliedVolts.getValueAsDouble();

        inputs.absoluteAngleConnected = BaseStatusSignal.isAllGood(angleAbsolutePosition);
        inputs.angleAbsolutePosition =
            Rotation2d.fromRotations(angleAbsolutePosition.getValueAsDouble()).minus(angleOffset);

        for (int i = 0; i < Constants.Swerve.MAX_ODOMETRY_SUBTICK_MEASUREMENTS; i++) {
            if (drivePositionQueue.isEmpty()) {
                break;
            }
            inputs.odometryDrivePositionsRad[i] = drivePositionQueue.poll();
            inputs.odometryTurnPositions[i] = Rotation2d.fromRotations(anglePositionQueue.poll());
        }
        drivePositionQueue.clear();
        anglePositionQueue.clear();
    }

    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
    private final PositionVoltage positionRequest = new PositionVoltage(0.0);

    @Override
    public void driveOpenLoop(double output) {
        driveMotor.setControl(voltageRequest.withOutput(output));
    }

    @Override
    public void turnOpenLoop(double output) {
        angleMotor.setControl(voltageRequest.withOutput(output));
    }

    @Override
    public void driveVelocity(double radPerSec, double ff) {
        driveMotor.setControl(
            velocityRequest.withVelocity(Units.radiansToRotations(radPerSec)).withFeedForward(ff));
    }

    @Override
    public void turnPosition(Rotation2d heading) {
        angleMotor.setControl(positionRequest.withPosition(heading.getRotations()));
    }

    @Override
    public void setDrivePID(double kP, double kI, double kD, double kS, double kV, double kA) {
        driveConfig.Slot0.kP = kP;
        driveConfig.Slot0.kI = kI;
        driveConfig.Slot0.kD = kD;
        driveConfig.Slot0.kS = kS;
        driveConfig.Slot0.kV = kV;
        driveConfig.Slot0.kA = kA;
        tryUntilOk(5, () -> driveMotor.getConfigurator().apply(driveConfig, 0.25));
    }

    @Override
    public void setAnglePID(double kP, double kI, double kD) {
        angleConfig.Slot0.kP = kP;
        angleConfig.Slot0.kI = kI;
        angleConfig.Slot0.kD = kD;
        tryUntilOk(5, () -> angleMotor.getConfigurator().apply(angleConfig, 0.25));
    }

    private static final Executor brakeModeExecutor = Executors.newFixedThreadPool(8);

    @Override
    public void setBrakeMode(boolean enabled) {
        brakeModeExecutor.execute(() -> {
            synchronized (driveConfig) {
                driveConfig.MotorOutput.NeutralMode =
                    enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
                tryUntilOk(5, () -> driveMotor.getConfigurator().apply(driveConfig, 0.25));
            }
        });
        brakeModeExecutor.execute(() -> {
            synchronized (angleConfig) {
                angleConfig.MotorOutput.NeutralMode =
                    enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
                tryUntilOk(5, () -> angleMotor.getConfigurator().apply(angleConfig, 0.25));
            }
        });
    }



}
