package frc.robot.util.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import lombok.Builder;
import lombok.Data;

@Builder
@Data
public class ModuleConfig {

    public final int driveMotorId;
    public final int angleMotorId;
    public final int canCoderId;
    public final Rotation2d angleOffset;

}
