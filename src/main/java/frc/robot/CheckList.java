package frc.robot;

import java.util.EnumSet;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Semi-automated prematch checklist */
public class CheckList {

    /** Set up hooks */
    public static void setup_hooks() {
        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        instance.addListener(instance.getBooleanTopic("/checklist/enable").subscribe(false),
            EnumSet.of(NetworkTableEvent.Kind.kValueAll), (data) -> {
                boolean enabled = data.valueData.value.getBoolean();
                if (enabled) {
                    // TODO enable
                }
            });
    }



}
