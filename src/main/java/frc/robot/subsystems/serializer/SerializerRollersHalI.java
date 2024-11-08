package frc.robot.subsystems.serializer;

import frc.library.StartupNeutralMode;
import org.growingstems.measurements.Measurements.Voltage;

public interface SerializerRollersHalI extends StartupNeutralMode {
    void brake();

    void coast();

    void setPower(Voltage power);

    void update();
}
