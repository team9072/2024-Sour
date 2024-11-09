package frc.robot.subsystems.serializer.feeder;

import frc.library.StartupNeutralMode;
import org.growingstems.measurements.Measurements.Voltage;

public interface FeederRollersHalI extends StartupNeutralMode {
    void brake();

    void coast();

    void setPower(Voltage power);

    void update();
}
