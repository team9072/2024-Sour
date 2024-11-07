package frc.robot.subsystems.elevator;

import frc.library.StartupNeutralMode;
import org.growingstems.measurements.Measurements.Voltage;

public interface ElevatorRollersHalI extends StartupNeutralMode {
    void brake();

    void coast();

    void setPower(Voltage power);

    void update();
}
