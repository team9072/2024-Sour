package frc.robot.subsystems.serializer.decisionRollers;

import frc.library.StartupNeutralMode;
import org.growingstems.measurements.Measurements.Voltage;

public interface DecisionRollersHalI extends StartupNeutralMode {
    void brake();

    void coast();

    void setPower(Voltage power);

    void update();
}
