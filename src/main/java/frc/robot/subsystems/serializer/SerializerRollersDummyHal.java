package frc.robot.subsystems.serializer;

import org.growingstems.measurements.Measurements.Voltage;

public class SerializerRollersDummyHal implements SerializerRollersHalI {
    @Override
    public void setPower(Voltage power) {
        // NOP
    }

    @Override
    public void setIntendedNeutralMode() {
        // NOP
    }

    @Override
    public void update() {
        // NOP
    }

    @Override
    public void brake() {
        // NOP
    }

    @Override
    public void coast() {
        // NOP
    }
}
