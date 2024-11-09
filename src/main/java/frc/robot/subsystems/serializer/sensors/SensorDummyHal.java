package frc.robot.subsystems.serializer.sensors;

public class SensorDummyHal implements SensorHalI {
    @Override
    public boolean frontHasNote() {
        return false;
    }

    @Override
    public boolean rearHasNote() {
        return false;
    }
}
