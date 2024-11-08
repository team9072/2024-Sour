package frc.robot.subsystems.serializer.sensors;

public class SensorDummyHal implements SensorHalI {
	@Override
	public boolean getFrontBreak() {
        return false;
	}

	@Override
	public boolean getRearBreak() {
        return false;
	}
}
