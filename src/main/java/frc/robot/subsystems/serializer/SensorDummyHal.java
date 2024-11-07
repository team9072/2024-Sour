package frc.robot.subsystems.serializer;

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
