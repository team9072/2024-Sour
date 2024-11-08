package frc.robot.subsystems.elevator.sensor;

import edu.wpi.first.wpilibj.DigitalInput;

public class SensorHal implements SensorHalI {
    private static final int k_frontDioId = 0;
    private final DigitalInput m_frontBreak;

    public SensorHal() {
        m_frontBreak = new DigitalInput(k_frontDioId);
    }

    public boolean hasNote() {
        return m_frontBreak.get();
    }
}