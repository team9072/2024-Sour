package frc.robot.subsystems.serializer.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class SensorHal implements SensorHalI {
    private static final int k_frontDioId = 1;
    private final DigitalInput m_frontBreak;

    private static final int k_rearDioId = 2;
    private final DigitalInput m_rearBreak;

    public SensorHal() {
        m_frontBreak = new DigitalInput(k_frontDioId);
        m_rearBreak = new DigitalInput(k_rearDioId);
    }

    @Override
    public boolean frontHasNote() {
        return !m_frontBreak.get();
    }

    @Override
    public boolean rearHasNote() {
        return !m_rearBreak.get();
    }
}
