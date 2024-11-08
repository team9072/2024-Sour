package frc.robot.subsystems.serializer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.growingstems.measurements.Measurements.Voltage;

public class Serializer extends SubsystemBase {
    private final SerializerRollersHal m_rollers;

    public Serializer(SerializerRollersHal rollers) {
        m_rollers = rollers;
    }

    private void _stop() {
        m_rollers.brake();
    }

    private void startForward() {
        m_rollers.setPower(new Voltage(4));
    }

    private void startReverse() {
        m_rollers.setPower(new Voltage(-4));
    }

    public Command insertToShooter() {
        return startEnd(this::startForward, this::_stop);
    }

    public Command reverseToAmp() {
        return startEnd(this::startReverse, this::_stop);
    }
}
