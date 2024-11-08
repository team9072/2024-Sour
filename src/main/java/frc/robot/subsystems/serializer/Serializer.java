package frc.robot.subsystems.serializer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.growingstems.measurements.Measurements.Voltage;

public class Serializer extends SubsystemBase {
    private final SerializerRollersHalI m_rollers;

    public Serializer(SerializerRollersHalI rollers) {
        m_rollers = rollers;
    }

    @Override
    public void periodic() {
        m_rollers.update();
    }

    private void _stop() {
        m_rollers.brake();
    }

    private void startForward() {
        m_rollers.setPower(new Voltage(8));
    }

    private void startReverse() {
        m_rollers.setPower(new Voltage(-8));
    }

    public Command insertToShooter() {
        return startEnd(this::startForward, this::_stop);
    }

    public Command reverseToAmp() {
        return startEnd(this::startReverse, this::_stop);
    }
}
