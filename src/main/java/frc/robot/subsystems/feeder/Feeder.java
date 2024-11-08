package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.feeder.sensors.SensorHalI;
import org.growingstems.measurements.Measurements.Voltage;

public class Feeder extends SubsystemBase {
    private final FeederRollersHalI m_rollers;
    private final SensorHalI m_sensors;

    private final Trigger m_frontSensor;
    private final Trigger m_rearSensor;

    public Feeder(FeederRollersHalI rollers, SensorHalI sensor) {
        m_rollers = rollers;
        m_sensors = sensor;
        m_frontSensor = new Trigger(m_sensors::frontHasNote);
        m_rearSensor = new Trigger(m_sensors::rearHasNote);
    }

    @Override
    public void periodic() {
        m_rollers.update();
    }

    private void _stop() {
        m_rollers.brake();
        m_rollers.brake();
    }

    private void startIntake() {
        m_rollers.setPower(new Voltage(4));
    }

    private void startReverse() {
        m_rollers.setPower(new Voltage(-4));
    }

    public Trigger getFrontSensor() {
        return m_frontSensor;
    }

    public Trigger getRearSensor() {
        return m_rearSensor;
    }

    public Command stop() {
        return runOnce(this::_stop);
    }

    public Command intake() {
        return startEnd(this::startIntake, this::_stop);
    }

    public Command reverse() {
        return startEnd(this::startReverse, this::_stop);
    }
}
