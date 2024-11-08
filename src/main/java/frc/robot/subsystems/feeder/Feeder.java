package frc.robot.subsystems.feeder;

import org.growingstems.measurements.Measurements.Voltage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.feeder.sensors.SensorHal;

public class Feeder extends SubsystemBase{
    private final FeederRollersHal m_rollers;
    private final SensorHal m_sensors;

    public Feeder(FeederRollersHal rollers, SensorHal sensors) {
        m_rollers = rollers;
        m_sensors = sensors;
    }

    private void stop() {
        m_rollers.brake();
        m_rollers.brake();
    }

    private void startIntake() {
        m_rollers.setPower(new Voltage(4));
    }

    public Command intake() {
        return Commands.sequence(
            run(this::startIntake),
            Commands.waitUntil(m_sensors::getRearBreak)
        ).finallyDo(this::stop);
    }
}
