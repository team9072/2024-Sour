package frc.robot.subsystems.elevator;

import org.growingstems.measurements.Measurements.Voltage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.elevator.rollers.ElevatorRollersHalI;
import frc.robot.subsystems.elevator.sensor.SensorHalI;

public class Elevator extends SubsystemBase {
    private final ElevatorRollersHalI m_rollers;
    private final Trigger m_sensor;

    public Elevator(ElevatorRollersHalI rollers, SensorHalI sensor) {
        m_rollers = rollers;
        m_sensor = new Trigger(sensor::hasNote);
    }

    private void stopRollers() {
        m_rollers.brake();
    }

    private void startRollersForward() {
        m_rollers.setPower(new Voltage(12));
    }

    public Trigger getSensor() {
        return m_sensor;
    }

    public Command loadNote() {
        return startEnd(this::startRollersForward, this::stopRollers)
        .raceWith(Commands.waitUntil(m_sensor).andThen(Commands.waitSeconds(0.1)));
    }

    public Command ejectNote() {
        return startEnd(this::startRollersForward, this::stopRollers)
            .raceWith(Commands.waitUntil(m_sensor.negate()).andThen(Commands.waitSeconds(0.2)));
    }
}
