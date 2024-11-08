package frc.robot.subsystems.elevator;

import org.growingstems.measurements.Measurements.Voltage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.rollers.ElevatorRollersHalI;
import frc.robot.subsystems.elevator.sensor.SensorHalI;

public class Elevator extends SubsystemBase {
    private ElevatorRollersHalI m_rollers;
    private SensorHalI m_sensor;

    Elevator(ElevatorRollersHalI rollers, SensorHalI sensor) {
        m_rollers = rollers;
        m_sensor = sensor;
    }

    private void stopRollers() {
        m_rollers.brake();
    }

    private void startRollersForward() {
        m_rollers.setPower(new Voltage(4));
    }

    public Command loadNote() {
        return startEnd(this::startRollersForward, this::stopRollers).until(m_sensor::hasNote);
    }

    public Command ejectNote() {
        return startEnd(this::startRollersForward, this::stopRollers)
            .raceWith(Commands.waitUntil(m_sensor::hasNote).andThen(Commands.waitSeconds(0.2)));
    }
}
