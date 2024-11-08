package frc.robot.subsystems.serializer;

import org.growingstems.measurements.Measurements.Voltage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.serializer.decisionrollers.DecisionRollersHalI;
import frc.robot.subsystems.serializer.feeder.FeederRollersHalI;
import frc.robot.subsystems.serializer.sensors.SensorHalI;

public class Serializer extends SubsystemBase{
    private final DecisionRollersHalI m_decisionRollers;
    private final FeederRollersHalI m_feederRollers;
    private final SensorHalI m_sensors;

    public Serializer(DecisionRollersHalI decisionRollers, FeederRollersHalI feederRollers, SensorHalI sensors) {
        m_decisionRollers = decisionRollers;
        m_feederRollers = feederRollers;
        m_sensors = sensors;
    }

    private void stop() {
        m_decisionRollers.brake();
        m_feederRollers.brake();
    }

    private void startIntake() {
        m_feederRollers.setPower(new Voltage(4));
    }

    public Command intake() {
        return Commands.sequence(
            run(this::startIntake),
            Commands.waitUntil(m_sensors::getRearBreak)
        ).finallyDo(this::stop);
    }
}
