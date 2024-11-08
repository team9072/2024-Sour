package frc.robot.subsystems.feeder;

import org.growingstems.measurements.Measurements.Voltage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.feeder.sensors.SensorHal;

public class Feeder extends SubsystemBase{
    private final FeederRollersHal m_rollers;
    private final SensorHal m_sensor;

    public Feeder(FeederRollersHal rollers, SensorHal sensor) {
        m_rollers = rollers;
        m_sensor = sensor;
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

    public boolean getFrontSensor() {
        return m_sensor.frontHasNote();
    }

    public boolean getRearSensor() {
        return m_sensor.rearHasNote();
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
