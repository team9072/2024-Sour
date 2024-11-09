package frc.robot.subsystems.serializer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.serializer.decisionRollers.DecisionRollersHalI;
import frc.robot.subsystems.serializer.feeder.FeederRollersHalI;
import frc.robot.subsystems.serializer.sensors.SensorHalI;

import org.growingstems.measurements.Measurements.Voltage;

public class Serializer extends SubsystemBase {
    private final FeederRollersHalI m_intakeRollers;
    private final DecisionRollersHalI m_decisionRollers;

    private final Trigger m_frontSensor;
    private final Trigger m_rearSensor;

    public Serializer(FeederRollersHalI intakeRollers, DecisionRollersHalI decisionRollers, SensorHalI sensors) {
        m_intakeRollers = intakeRollers;
        m_decisionRollers = decisionRollers;
        m_frontSensor = new Trigger(sensors::frontHasNote);
        m_rearSensor = new Trigger(sensors::rearHasNote);
    }

    @Override
    public void periodic() {
        m_intakeRollers.update();
    }

    private void stopRollers() {
        m_intakeRollers.brake();
        m_decisionRollers.brake();
    }

    private void startIntake() {
        m_intakeRollers.setPower(new Voltage(12));
        m_decisionRollers.brake();
    }

    private void startReverse() {
        m_intakeRollers.setPower(new Voltage(-12));
        m_decisionRollers.setPower(new Voltage(-12));
    }

    private void startFeedShooter() {
        m_intakeRollers.setPower(new Voltage(8));
        m_decisionRollers.setPower(new Voltage(-8));
    }

    private void startFeedElevator() {
        m_intakeRollers.setPower(new Voltage(8));
        m_decisionRollers.setPower(new Voltage(-8));
    }

    public Trigger getFrontSensor() {
        return m_frontSensor;
    }

    public Trigger getRearSensor() {
        return m_rearSensor;
    }

    public Command stop() {
        return runOnce(this::stopRollers);
    }

    public Command intake() {
        return startEnd(this::startIntake, this::stopRollers).until(m_rearSensor);
    }

    public Command reverse() {
        return startEnd(this::startReverse, this::stopRollers);
    }

    public Command ejectNote() {
        return reverse().until(m_frontSensor.negate());
    }

    public Command feedShooter() {
        return startEnd(this::startFeedShooter, this::stopRollers).until(m_rearSensor.negate());
    }

    public Command loadElevator() {
        return reverse().until(m_rearSensor.negate())
            .andThen(run(this::startFeedElevator))
            .finallyDo(this::stopRollers);
    }
}
