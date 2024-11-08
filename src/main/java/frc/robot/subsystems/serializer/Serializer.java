package frc.robot.subsystems.serializer;

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
}
