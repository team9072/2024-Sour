/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.rollers.ElevatorRollersDummyHal;
import frc.robot.subsystems.elevator.rollers.ElevatorRollersHal;
import frc.robot.subsystems.serializer.Serializer;
import frc.robot.subsystems.serializer.decisionRollers.DecisionRollersDummyHal;
import frc.robot.subsystems.serializer.decisionRollers.DecisionRollersHal;
import frc.robot.subsystems.serializer.feeder.FeederRollersDummyHal;
import frc.robot.subsystems.serializer.feeder.FeederRollersHal;
import frc.robot.subsystems.serializer.sensors.SensorDummyHal;
import frc.robot.subsystems.serializer.sensors.SensorHal;

public class Robot {
    public static final String k_canivoreCan = "canivore1";

    public final CommandXboxController m_driveController = new CommandXboxController(0);

    public final Serializer m_feeder;
    public final Elevator m_elevator;

    private Command m_autoCommand = null;

    public Robot() {
        if (RobotRunner.isReal()) {
            m_feeder = new Serializer(new FeederRollersHal(), new DecisionRollersHal(), new SensorHal());
            m_elevator = new Elevator(new ElevatorRollersHal(), new frc.robot.subsystems.elevator.sensor.SensorHal());
        } else {
            m_feeder = new Serializer(new FeederRollersDummyHal(), new DecisionRollersDummyHal(), new SensorDummyHal());
            m_elevator = new Elevator(new ElevatorRollersDummyHal(), new frc.robot.subsystems.elevator.sensor.SensorHal());
        }

        configureBindings();
    }

    public void update() {}

    private void configureBindings() {
        m_driveController.b().whileTrue(m_feeder.intake());

        m_driveController.x().onTrue(
            Commands.race(
                m_feeder.loadElevator(),
                m_elevator.loadNote()));
        
        m_driveController.a().whileTrue(m_feeder.ejectNote());
        
        m_driveController.y().onTrue(m_elevator.ejectNote());
    }

    public void updateAutoCommand() {}

    public Command getAutonomousUCommand() {
        if (m_autoCommand == null) {
            updateAutoCommand();
        }

        return m_autoCommand;
    }
}
