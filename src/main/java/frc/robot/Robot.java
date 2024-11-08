/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.logging.LogBuilder;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.rollers.ElevatorRollersDummyHal;
import frc.robot.subsystems.elevator.rollers.ElevatorRollersHal;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederRollersDummyHal;
import frc.robot.subsystems.feeder.FeederRollersHal;
import frc.robot.subsystems.feeder.sensors.SensorDummyHal;
import frc.robot.subsystems.feeder.sensors.SensorHal;
import frc.robot.subsystems.serializer.Serializer;
import frc.robot.subsystems.serializer.SerializerRollersDummyHal;
import frc.robot.subsystems.serializer.SerializerRollersHal;

public class Robot {
    public static final String k_canivoreCan = "canivore1";

    public final CommandXboxController m_driveController = new CommandXboxController(0);

    public final Feeder m_feeder;
    public final Serializer m_serializer;
    public final Elevator m_elevator;

    private Command m_autoCommand = null;

    public Robot() {
        if (RobotRunner.isReal()) {
            m_feeder = new Feeder(new FeederRollersHal(), new SensorHal());
            m_serializer = new Serializer(new SerializerRollersHal());
            m_elevator = new Elevator(new ElevatorRollersHal(), new frc.robot.subsystems.elevator.sensor.SensorHal());
        } else {
            m_feeder = new Feeder(new FeederRollersDummyHal(), new SensorDummyHal());
            m_serializer = new Serializer(new SerializerRollersDummyHal());
            m_elevator = new Elevator(new ElevatorRollersDummyHal(), new frc.robot.subsystems.elevator.sensor.SensorHal());
        }

        configureBindings();
    }

    public void update() {}

    private void configureBindings() {
        m_driveController.b().whileTrue(
            m_feeder.intake().until(m_feeder.getRearSensor()));

        m_driveController.x().whileTrue(
            Commands.race(
                m_feeder.intake(),
                m_serializer.reverseToAmp(),
                m_elevator.loadNote()));
        
        m_driveController.a().whileTrue(
            m_feeder.reverse().until(m_feeder.getFrontSensor().negate()));
        
        m_driveController.y().onTrue(
            m_elevator.ejectNote()
        );
    }

    public void updateAutoCommand() {}

    public Command getAutonomousUCommand() {
        if (m_autoCommand == null) {
            updateAutoCommand();
        }

        return m_autoCommand;
    }
}
