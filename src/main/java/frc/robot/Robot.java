/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.logging.LogBuilder;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederRollersDummyHal;
import frc.robot.subsystems.feeder.FeederRollersHal;
import frc.robot.subsystems.feeder.sensors.SensorDummyHal;
import frc.robot.subsystems.feeder.sensors.SensorHal;

public class Robot {
    public static final String k_canivoreCan = "canivore1";

    public final CommandXboxController m_driveController = new CommandXboxController(0);

    public final Feeder m_feeder;

    private Command m_autoCommand = null;

    public Robot() {
        if (RobotRunner.isReal()) {
            m_feeder = new Feeder(new FeederRollersHal(), new SensorHal());
        } else {
            m_feeder = new Feeder(new FeederRollersDummyHal(), new SensorDummyHal());
        }

        configureBindings();
    }

    public void update() {}

    private void configureBindings() {
        m_driveController.b().whileTrue(
            m_feeder.intake().until(m_feeder.getRearSensor()));
        
        m_driveController.a().whileTrue(
            m_feeder.reverse().until(m_feeder.getFrontSensor().negate()));
    }

    public void updateAutoCommand() {}

    public Command getAutonomousUCommand() {
        if (m_autoCommand == null) {
            updateAutoCommand();
        }

        return m_autoCommand;
    }
}
