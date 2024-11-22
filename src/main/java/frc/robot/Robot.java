/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.deploy.IntakeDeployDummyHal;
import frc.robot.subsystems.intake.deploy.IntakeDeployHal;
import frc.robot.subsystems.intake.rollers.IntakeRollersDummyHal;
import frc.robot.subsystems.intake.rollers.IntakeRollersHal;

public class Robot {
    public static final String k_canivoreCan = "canivore1";

    private Command m_autoCommand = null;
    private final Intake m_intake;

    public Robot() {
        if (RobotRunner.isReal()) {
            m_intake = new Intake(new IntakeRollersHal(), new IntakeDeployHal());
        } else {
            m_intake = new Intake(new IntakeRollersDummyHal(), new IntakeDeployDummyHal());
        }

        configureBindings();
    }

    public void update() {}

    private void configureBindings() {}

    public void updateAutoCommand() {}

    public Command getAutonomousUCommand() {
        if (m_autoCommand == null) {
            updateAutoCommand();
        }

        return m_autoCommand;
    }
}
