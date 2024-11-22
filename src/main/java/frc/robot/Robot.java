/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

public class Robot {
    public static final String k_canivoreCan = "CANivore";

    private Command m_autoCommand = null;

    public Robot() {
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
