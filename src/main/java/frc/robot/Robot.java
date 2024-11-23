/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.logging.LogBuilder;
import frc.robot.subsystems.Drive;

public class Robot {
    public static final String k_canivoreCan = "CANivore";
    private Command m_autoCommand = null;
    private Drive m_drive;
    private TeleopControls m_controls = new TeleopControls();

    public Robot(LogBuilder builder) {
        configureBindings();

        m_drive = new Drive(builder);
    }

    public void update() {}

    private void configureBindings() {
        m_drive.setDefaultCommand(m_drive.getDirectDriveFCommand(m_controls.drivePowerSupplier, m_controls.turnPowerSupplier, DriveRequestType.OpenLoopVoltage));
    }

    public void updateAutoCommand() {}

    public Command getAutonomousUCommand() {
        if (m_autoCommand == null) {
            updateAutoCommand();
        }

        return m_autoCommand;
    }
}
