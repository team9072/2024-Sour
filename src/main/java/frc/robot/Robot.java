/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.wheels.ShooterWheelsDummyHal;
import frc.robot.subsystems.shooter.wheels.ShooterWheelsHal;

public class Robot {
    public static final String k_canivoreCan = "canivore1";

    private final CommandXboxController m_driverController = new CommandXboxController(0);

    private final Shooter m_shooter;

    private Command m_autoCommand = null;

    public Robot() {
        if (RobotRunner.isReal()) {
            m_shooter = new Shooter(new ShooterWheelsHal());
        } else {
            m_shooter = new Shooter(new ShooterWheelsDummyHal());
        }

        configureBindings();
    }

    public void update() {}

    private void configureBindings() {
        // Don't allow SysId at comp
        if (!RobotRunner.isCompetition()) configureSysIdBindings();
    }

    private void configureSysIdBindings() {
        // Shooter wheels just spin, so it is safe to lt them run unattended
        m_driverController.a().onTrue(Commands.sequence(
            Commands.runOnce(SignalLogger::start),
            // Top wheels SysId
            m_shooter.topWheelsSysIdDynamic(Direction.kForward), Commands.waitSeconds(2),
            m_shooter.topWheelsSysIdDynamic(Direction.kReverse), Commands.waitSeconds(2),
            m_shooter.topWheelsSysIdQuasistatic(Direction.kForward), Commands.waitSeconds(2),
            m_shooter.topWheelsSysIdQuasistatic(Direction.kReverse), Commands.waitSeconds(2),
            // Bottom wheels SysId
            m_shooter.bottomWheelsSysIdDynamic(Direction.kForward), Commands.waitSeconds(2),
            m_shooter.bottomWheelsSysIdDynamic(Direction.kReverse), Commands.waitSeconds(2),
            m_shooter.bottomWheelsSysIdQuasistatic(Direction.kForward), Commands.waitSeconds(2),
            m_shooter.bottomWheelsSysIdQuasistatic(Direction.kReverse), Commands.waitSeconds(2),
            Commands.runOnce(SignalLogger::stop)
        ).onlyIf(RobotState::isTest));
    }

    public void updateAutoCommand() {}

    public Command getAutonomousUCommand() {
        if (m_autoCommand == null) {
            updateAutoCommand();
        }

        return m_autoCommand;
    }
}
