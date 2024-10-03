/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.growingstems.measurements.Measurements.Time;

public class RobotRunner extends TimedRobot {
    private static final Time k_loopTime = Time.milliseconds(20.0);

    private Robot m_robot;
    private Command m_autonomousCommand;

    public static boolean isCompetition() {
        return false;
    }

    public RobotRunner() {
        super(k_loopTime.asSeconds());
        CommandScheduler.getInstance().setPeriod(k_loopTime.asSeconds());

        enableLiveWindowInTest(false);
        LiveWindow.setEnabled(false);
        Shuffleboard.disableActuatorWidgets();
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    @Override
    public void robotInit() {
        m_robot = new Robot();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {
        m_robot.updateAutoCommand();
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robot.getAutonomousUCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
