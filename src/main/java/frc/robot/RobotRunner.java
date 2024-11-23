/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.library.RobotRunnerBase;
import frc.robot.logging.LogBuilder;
import java.io.File;
import java.nio.file.Paths;
import org.growingstems.measurements.Measurements.Time;

public class RobotRunner extends RobotRunnerBase {
    public final File k_logDir =
            Paths.get(Filesystem.getOperatingDirectory().getPath(), "logs").toFile();

    private static final Time k_maxDesiredLoopTime = Time.milliseconds(20.0);

    private Robot m_robot;
    private Command m_autonomousCommand;

    public static boolean isCompetition() {
        return false;
    }

    public RobotRunner() {
        CommandScheduler.getInstance().setPeriod(k_maxDesiredLoopTime.asSeconds());
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    @Override
    public void robotInit() {
        var builder = new LogBuilder(k_logDir, "RobotLog");
        m_robot = new Robot(builder);
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

    @Override
    protected void emergencyStopInit() {}

    @Override
    protected void emergencyStopPeriodic() {}
}
