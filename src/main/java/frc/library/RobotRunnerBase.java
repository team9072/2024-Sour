/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.internal.DriverStationModeThread;
import org.growingstems.frc.util.RobotMatchState;
import org.growingstems.frc.util.RobotMatchState.MatchMode;
import org.growingstems.frc.util.RobotMatchState.MatchState;
import org.growingstems.frc.util.WpiTimeSource;
import org.growingstems.measurements.Measurements.Time;
import org.growingstems.util.timer.TimeSource;
import org.growingstems.util.timer.Timer;

/**
 * Almost bare-bones robot runner class. This class contains a minimalistic implementation of
 * startCompetition() which acts as the main thread loop of the robot code. Also includes timing
 * metrics to check how long each part of robot code takes to run.
 */
public abstract class RobotRunnerBase extends RobotBase {
    /** Called the first time robot code is ran. */
    protected abstract void robotInit();

    /**
     * Called every loop iteration of the main thread, no matter what mode or state the robot is in.
     * For instance, this will run even when the robot is disabled.
     */
    protected abstract void robotPeriodic();

    /** Called when the estop is hit. */
    protected abstract void emergencyStopInit();

    /** Called while the robot is estop'd. */
    protected abstract void emergencyStopPeriodic();

    /** Called once when the robot transitions to being disabled. */
    protected abstract void disabledInit();

    /** Called while the robot is disabled. */
    protected abstract void disabledPeriodic();

    /** Called whenever the robot is being enabled, and before enabled related code is ran. */
    protected abstract void disabledExit();

    /** Called once when autonomous mode is switched to and enabled. */
    protected abstract void autonomousInit();

    /** Called while autonomous mode is running. */
    protected abstract void autonomousPeriodic();

    /** Called once before switching from autonomous mode to another mode or before being disabled. */
    protected abstract void autonomousExit();

    /** Called once when teleop mode is switched to and enabled. */
    protected abstract void teleopInit();

    /** Called while teleop mode is running. */
    protected abstract void teleopPeriodic();

    /** Called once before switching from teleop mode to another mode or before being disabled. */
    protected abstract void teleopExit();

    /** Called once when test mode is switched to and enabled. */
    protected abstract void testInit();

    /** Called while test mode is running. */
    protected abstract void testPeriodic();

    /** Called once before switching from test mode to another mode or before being disabled. */
    protected abstract void testExit();

    private MatchState m_previousMatchState = MatchState.UNKNOWN_DISABLED;
    private volatile boolean m_exit = false;

    // Timers
    private final TimeSource m_wpiTimeSource = new WpiTimeSource();
    private Timer m_mainLoopTimer = m_wpiTimeSource.createTimer(); // .start();
    private Timer m_mainExecutionTimer = m_wpiTimeSource.createTimer(); // .start();
    private Timer m_sectionTimer = m_wpiTimeSource.createTimer(); // .start();

    // Run time variables
    private Time m_mainLoopTime = Time.ZERO;
    private Time m_mainExecutionTime = Time.ZERO;
    private Time m_exitTime = Time.ZERO;
    private Time m_initTime = Time.ZERO;
    private Time m_robotPeriodicTime = Time.ZERO;
    private Time m_modePeriodicTime = Time.ZERO;
    private Time m_sleepTime = Time.ZERO;

    @Override
    public void startCompetition() {
        robotInit();

        DriverStationModeThread modeThread = new DriverStationModeThread();

        int event = WPIUtilJNI.createEvent(false, false);

        DriverStation.provideRefreshedDataEventHandle(event);

        // Tell the DS that the robot is ready to be enabled
        System.out.println("********** Robot program startup complete **********");
        DriverStationJNI.observeUserProgramStarting();

        m_mainLoopTimer.reset();
        while (!Thread.currentThread().isInterrupted() && !m_exit) {
            m_mainExecutionTimer.reset();
            // This has to be called in order for RobotMatchState, which uses DriverStation,
            // to work.
            DriverStation.refreshData();
            var currentMatchState = RobotMatchState.getMatchState();
            var matchStateChanged = m_previousMatchState != currentMatchState;
            m_previousMatchState = currentMatchState;

            if (matchStateChanged) {
                // Run Exit Functions
                m_sectionTimer.reset();
                if (m_previousMatchState.enabled) {
                    if (m_previousMatchState.matchMode == MatchMode.AUTO) {
                        autonomousExit();
                        modeThread.inAutonomous(false);
                    } else if (m_previousMatchState.matchMode == MatchMode.TELE) {
                        teleopExit();
                        modeThread.inTeleop(false);
                    } else if (m_previousMatchState.matchMode == MatchMode.TEST) {
                        testExit();
                        modeThread.inTest(false);
                    }
                } else {
                    disabledExit();
                    modeThread.inDisabled(false);
                }
                m_exitTime = m_sectionTimer.get();

                // Run Init Functions
                m_sectionTimer.reset();
                if (currentMatchState.enabled) {
                    if (currentMatchState.matchMode == MatchMode.AUTO) {
                        modeThread.inAutonomous(true);
                        autonomousInit();
                    } else if (currentMatchState.matchMode == MatchMode.TELE) {
                        modeThread.inTeleop(true);
                        teleopInit();
                    } else if (currentMatchState.matchMode == MatchMode.TEST) {
                        modeThread.inTest(true);
                        testInit();
                    } else if (currentMatchState.matchMode == MatchMode.EMERGENCY_STOPPED) {
                        emergencyStopInit();
                    }
                } else {
                    modeThread.inDisabled(true);
                    disabledInit();
                }
                m_initTime = m_sectionTimer.get();
            } else {
                m_initTime = Time.ZERO;
                m_exitTime = Time.ZERO;
            }

            m_sectionTimer.reset();
            robotPeriodic();
            m_robotPeriodicTime = m_sectionTimer.get();

            // Run Periodic Functions
            m_sectionTimer.reset();
            if (currentMatchState.enabled) {
                if (currentMatchState.matchMode == MatchMode.EMERGENCY_STOPPED) {
                    emergencyStopPeriodic();
                } else if (currentMatchState.matchMode == MatchMode.AUTO) {
                    autonomousPeriodic();
                } else if (currentMatchState.matchMode == MatchMode.TELE) {
                    teleopPeriodic();
                } else if (currentMatchState.matchMode == MatchMode.TEST) {
                    testPeriodic();
                }
            } else {
                disabledPeriodic();
            }
            m_modePeriodicTime = m_sectionTimer.get();
            m_mainExecutionTime = m_mainExecutionTimer.get();

            m_sectionTimer.reset();
            try {
                // Other options include Thread.yield() and Thread.sleep(0).
                // I decided on sleep(1) to hopefully guarantee the CPU allows other threads to
                // run.
                Thread.sleep(1);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            m_sleepTime = m_sectionTimer.get();
            m_mainLoopTime = m_mainLoopTimer.reset();
        }

        DriverStation.removeRefreshedDataEventHandle(event);
        modeThread.close();
    }

    @Override
    public void endCompetition() {
        m_exit = true;
    }

    /**
     * Gets the amount of time it took for the main thread to fully loop.
     *
     * @return the main loop time
     */
    public Time getMainLoopTime() {
        return m_mainLoopTime;
    }

    /**
     * Gets the amount of time it took to execute all robot specific code that is ran in the main
     * thread. This includes everything except the time spent sleeping the thread in order to let
     * other threads have a turn.
     *
     * @return the main execution run time
     */
    public Time getMainExecutionTime() {
        return m_mainExecutionTime;
    }

    /**
     * Gets the amount of time it took to execute the code in the exit of the previous mode. Exit for
     * the previous mode is ran before the init code for the current mode within the current loop
     * iteration. Will return zero if exit wasn't ran.
     *
     * @return the previous mode's exit run time
     */
    public Time getExitTime() {
        return m_exitTime;
    }

    /**
     * Gets the amount of time it took to execute the code in the init of the current mode. Init for
     * the current mode is ran after the exit code for the previous mode within the current loop
     * iteration. Will return zero if init wasn't ran.
     *
     * @return the current mode's init run time
     */
    public Time getInitTime() {
        return m_initTime;
    }

    /**
     * Gets the amount of time it took to run the general robot periodic code. This is the code ran
     * within the {@link robotPeriodic} function which always runs no matter what mode or state the
     * robot code is in.
     *
     * @return the robot periodic run time
     */
    public Time getRobotPeriodicTime() {
        return m_robotPeriodicTime;
    }

    /**
     * Gets the amount of time it took to run the mode specific periodic code. This is the code ran
     * within the periodic function for the currently running mode.
     *
     * @return the mode periodic run time
     */
    public Time getModePeriodicTime() {
        return m_modePeriodicTime;
    }

    /**
     * Gets the amount of time the main thread slept while the CPU let other threads run.
     *
     * @return the time spent sleeping the main thread
     */
    public Time getSleepTime() {
        return m_sleepTime;
    }
}
