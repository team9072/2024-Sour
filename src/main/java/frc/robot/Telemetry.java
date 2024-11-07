/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Telemetry {
    // -------------------
    //    General Robot
    // -------------------
    public static class TeleRobot {
        private static final ShuffleboardTab robotTab = Shuffleboard.getTab("Robot");

        public static final GenericEntry logFile =
                robotTab.add("Log File", "Unset").withPosition(5, 0).withSize(2, 1).getEntry();

        public static final GenericEntry loggedBytes =
                robotTab.add("Bytes per Frame", 0).withPosition(4, 1).withSize(1, 1).getEntry();

        public static final GenericEntry loggedKBPerSecond = robotTab
                .add("Bytes per Second (kBps)", 0)
                .withPosition(5, 1)
                .withSize(2, 1)
                .getEntry();

        public static final GenericEntry loggedTotalBytesKB = robotTab
                .add("Total Bytes Logged (kB)", 0)
                .withPosition(5, 2)
                .withSize(2, 1)
                .getEntry();
    }
}
