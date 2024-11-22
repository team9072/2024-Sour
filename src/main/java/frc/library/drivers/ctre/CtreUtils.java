/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library.drivers.ctre;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import org.growingstems.measurements.Measurements.Time;

/** Extended by other helper classes/functions to setup CTRE CAN devices. */
public abstract class CtreUtils {

    protected static final boolean k_defaultErrorPrintTrace = true;

    public static final Time k_defaultConfiguratorTimeout = Time.seconds(0.5);

    /** What role a device plays in a master/follower relationship if there is one. */
    public enum MasterFollowerRole {
        ALONE,
        MASTER,
        FOLLOWER,
        REMOTE_SENSOR
    }

    /** Contains settings relevant to all base CTRE motor controllers. */
    protected static class BaseCtreSettings {
        /**
         * Timeout that is used for hardware calls that are configuring the device. These calls can take
         * longer and should be given a little more time to occur.
         */
        public static final int k_cfgTimeout_ms = 100;

        /**
         * Timeout that is used for hardware calls that are configuring the device that take longer then
         * normal config calls. These calls can take longer and should be given even more time to occur.
         */
        public static final int k_cfgTimeoutLong_ms = 200;

        /**
         * Timeout that is used for regular set hardware calls. These calls are generally quick and
         * don't need a long timeout.
         */
        public static final int k_setTimeout_ms = 20;

        protected int m_configTimeout_ms = k_cfgTimeout_ms;

        public void setConfigTimeout(int period_ms) {
            m_configTimeout_ms = period_ms;
        }
    }

    /** Container class for all CTRE CAN Frame Periods */
    protected static class BaseFramePeriodConfiguration {
        /**
         * The max period that the hardware will accept when setting status rates. Any value larger than
         * this will just wrap.
         */
        public static final int k_maxPeriod_ms = 255;

        public static final int k_minPeriod_ms = 1;

        /**
         * A reasonable minimum period that the hardware will accept when setting status rates. Any
         * value smaller than this is considered un-reasonable and is highly not recommended.
         */
        public static final int k_reasonableMinPeriod_ms = 5;

        protected int coercePeriod(int period) {
            if (period < k_minPeriod_ms) {
                return k_minPeriod_ms;
            } else if (period > k_maxPeriod_ms) {
                return k_maxPeriod_ms;
            } else {
                return period;
            }
        }
    }

    /**
     * Method for checking CTRE product errors.
     *
     * @param error Error code returned from CTRE HAL call.
     * @param deviceName String representing the device that returned the error.
     * @param functionName String representing the function that returned the error.
     * @param printErrorTrace If true, append stack trace to error string.
     */
    protected static void checkError(
            ErrorCode error, String deviceName, String functionName, boolean printErrorTrace) {
        if (error != ErrorCode.OK) {
            DriverStation.reportError(
                    deviceName + " error code (" + error + ") found while running: " + functionName,
                    printErrorTrace);
        }
    }

    public static final ControlRequest getControlRequest(NeutralModeValue neutralMode) {
        return switch (neutralMode) {
            case Coast -> new CoastOut();
            case Brake -> new StaticBrake();
        };
    }
}
