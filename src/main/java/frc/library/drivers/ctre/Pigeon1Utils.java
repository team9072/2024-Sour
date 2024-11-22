/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library.drivers.ctre;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.BasePigeon;
import com.ctre.phoenix.sensors.PigeonIMU;

/** Contains utilities code meant for a CTRE Pigeon 1 IMU. */
public class Pigeon1Utils extends BasePigeonUtils {
    /** The CTRE product name of the device. */
    public static String k_deviceProductName = "Pigeon1";

    /** Pigeon 1 configuration options. Contains the class' frame period class. */
    public static class Pigeon1Settings extends BasePigeonSettings {
        /**
         * Given a Pigeon 1 device, configures the device using the settings stored within this class.
         *
         * @param pigeon1 The device to be configured.
         * @param printErrorTrace If true, append stack trace to error string when error is reported.
         */
        public void configDevice(PigeonIMU pigeon1, boolean printErrorTrace) {
            super.configDevice((BasePigeon) pigeon1, k_deviceProductName, printErrorTrace);
        }

        /**
         * Given a Pigeon 1 device, configures the device using the settings stored within this class.
         * Defaults to print out errors.
         *
         * @param pigeon1 The device to be configured.
         */
        public void configDevice(PigeonIMU pigeon1) {
            configDevice(pigeon1, k_defaultErrorPrintTrace);
        }
    }

    /**
     * Sets it up a Pigeon 1 with FRC Team 836 The RoboBees's preferred default settings.
     *
     * @param canId The device ID of the Pigeon 1.
     * @param printErrorTrace If true, any errors that get printed will also print the error trace.
     * @return A newly created PigeonIMU object with default settings.
     */
    public static PigeonIMU createDefaultPigeon1(int canId, boolean printErrorTrace) {
        PigeonIMU pigeon1 = new PigeonIMU(canId);
        (new Pigeon1Settings()).configDevice(pigeon1, k_deviceProductName, printErrorTrace);
        return pigeon1;
    }

    /**
     * Sets it up a Pigeon 1 with FRC Team 836 The RoboBees's preferred default settings. Defaults to
     * print out errors.
     *
     * @param canId The device ID of the Pigeon 1.
     * @return A newly created PigeonIMU object with default settings.
     */
    public static PigeonIMU createDefaultPigeon1(int canId) {
        return createDefaultPigeon1(canId, k_defaultErrorPrintTrace);
    }

    /**
     * Method for checking Pigeon 1 errors. By default, if there is an error, the error trace will be
     * printed.
     *
     * @param error Error code returned from CTRE HAL call.
     * @param functionName String representing the function that returned the error.
     */
    protected static void checkError(ErrorCode error, String functionName) {
        checkError(error, k_deviceProductName, functionName, k_defaultErrorPrintTrace);
    }

    /**
     * Method for checking Pigeon 1 errors.
     *
     * @param error Error code returned from CTRE HAL call.
     * @param functionName String representing the function that returned the error.
     * @param printErrorTrace If true, append stack trace to error string.
     */
    protected static void checkError(ErrorCode error, String functionName, boolean printErrorTrace) {
        checkError(error, k_deviceProductName, functionName, printErrorTrace);
    }
}
