/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library.drivers.ctre;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.BasePigeon;
import com.ctre.phoenix.sensors.PigeonIMU_ControlFrame;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

/** Contains utilities code meant for Pigeon 1 and Pigeon 2 IMUs. */
public abstract class BasePigeonUtils extends CtreUtils {
    /**
     * Expanded Pigeon 2 configuration options. Combines both the vendor class for Pigeon 2
     * configuration options with this classes frame period class to allow for configuring everything
     * with a single container class.
     */
    protected static class BasePigeonSettings extends BaseCtreSettings {
        public PigeonFramePeriodConfiguration framePeriodConfig = new PigeonFramePeriodConfiguration();

        /**
         * Given a BaseTalon device, configures the device using the settings stored within this class.
         *
         * @param basePigeon The device to be configured.
         * @param deviceName The model name of the device to be configured.
         * @param printErrorTrace If true, append stack trace to error string when error is reported.
         */
        protected void configDevice(BasePigeon basePigeon, String deviceName, boolean printErrorTrace) {
            ErrorCode error = ErrorCode.OK;
            // Start by resetting the device to Factory Default
            error = basePigeon.configFactoryDefault(m_configTimeout_ms);
            checkError(error, deviceName, "configFactoryDefault", printErrorTrace);

            framePeriodConfig.configFramePeriod(
                    basePigeon, deviceName, m_configTimeout_ms, printErrorTrace);
        }
    }

    /** All the various status frame periods that be configured on both Pigeon's. */
    public static class PigeonFramePeriodConfiguration extends BaseFramePeriodConfiguration {
        // Status Frames
        protected int m_condStatus_1_General = BaseFramePeriodConfiguration.k_maxPeriod_ms;
        protected int m_condStatus_9_SixDeg_YPR = 10;
        protected int m_condStatus_6_SensorFusion = BaseFramePeriodConfiguration.k_maxPeriod_ms;
        protected int m_condStatus_11_GyroAccum = BaseFramePeriodConfiguration.k_maxPeriod_ms;
        protected int m_condStatus_2_GeneralCompass = BaseFramePeriodConfiguration.k_maxPeriod_ms;
        protected int m_condStatus_3_GeneralAccel = BaseFramePeriodConfiguration.k_maxPeriod_ms;
        protected int m_condStatus_10_SixDeg_Quat = BaseFramePeriodConfiguration.k_maxPeriod_ms;
        protected int m_rawStatus_4_Mag = BaseFramePeriodConfiguration.k_maxPeriod_ms;
        protected int m_biasedStatus_2_Gyro = BaseFramePeriodConfiguration.k_maxPeriod_ms;
        protected int m_biasedStatus_4_Mag = BaseFramePeriodConfiguration.k_maxPeriod_ms;
        protected int m_biasedStatus_6_Accel = BaseFramePeriodConfiguration.k_maxPeriod_ms;

        // Control Frames
        protected int m_control_1 = BaseFramePeriodConfiguration.k_maxPeriod_ms;

        /**
         * Sets the CondStatus_1_General status frame period in milliseconds.
         *
         * @param period_ms The status frame period milliseconds.
         */
        public void setCondStatus_1_General(int period_ms) {
            m_condStatus_1_General = coercePeriod(period_ms);
        }

        /**
         * Sets the CondStatus_9_SixDeg_YPR status frame period in milliseconds.
         *
         * @param period_ms The status frame period milliseconds.
         */
        public void setCondStatus_9_SixDeg_YPR(int period_ms) {
            m_condStatus_9_SixDeg_YPR = coercePeriod(period_ms);
        }

        /**
         * Sets the CondStatus_6_SensorFusion status frame period in milliseconds.
         *
         * @param period_ms The status frame period milliseconds.
         */
        public void setCondStatus_6_SensorFusion(int period_ms) {
            m_condStatus_6_SensorFusion = coercePeriod(period_ms);
        }

        /**
         * Sets the CondStatus_11_GyroAccum status frame period in milliseconds.
         *
         * @param period_ms The status frame period milliseconds.
         */
        public void setCondStatus_11_GyroAccum(int period_ms) {
            m_condStatus_11_GyroAccum = coercePeriod(period_ms);
        }

        /**
         * Sets the CondStatus_2_GeneralCompass status frame period in milliseconds.
         *
         * @param period_ms The status frame period milliseconds.
         */
        public void setCondStatus_2_GeneralCompass(int period_ms) {
            m_condStatus_2_GeneralCompass = coercePeriod(period_ms);
        }

        /**
         * Sets the CondStatus_3_GeneralAccel status frame period in milliseconds.
         *
         * @param period_ms The status frame period milliseconds.
         */
        public void setCondStatus_3_GeneralAccel(int period_ms) {
            m_condStatus_3_GeneralAccel = coercePeriod(period_ms);
        }

        /**
         * Sets the CondStatus_10_SixDeg_Quat status frame period in milliseconds.
         *
         * @param period_ms The status frame period milliseconds.
         */
        public void setCondStatus_10_SixDeg_Quat(int period_ms) {
            m_condStatus_10_SixDeg_Quat = coercePeriod(period_ms);
        }

        /**
         * Sets the RawStatus_4_Mag status frame period in milliseconds.
         *
         * @param period_ms The status frame period milliseconds.
         */
        public void setRawStatus_4_Mag(int period_ms) {
            m_rawStatus_4_Mag = coercePeriod(period_ms);
        }

        /**
         * Sets the BiasedStatus_2_Gyro status frame period in milliseconds.
         *
         * @param period_ms The status frame period milliseconds.
         */
        public void setBiasedStatus_2_Gyro(int period_ms) {
            m_biasedStatus_2_Gyro = coercePeriod(period_ms);
        }

        /**
         * Sets the BiasedStatus_4_Mag status frame period in milliseconds.
         *
         * @param period_ms The status frame period milliseconds.
         */
        public void setBiasedStatus_4_Mag(int period_ms) {
            m_biasedStatus_4_Mag = coercePeriod(period_ms);
        }

        /**
         * Sets the BiasedStatus_6_Accel status frame period in milliseconds.
         *
         * @param period_ms The status frame period milliseconds.
         */
        public void setBiasedStatus_6_Accel(int period_ms) {
            m_biasedStatus_6_Accel = coercePeriod(period_ms);
        }

        /**
         * Sets the Control_1 control frame period in milliseconds.
         *
         * @param period_ms The control frame period milliseconds.
         */
        public void setControl_1(int period_ms) {
            m_control_1 = coercePeriod(period_ms);
        }

        /**
         * Sets the period of all CAN frames for a Pigeon.
         *
         * <p>These settings are not persistent and is lost when device is reset. If this is a concern,
         * calling application can use hasResetOccurred() to determine if the status frame needs to be
         * reconfigured.
         *
         * @param pigeon pigeon to configure
         * @param deviceName the product name of the CTRE device
         * @param timeout_ms Timeout value in ms. If nonzero, function will wait for config success and
         *     report an error if it times out. If zero, no blocking or checking is performed.
         * @param printErrorTrace If true, append stack trace to error string when error is reported.
         */
        protected void configFramePeriod(
                BasePigeon pigeon, String deviceName, int timeout_ms, boolean printErrorTrace) {
            ErrorCode error = ErrorCode.OK;

            // Calibration Status
            // IMU Temperature - Default Period >100ms
            error = pigeon.setStatusFramePeriod(
                    PigeonIMU_StatusFrame.CondStatus_1_General, m_condStatus_1_General, timeout_ms);
            checkError(error, deviceName, "setStatusFramePeriod", printErrorTrace);

            // Six degree fused Yaw, Pitch, Roll - Default Period 10ms
            error = pigeon.setStatusFramePeriod(
                    PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, m_condStatus_9_SixDeg_YPR, timeout_ms);
            checkError(error, deviceName, "setStatusFramePeriod", printErrorTrace);

            // Nine degree fused Yaw, Pitch, Roll (requires magnetometer calibration). -
            // Default Period 10ms
            error = pigeon.setStatusFramePeriod(
                    PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, m_condStatus_6_SensorFusion, timeout_ms);
            checkError(error, deviceName, "setStatusFramePeriod", printErrorTrace);

            // Accumulated Gyro Angles - Default Period 20ms
            error = pigeon.setStatusFramePeriod(
                    PigeonIMU_StatusFrame.CondStatus_11_GyroAccum, m_condStatus_11_GyroAccum, timeout_ms);
            checkError(error, deviceName, "setStatusFramePeriod", printErrorTrace);

            // This is undocumented in Phoenix Documents
            error = pigeon.setStatusFramePeriod(
                    PigeonIMU_StatusFrame.CondStatus_2_GeneralCompass,
                    m_condStatus_2_GeneralCompass,
                    timeout_ms);
            checkError(error, deviceName, "setStatusFramePeriod", printErrorTrace);

            // Accelerometer derived angles - Default Period >100m
            error = pigeon.setStatusFramePeriod(
                    PigeonIMU_StatusFrame.CondStatus_3_GeneralAccel, m_condStatus_3_GeneralAccel, timeout_ms);
            checkError(error, deviceName, "setStatusFramePeriod", printErrorTrace);

            // Six degree fused Quaternion - Default Period >100ms
            error = pigeon.setStatusFramePeriod(
                    PigeonIMU_StatusFrame.CondStatus_10_SixDeg_Quat, m_condStatus_10_SixDeg_Quat, timeout_ms);
            checkError(error, deviceName, "setStatusFramePeriod", printErrorTrace);

            // This is undocumented in Phoenix Documents
            error = pigeon.setStatusFramePeriod(
                    PigeonIMU_StatusFrame.RawStatus_4_Mag, m_rawStatus_4_Mag, timeout_ms);
            checkError(error, deviceName, "setStatusFramePeriod", printErrorTrace);

            // Biased gyro values (x,y,z) - Default Period >100ms
            error = pigeon.setStatusFramePeriod(
                    PigeonIMU_StatusFrame.BiasedStatus_2_Gyro, m_biasedStatus_2_Gyro, timeout_ms);
            checkError(error, deviceName, "setStatusFramePeriod", printErrorTrace);

            // Unprocessed magnetometer values (x,y,z) - Default Period 20ms
            error = pigeon.setStatusFramePeriod(
                    PigeonIMU_StatusFrame.BiasedStatus_4_Mag, m_biasedStatus_4_Mag, timeout_ms);
            checkError(error, deviceName, "setStatusFramePeriod", printErrorTrace);

            // Biased accelerometer values (x,y,z) - Default Period >100ms
            error = pigeon.setStatusFramePeriod(
                    PigeonIMU_StatusFrame.BiasedStatus_6_Accel, m_biasedStatus_6_Accel, timeout_ms);
            checkError(error, deviceName, "setStatusFramePeriod", printErrorTrace);

            // Control Frame
            // This is undocumented in Phoenix Documents
            pigeon.setControlFramePeriod(PigeonIMU_ControlFrame.Control_1, m_control_1);
        }
    }
}
