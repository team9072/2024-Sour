/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library.drivers.ctre;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.BaseMotorControllerConfiguration;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;

/** Provides helper classes/functions to setup CTRE Motor CAN devices. */
public abstract class BaseCanMotorControllerUtils extends CtreUtils {
    public enum PidfSlot {
        SLOT_0(0),
        SLOT_1(1),
        SLOT_2(2),
        SLOT_3(3);

        public final int num;

        private PidfSlot(int num) {
            this.num = num;
        }
    }

    public enum PidfIdx {
        PRIMARY(0),
        AUXILIARY(1);

        public final int idx;

        private PidfIdx(int idx) {
            this.idx = idx;
        }
    }

    /** Contains settings relevant to all base CTRE motor controllers. */
    protected static class BaseControllerSettings extends BaseCtreSettings {
        /** Setting for enabling Voltage Compensation. */
        public boolean voltageCompEnabled = true;

        public boolean sensorPhase = false;

        public NeutralMode neutralMode = NeutralMode.EEPROMSetting;

        public boolean inverted = false;

        public PidfSlot pidPrimarySlot = PidfSlot.SLOT_0;

        public PidfSlot pidAuxilarySlot = PidfSlot.SLOT_0;

        /** The preferred voltage compensation to use on CTRE motor devices */
        protected static double k_preferredVoltageCompSaturation_V = 12.0;

        /** Constructs default base CTRE controller settings. */
        public BaseControllerSettings() {
            super();
        }

        public static void setPidSlotSettings(
                PidfSlot slot, SlotConfiguration slotConfig, BaseMotorControllerConfiguration config) {
            switch (slot) {
                case SLOT_0:
                    config.slot0 = slotConfig;
                    break;
                case SLOT_1:
                    config.slot1 = slotConfig;
                    break;
                case SLOT_2:
                    config.slot2 = slotConfig;
                    break;
                case SLOT_3:
                    config.slot3 = slotConfig;
                    break;
            }
        }

        /**
         * Given a CTRE motor controller, configures the device with all the settings contained within
         * this class.
         *
         * @param baseController Base CTRE motor controller to configure.
         * @param deviceName The model name of the CTRE device.
         * @param printErrorTrace If true, append stack trace to error string when error is reported.
         */
        protected void configDevice(
                BaseMotorController baseController, String deviceName, boolean printErrorTrace) {
            baseController.enableVoltageCompensation(voltageCompEnabled);
            baseController.setSensorPhase(sensorPhase);
            baseController.setNeutralMode(neutralMode);
            baseController.setInverted(inverted);
            baseController.selectProfileSlot(pidPrimarySlot.num, PidfIdx.PRIMARY.idx);
            baseController.selectProfileSlot(pidAuxilarySlot.num, PidfIdx.AUXILIARY.idx);

            baseController.neutralOutput();
        }
    }

    /** Container class for Frame Period settings used by all CTRE CAN Motor Controllers. */
    public static class FramePeriodConfiguration extends BaseFramePeriodConfiguration {
        // Status Frames
        protected int m_status_1_General;
        protected int m_status_2_Feedback0;
        protected int m_status_4_AinTempVbat;
        protected int m_status_6_Misc;
        protected int m_status_7_CommStatus;
        protected int m_status_9_MotProfBuffer;
        protected int m_status_10_Targets;
        protected int m_status_12_Feedback1;
        protected int m_status_13_Base_PIDF0;
        protected int m_status_14_Turn_PIDF1;
        protected int m_status_15_FirmwareApiStatus;
        protected int m_status_17_Targets1;

        // Control Frames
        protected int m_control_3_General;
        protected int m_control_4_Advanced;
        protected int m_control_6_MotProfAddTrajPoint;

        /** Creates a FramePeriodConfiguration object with default settings. */
        public FramePeriodConfiguration() {
            this(MasterFollowerRole.ALONE);
        }

        /**
         * Creates a FramePeriodConfiguration object based on what role the device plays in a
         * master/follower relationship if there is one.
         *
         * @param masterFollowerRole What role the device plays in a master/follower relationship if
         *     there is one.
         */
        public FramePeriodConfiguration(MasterFollowerRole masterFollowerRole) {
            super();

            // Status Frames
            if (masterFollowerRole == MasterFollowerRole.MASTER) {
                m_status_1_General = k_reasonableMinPeriod_ms;
            } else if (masterFollowerRole == MasterFollowerRole.FOLLOWER) {
                m_status_1_General = k_maxPeriod_ms;
            } else {
                m_status_1_General = 10;
            }

            if (masterFollowerRole == MasterFollowerRole.FOLLOWER) {
                m_status_2_Feedback0 = k_maxPeriod_ms;
            } else {
                m_status_2_Feedback0 = 20;
            }

            m_status_4_AinTempVbat = k_maxPeriod_ms;
            m_status_6_Misc = k_maxPeriod_ms;
            m_status_7_CommStatus = k_maxPeriod_ms;
            m_status_9_MotProfBuffer = k_maxPeriod_ms;
            m_status_10_Targets = k_maxPeriod_ms;
            m_status_12_Feedback1 = k_maxPeriod_ms;
            m_status_13_Base_PIDF0 = k_maxPeriod_ms;
            m_status_14_Turn_PIDF1 = k_maxPeriod_ms;
            m_status_15_FirmwareApiStatus = k_maxPeriod_ms;
            m_status_17_Targets1 = k_maxPeriod_ms;

            // Control Frames
            if (masterFollowerRole == MasterFollowerRole.FOLLOWER) {
                m_control_3_General = k_maxPeriod_ms;
            } else {
                m_control_3_General = 10;
            }
            m_control_4_Advanced = k_maxPeriod_ms;
            m_control_6_MotProfAddTrajPoint = k_maxPeriod_ms;
        }

        /**
         * Sets the Status_1_General status frame period in milliseconds.
         *
         * <p>Purpose: Applied Motor Output, Fault Information, Limit Switch Information.
         *
         * @param period_ms The status frame period milliseconds.
         */
        public void setStatus_1_General(int period_ms) {
            m_status_1_General = coercePeriod(period_ms);
        }

        /**
         * Sets the Status_2_Feedback0 status frame period in milliseconds.
         *
         * <p>Purpose: Selected Sensor Position (PID 0), Selected Sensor Velocity (PID 0), Brushed
         * Supply Current Measurement, Sticky Fault Information.
         *
         * @param period_ms The status frame period milliseconds.
         */
        public void setStatus_2_Feedback0(int period_ms) {
            m_status_2_Feedback0 = coercePeriod(period_ms);
        }

        /**
         * Sets the Status_4_AinTempVbat status frame period in milliseconds.
         *
         * <p>Purpose: Analog Input, Supply Battery Voltage, Controller Temperature.
         *
         * @param period_ms The status frame period milliseconds.
         */
        public void setStatus_4_AinTempVbat(int period_ms) {
            m_status_4_AinTempVbat = coercePeriod(period_ms);
        }

        /**
         * Sets the Status_6_Misc status frame period in milliseconds.
         *
         * <p>Purpose:
         *
         * @param period_ms The status frame period milliseconds.
         */
        public void setStatus_6_Misc(int period_ms) {
            m_status_6_Misc = coercePeriod(period_ms);
        }

        /**
         * Sets the Status_7_CommStatus status frame period in milliseconds.
         *
         * <p>Purpose:
         *
         * @param period_ms The status frame period milliseconds.
         */
        public void setStatus_7_CommStatus(int period_ms) {
            m_status_7_CommStatus = coercePeriod(period_ms);
        }

        /**
         * Sets the Status_9_MotProfBuffer status frame period in milliseconds.
         *
         * <p>Purpose: Motion profile buffer status.
         *
         * @param period_ms The status frame period milliseconds.
         */
        public void setStatus_9_MotProfBuffer(int period_ms) {
            m_status_9_MotProfBuffer = coercePeriod(period_ms);
        }

        /**
         * Sets the Status_10_Targets status frame period in milliseconds.
         *
         * <p>Purpose: Motion Profiling/Motion Magic Information
         *
         * @param period_ms The status frame period milliseconds.
         */
        public void setStatus_10_Targets(int period_ms) {
            m_status_10_Targets = coercePeriod(period_ms);
        }

        /**
         * Sets the Status_12_Feedback1 status frame period in milliseconds.
         *
         * <p>Purpose: Selected Sensor Position (Aux PID 1), Selected Sensor Velocity (Aux PID 1)
         *
         * @param period_ms The status frame period milliseconds.
         */
        public void setStatus_12_Feedback1(int period_ms) {
            m_status_12_Feedback1 = coercePeriod(period_ms);
        }

        /**
         * Sets the Status_13_Base_PIDF0 status frame period in milliseconds.
         *
         * <p>Purpose: PID0 (Primary PID) Information.
         *
         * @param period_ms The status frame period milliseconds.
         */
        public void setStatus_13_Base_PIDF0(int period_ms) {
            m_status_13_Base_PIDF0 = coercePeriod(period_ms);
        }

        /**
         * Sets the Status_14_Turn_PIDF1 status frame period in milliseconds.
         *
         * <p>Purpose: PID1 (Auxiliary PID) Information.
         *
         * @param period_ms The status frame period milliseconds.
         */
        public void setStatus_14_Turn_PIDF1(int period_ms) {
            m_status_14_Turn_PIDF1 = coercePeriod(period_ms);
        }

        /**
         * Sets the Status_15_FirmwareApiStatus status frame period in milliseconds.
         *
         * <p>Purpose: Undocumented.
         *
         * @param period_ms The status frame period milliseconds.
         */
        public void setStatus_15_FirmwareApiStatus(int period_ms) {
            m_status_15_FirmwareApiStatus = coercePeriod(period_ms);
        }

        /**
         * Sets the Status_17_Targets1 status frame period in milliseconds.
         *
         * <p>Purpose: Undocumented.
         *
         * @param period_ms The status frame period milliseconds.
         */
        public void setStatus_17_Targets1(int period_ms) {
            m_status_17_Targets1 = coercePeriod(period_ms);
        }

        /**
         * Sets the Control_3_General control frame period in milliseconds.
         *
         * <p>Purpose: Undocumented.
         *
         * @param period_ms The control frame period milliseconds.
         */
        public void setControl_3_General(int period_ms) {
            m_control_3_General = coercePeriod(period_ms);
        }

        /**
         * Sets the Control_4_Advanced control frame period in milliseconds.
         *
         * <p>Purpose: Undocumented.
         *
         * @param period_ms The control frame period milliseconds.
         */
        public void setControl_4_Advanced(int period_ms) {
            m_control_4_Advanced = coercePeriod(period_ms);
        }

        /**
         * Sets the Control_6_MotProfAddTrajPoint control frame period in milliseconds.
         *
         * <p>Purpose: Undocumented.
         *
         * @param period_ms The control frame period milliseconds.
         */
        public void setControl_6_MotProfAddTrajPoint(int period_ms) {
            m_control_6_MotProfAddTrajPoint = coercePeriod(period_ms);
        }

        /**
         * Sets the period of all CAN frames.
         *
         * <p>These settings are not persistent and is lost when device is reset. If this is a concern,
         * calling application can use hasResetOccurred() to determine if the status frame needs to be
         * reconfigured.
         *
         * @param baseController controller to configure
         * @param deviceName the product name of the CTRE device
         * @param timeout_ms Timeout value in ms. If nonzero, function will wait for config success and
         *     report an error if it times out. If zero, no blocking or checking is performed.
         * @param printErrorTrace If true, append stack trace to error string when error is reported.
         */
        public void configFramePeriod(
                BaseMotorController baseController,
                String deviceName,
                int timeout_ms,
                boolean printErrorTrace) {
            ErrorCode error = ErrorCode.OK;
            // Status Frames
            error = baseController.setStatusFramePeriod(
                    StatusFrame.Status_1_General, m_status_1_General, timeout_ms);
            checkError(error, deviceName, "setStatusFramePeriod", printErrorTrace);

            error = baseController.setStatusFramePeriod(
                    StatusFrame.Status_2_Feedback0, m_status_2_Feedback0, timeout_ms);
            checkError(error, deviceName, "setStatusFramePeriod", printErrorTrace);

            error = baseController.setStatusFramePeriod(
                    StatusFrame.Status_4_AinTempVbat, m_status_4_AinTempVbat, timeout_ms);
            checkError(error, deviceName, "setStatusFramePeriod", printErrorTrace);

            error = baseController.setStatusFramePeriod(
                    StatusFrame.Status_6_Misc, m_status_6_Misc, timeout_ms);
            checkError(error, deviceName, "setStatusFramePeriod", printErrorTrace);

            error = baseController.setStatusFramePeriod(
                    StatusFrame.Status_7_CommStatus, m_status_7_CommStatus, timeout_ms);
            checkError(error, deviceName, "setStatusFramePeriod", printErrorTrace);

            error = baseController.setStatusFramePeriod(
                    StatusFrame.Status_9_MotProfBuffer, m_status_9_MotProfBuffer, timeout_ms);
            checkError(error, deviceName, "setStatusFramePeriod", printErrorTrace);

            error = baseController.setStatusFramePeriod(
                    StatusFrame.Status_10_Targets, m_status_10_Targets, timeout_ms);
            checkError(error, deviceName, "setStatusFramePeriod", printErrorTrace);

            error = baseController.setStatusFramePeriod(
                    StatusFrame.Status_12_Feedback1, m_status_12_Feedback1, timeout_ms);
            checkError(error, deviceName, "setStatusFramePeriod", printErrorTrace);

            error = baseController.setStatusFramePeriod(
                    StatusFrame.Status_13_Base_PIDF0, m_status_13_Base_PIDF0, timeout_ms);
            checkError(error, deviceName, "setStatusFramePeriod", printErrorTrace);

            error = baseController.setStatusFramePeriod(
                    StatusFrame.Status_14_Turn_PIDF1, m_status_14_Turn_PIDF1, timeout_ms);
            checkError(error, deviceName, "setStatusFramePeriod", printErrorTrace);

            error = baseController.setStatusFramePeriod(
                    StatusFrame.Status_15_FirmwareApiStatus, m_status_15_FirmwareApiStatus, timeout_ms);
            checkError(error, deviceName, "setStatusFramePeriod", printErrorTrace);

            error = baseController.setStatusFramePeriod(
                    StatusFrame.Status_17_Targets1, m_status_17_Targets1, timeout_ms);
            checkError(error, deviceName, "setStatusFramePeriod", printErrorTrace);

            // // Control Frames
            // error = baseController.setControlFramePeriod(ControlFrame.Control_3_General,
            // m_control_3_General);
            // checkError(error, deviceName, "setControlFramePeriod", printErrorTrace);

            // // TODO: Uncomment this once we know what it actually does
            // error = baseController.setControlFramePeriod(ControlFrame.Control_4_Advanced,
            // m_control_4_Advanced);
            // checkError(error, deviceName, "setControlFramePeriod", printErrorTrace);

            // error = baseController.setControlFramePeriod(ControlFrame.Control_6_MotProfAddTrajPoint,
            //         m_control_6_MotProfAddTrajPoint);
            // checkError(error, deviceName, "setControlFramePeriod", printErrorTrace);
        }
    }
}
