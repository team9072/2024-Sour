/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.growingstems.measurements.Measurements.Time;

/** Extended by other helper classes/functions to setup CTRE CAN devices. */
public abstract class CtreUtils {
    /**
     * The default timeout used when applying the configurator on a CTRE device.
     */
    public static final Time k_defaultConfiguratorTimeout = Time.seconds(0.5);

    /**
     * Returns the return stop motor control request that matches the given neutral mode.
     * @param neutralMode The neutral mode to match
     * @return The corresponding stopping control request that matches the given neutral mode
     */
    public static final ControlRequest getControlRequest(NeutralModeValue neutralMode) {
        return switch (neutralMode) {
            case Coast -> new CoastOut();
            case Brake -> new StaticBrake();
        };
    }
}
