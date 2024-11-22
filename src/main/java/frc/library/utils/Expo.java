/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library.utils;

import org.growingstems.math.Range;
import org.growingstems.signals.api.SignalModifier;

/**
 * Expo is based off of expo settings commonly found on RC remote controllers. Expo is meant to
 * remap a linear control into an exponential based control to give more control at lower speeds
 * while still providing full speed when the joystick is fully deflected.
 *
 * <p>See math <a href=
 * "https://github.com/ArduPilot/ardupilot/blob/ed0921a4186087c5b2ac13187b2f4af2cc9c33ec/libraries/AP_Math/AP_Math.cpp#L123">here</a>
 */
public class Expo implements SignalModifier<Double, Double> {
    private double m_adjustment;

    /** @param adjustment [-1.0, -1.0] */
    public Expo(double adjustment) {
        setAdjustment(adjustment);
    }

    public void setAdjustment(double adjustment) {
        m_adjustment = Range.coerceValue(adjustment, -1.0, 1.0);
    }

    /** Input value is internally coerced to [-1.0, 1.0]. */
    @Override
    public Double update(Double in) {
        var inCoerced = Range.coerceValue(in, -1.0, 1.0);

        if (m_adjustment > 0.0) {
            return (1.0 - m_adjustment) * inCoerced + m_adjustment * inCoerced * inCoerced * inCoerced;
        } else {
            double offsetX = 0.0;
            double offsetY = 0.0;
            double adjustment = m_adjustment * -1.0;

            if (inCoerced > 0.0) {
                offsetX = -1.0;
                offsetY = 1.0;
            } else {
                offsetX = 1.0;
                offsetY = -1.0;
            }

            inCoerced += offsetX;
            // This could possibly be simplified, but I'm not smart enough to realize how to do that.
            return ((1.0 - adjustment) * inCoerced + adjustment * inCoerced * inCoerced * inCoerced)
                    + offsetY;
        }
    }
}
