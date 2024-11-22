/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library;

import org.growingstems.measurements.Angle;
import org.growingstems.signals.api.SignalModifier;

public class SnapThirds implements SignalModifier<Angle, Angle> {
    private static final int k_segments = 3;

    @Override
    public Angle update(Angle angle) {
        return update(angle, Angle.ZERO);
    }

    public Angle update(Angle angle, Angle offset) {
        // First we scale up the angle so that -60, 60 is -360, 360.
        // This is important as all our Angle classes generally work on a -pi, pi range
        var angleScaled = angle.add(offset).mul((double) k_segments);

        // Shift the snapped angle back to the original scaled angle's scope
        var shiftedAngle = Angle.ZERO.shiftWrappedAngleScope(angleScaled);

        // Return the scaled snapped angle back to the original scaling to get the final snapped angle
        return shiftedAngle.div((double) k_segments).sub(offset);
    }
}
