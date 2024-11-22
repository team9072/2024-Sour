/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library.math;

import org.growingstems.math.Vector2dU;
import org.growingstems.measurements.Angle;
import org.growingstems.measurements.Unit;
import org.growingstems.signals.api.SignalModifier;

public class VectorUModifiers {
    public static <U extends Unit<U>, V extends Unit<V>>
            SignalModifier<Vector2dU<U>, Vector2dU<V>> xyModifier(
                    SignalModifier<U, V> x, SignalModifier<U, V> y) {
        return v -> new Vector2dU<V>(x.update(v.getX()), y.update(v.getY()));
    }

    public static <U extends Unit<U>, V extends Unit<V>>
            SignalModifier<Vector2dU<U>, Vector2dU<V>> magAngleModifier(
                    SignalModifier<U, V> mag, SignalModifier<Angle, Angle> angle) {
        return v -> Vector2dU.fromPolar(mag.update(v.getMagnitude()), angle.update(v.getAngle()));
    }

    public static <U extends Unit<U>, V extends Unit<V>>
            SignalModifier<Vector2dU<U>, Vector2dU<V>> magModifier(SignalModifier<U, V> mag) {
        return magAngleModifier(mag, a -> a);
    }
}
