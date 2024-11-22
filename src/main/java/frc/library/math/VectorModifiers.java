/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library.math;

import java.util.function.Supplier;
import org.growingstems.math.Range;
import org.growingstems.math.Vector2d;
import org.growingstems.math.Vector2dU;
import org.growingstems.measurements.Angle;
import org.growingstems.measurements.Unit;
import org.growingstems.signals.api.SignalModifier;

public class VectorModifiers {
    public static SignalModifier<Vector2d, Vector2d> xyModifier(
            SignalModifier<Double, Double> x, SignalModifier<Double, Double> y) {
        return v -> new Vector2d(x.update(v.getX()), y.update(v.getY()));
    }

    public static SignalModifier<Vector2d, Vector2d> magAngleModifier(
            SignalModifier<Double, Double> mag, SignalModifier<Angle, Angle> angle) {
        return v -> new Vector2d(mag.update(v.getMagnitude()), angle.update(v.getAngle()));
    }

    public static SignalModifier<Vector2d, Vector2d> magModifier(SignalModifier<Double, Double> mag) {
        return magAngleModifier(mag, a -> a);
    }

    public static SignalModifier<Vector2d, Vector2d> additiveInjection(Supplier<Vector2d> field) {
        return v -> v.add(field.get());
    }

    public static SignalModifier<Vector2d, Vector2d> directionalLimiter(
            Supplier<Vector2d> directionField) {
        var negativeRange = new Range(-1.0, 0.0);
        return v -> {
            var offsetVec = directionField.get().normalize(true);
            var scalar = negativeRange.coerceValue(offsetVec.dot(v));
            return v.add(offsetVec.mul(-scalar));
        };
    }

    public static <U extends Unit<U>>
            SignalModifier<Vector2dU<U>, Vector2dU<U>> directionLimiterUnitless(
                    Supplier<Vector2dU<U>> directionField, U scale) {
        var negativeRange = new Range(-1.0, 0.0);
        return v -> {
            var offsetVec = directionField.get().div(scale).toBaseVector2d().normalize(true);
            var scalar = negativeRange.coerceValue(offsetVec.dot(v.toBaseVector2d()));
            return v.add(offsetVec.mul(-scalar).forEachU(in -> scale.mul(in)));
        };
    }
}
