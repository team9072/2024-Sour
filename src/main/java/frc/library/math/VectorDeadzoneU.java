/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library.math;

import frc.library.control.SymmetricDeadzoneU;
import org.growingstems.math.Vector2dU;
import org.growingstems.measurements.Unit;
import org.growingstems.signals.api.SignalModifier;

public class VectorDeadzoneU<U extends Unit<U>>
        implements SignalModifier<Vector2dU<U>, Vector2dU<U>> {
    private final SignalModifier<Vector2dU<U>, Vector2dU<U>> m_modifier;
    private final Vector2dU<U> m_center;

    public enum Type {
        CIRCULAR,
        SQUARE
    }

    public VectorDeadzoneU(Vector2dU<U> centerValue, U radius, U deadzone, U deadOutput, Type type) {
        m_center = centerValue;

        var zero = deadzone.sub(deadzone);

        var deadzoneModifier = new SymmetricDeadzoneU<>(zero, radius, deadzone, deadOutput);
        m_modifier = switch (type) {
            case CIRCULAR -> VectorUModifiers.magModifier(deadzoneModifier);
            case SQUARE -> VectorUModifiers.xyModifier(deadzoneModifier, deadzoneModifier);};
    }

    public VectorDeadzoneU(Vector2dU<U> centerValue, U radius, U deadzone, U deadOutput) {
        this(centerValue, radius, deadzone, deadOutput, Type.CIRCULAR);
    }

    @Override
    public Vector2dU<U> update(Vector2dU<U> in) {
        return m_modifier.update(in.sub(m_center)).add(m_center);
    }
}
