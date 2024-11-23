/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library.math;

import org.growingstems.control.SymmetricDeadzone;
import org.growingstems.math.Vector2d;
import org.growingstems.signals.api.SignalModifier;

public class VectorDeadzone implements SignalModifier<Vector2d, Vector2d> {
    private final SignalModifier<Vector2d, Vector2d> m_modifier;
    private final Vector2d m_center;

    public enum Type {
        CIRCULAR,
        SQUARE
    }

    public VectorDeadzone(
            Vector2d centerValue, double radius, double deadzone, double deadOutput, Type type) {
        m_center = centerValue;

        var applyDeadzone = new SymmetricDeadzone(0.0, radius, deadzone, deadOutput);
        m_modifier = switch (type) {
            case CIRCULAR -> VectorModifiers.magModifier(applyDeadzone);
            case SQUARE -> VectorModifiers.xyModifier(applyDeadzone, applyDeadzone);};
    }

    public VectorDeadzone(Vector2d centerValue, double radius, double deadzone, double deadOutput) {
        this(centerValue, radius, deadzone, deadOutput, Type.CIRCULAR);
    }

    public VectorDeadzone(double deadzone, Type type) {
        this(new Vector2d(), 1.0, deadzone, 0.0, Type.CIRCULAR);
    }

    public VectorDeadzone(double deadzone) {
        this(deadzone, Type.CIRCULAR);
    }

    @Override
    public Vector2d update(Vector2d in) {
        return m_modifier.update(in.sub(m_center)).add(m_center);
    }
}
