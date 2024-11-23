/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library.math;

import org.growingstems.math.Vector2dU;
import org.growingstems.measurements.Angle;
import org.growingstems.measurements.Measurements.Length;
import org.growingstems.measurements.Measurements.SpatialFrequency;
import org.growingstems.measurements.Measurements.Unitless;
import org.growingstems.measurements.Unit;
import org.growingstems.signals.api.SignalModifier;

/*
 * Maps a length in 2d space to distance from a line (Length -> Length), and Length is mapped to
 * some parameter E
 */
public class LinearSpatialValley<E extends Unit<E>> implements SpatialValley<Length, Unitless, E> {

    // ax + by = c ; linear form
    private SpatialFrequency m_a;
    private SpatialFrequency m_b;
    private Unitless m_c;

    // Spatial derivative function
    private SignalModifier<Vector2dU<Unitless>, Vector2dU<E>> m_vecMapper;

    private SignalModifier<Length, Double> m_distanceFunctionDerivative;

    /*
     * Supplies the linear form in ax + by = c
     */
    public LinearSpatialValley(
            SpatialFrequency a,
            SpatialFrequency b,
            Unitless c,
            SignalModifier<Length, Double> distanceFunctionDerivative,
            SignalModifier<Vector2dU<Unitless>, Vector2dU<E>> vectorMap) {
        m_a = a;
        m_b = b;
        m_c = c;

        m_vecMapper = vectorMap;
    }

    /*
     * Makes a line between two points
     */
    public LinearSpatialValley(
            Vector2dU<Length> p1,
            Vector2dU<Length> p2,
            SignalModifier<Length, Double> distanceFunctionDerivative,
            SignalModifier<Vector2dU<Unitless>, Vector2dU<E>> vectorMapper) {

        // C = p1.x * p2.y - p1.y * p2.x; The determinant of the [p1, p2] matrix
        var c = p1.getX().mul(p2.getY()).sub(p1.getY().mul(p2.getX()));

        // B = p2.x - p1.x
        var b = p2.getX().sub(p1.getX());

        // A = p1.y - p2.y
        var a = p1.getY().sub(p2.getY());

        m_a = SpatialFrequency.perMeter(a.asMeters());
        m_b = SpatialFrequency.perMeter(b.asMeters());
        m_c = Unitless.none(c.asMetersSquared());

        m_vecMapper = vectorMapper;
        m_distanceFunctionDerivative = distanceFunctionDerivative;
    }

    /*
     * Makes a line through a point with a given angle.
     * Angle zero should be defined with respect to the provided coordinate system
     */
    public LinearSpatialValley(
            Vector2dU<Length> p1,
            Angle angle,
            SignalModifier<Length, Double> distanceFunctionDerivative,
            SignalModifier<Vector2dU<Unitless>, Vector2dU<E>> vectorMapper) {
        var p2 =
                (new Vector2dU<Length>(Length.meters(1.0), Length.ZERO)).rotate(angle).add(p1);
        var c = p1.getX().mul(p2.getY()).sub(p1.getY().mul(p2.getX()));
        var b = p2.getX().sub(p1.getX());
        var a = p1.getY().sub(p2.getY());

        m_a = SpatialFrequency.perMeter(a.asMeters());
        m_b = SpatialFrequency.perMeter(b.asMeters());
        m_c = Unitless.none(c.asMetersSquared());

        m_vecMapper = vectorMapper;
        m_distanceFunctionDerivative = distanceFunctionDerivative;
    }

    private Length distance(Vector2dU<Length> pos) {
        var linComponent = pos.getX().mul(m_a).add(pos.getY().mul(m_b)).add(m_c);

        var aSq = m_a.asPerMeter() * m_a.asPerMeter();
        var bSq = m_b.asPerMeter() * m_b.asPerMeter();
        var bottom = Math.sqrt(aSq + bSq);

        return linComponent.div(SpatialFrequency.perMeter(bottom));
    }

    @Override
    public Vector2dU<E> vectorMap(Vector2dU<Unitless> in) {
        return m_vecMapper.update(in);
    }

    @Override
    public Unitless xPartial(Vector2dU<Length> in) {
        var aSq = m_a.asPerMeter() * m_a.asPerMeter();
        var bSq = m_b.asPerMeter() * m_b.asPerMeter();
        var bottom = SpatialFrequency.perMeter(Math.sqrt(aSq + bSq));

        return m_a.div(bottom).mul(m_distanceFunctionDerivative.update(distance(in)));
    }

    @Override
    public Unitless yPartial(Vector2dU<Length> in) {
        var aSq = m_a.asPerMeter() * m_a.asPerMeter();
        var bSq = m_b.asPerMeter() * m_b.asPerMeter();
        var bottom = SpatialFrequency.perMeter(Math.sqrt(aSq + bSq));

        return m_b.div(bottom).mul(m_distanceFunctionDerivative.update(distance(in)));
    }

    /*
     * Class containing various derivative presets
     */
    public static final class LinearValleyUtils {
        // Maps distance to a quadratic parabola valley, which results in a linearly scaling gradient
        // as you move away from the line
        public static final SignalModifier<Length, Double> k_quadraticValleyDerivative_meters =
                in -> 2 * in.asMeters();

        // Maps distance to a "triangular" valley, resulting in a constant gradient towards the line
        // from both sides (imagine two slopes meeting at a bottom point)
        public static final SignalModifier<Length, Double> k_triangleValleyDerivative =
                // Normalize direction; (V / |V|)
                in -> in.div(in.abs()).asNone();
    }
}
