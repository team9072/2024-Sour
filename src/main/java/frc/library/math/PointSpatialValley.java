/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library.math;

import java.util.List;
import java.util.function.Supplier;
import org.growingstems.math.Vector2dU;
import org.growingstems.measurements.Measurements.Unitless;
import org.growingstems.measurements.Unit;
import org.growingstems.signals.api.SignalModifier;

/*
 * Maps a point in 2d-U space to some unitless gradient, which is mapped into E
 */
public class PointSpatialValley<U extends Unit<U>, E extends Unit<E>>
        implements SpatialValley<U, Unitless, E> {

    // A point in U space, with a scalar magnitude and "radius" of influence
    // Radius influences the distance that the "halfway height" lies away from the center

    // Radius and scale are both in U-space, since a gradient on a function F: U^2 -> U should
    // be unitless. This benifiets the type system, as well as giving a useful geometric
    // interpretation for what the "height map" looks like in physical space
    public static class ValleyPoint<U extends Unit<U>> {
        public Vector2dU<U> m_point;
        public U m_radius;
        public U m_scale;

        public ValleyPoint(Vector2dU<U> point, U radius, U scale) {
            m_point = point;
            m_radius = radius;
            m_scale = scale;
        }
    }

    private SignalModifier<Vector2dU<Unitless>, Vector2dU<E>> m_vectorMapper;
    private Supplier<List<ValleyPoint<U>>> m_pointSupplier;

    /*
     * Constructs with a dynamic point supplier
     */
    public PointSpatialValley(
            Supplier<List<ValleyPoint<U>>> points,
            SignalModifier<Vector2dU<Unitless>, Vector2dU<E>> vectorMapper) {
        m_vectorMapper = vectorMapper;
        m_pointSupplier = points;
    }

    /*
     * Constructs with a static array of points
     */
    public PointSpatialValley(
            SignalModifier<Vector2dU<Unitless>, Vector2dU<E>> vectorMapper, List<ValleyPoint<U>> points) {
        m_vectorMapper = vectorMapper;
        // Maybe this can be nicer
        m_pointSupplier = () -> points;
    }

    @Override
    public Vector2dU<E> vectorMap(Vector2dU<Unitless> in) {
        return m_vectorMapper.update(in);
    }

    // Only difference between these two is the multiplication of xdiff vs ydiff at the end
    // Kinda yucky but whatever
    @Override
    public Unitless xPartial(Vector2dU<U> in) {
        return m_pointSupplier.get().stream()
                .map((pt) -> {
                    // X partial derivative formula
                    // -(2k/R^2) * (1 / (1 + (xdiff/R)^2 + (ydiff/R)^2) ) * xdiff
                    var xdiff_over_r = in.getX().sub(pt.m_point.getX()).div(pt.m_radius);
                    var ydiff_over_r = in.getY().sub(pt.m_point.getY()).div(pt.m_radius);

                    // 1 + xdiff^2 + ydiff^2
                    var denom = ydiff_over_r
                            .mul(ydiff_over_r)
                            .add(xdiff_over_r.mul(xdiff_over_r))
                            .add(Unitless.none(1.0));

                    // -2k/R
                    var coeffs = pt.m_scale.neg().mul(2.0).div(pt.m_radius);

                    return coeffs.mul(xdiff_over_r).div(denom.mul(denom));
                })
                .reduce(Unitless.ZERO, (a, b) -> a.add(b));
    }

    @Override
    public Unitless yPartial(Vector2dU<U> in) {
        return m_pointSupplier.get().stream()
                .map((pt) -> {
                    // Y partial derivative formula
                    // -(2k/R^2) * (1 / (1 + (xdiff/R)^2 + (ydiff/R)^2) ) * ydiff
                    var xdiff_over_r = in.getX().sub(pt.m_point.getX()).div(pt.m_radius);
                    var ydiff_over_r = in.getY().sub(pt.m_point.getY()).div(pt.m_radius);

                    // 1 + xdiff^2 + ydiff^2
                    var denom = ydiff_over_r
                            .mul(ydiff_over_r)
                            .add(xdiff_over_r.mul(xdiff_over_r))
                            .add(Unitless.none(1.0));

                    // -2k/R
                    var coeffs = pt.m_scale.neg().mul(2.0).div(pt.m_radius);

                    return coeffs.mul(ydiff_over_r).div(denom.mul(denom));
                })
                .reduce(Unitless.ZERO, (a, b) -> a.add(b));
    }
}
