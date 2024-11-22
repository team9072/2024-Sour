/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library.control;

import org.growingstems.math.RangeU;
import org.growingstems.measurements.Unit;
import org.growingstems.signals.api.SignalModifier;

// DO NOT COPY TO GSTEMS-COMMON
// GSTEMS-COMMON commit 45b82714 makes a unit-based version of this class!

/**
 * Provides a symmetric scaling deadzone for numeric values. <br>
 * <br>
 * The deadzone takes a range of numbers (refered to as the operating range) and provides a subrange
 * (refered to as the deadzone) where outputs are no longer based upon the input value. The input
 * value is modified in the following ways:
 *
 * <ul>
 *   <li>input value inside deadzone: the output value will be a constant value. This value is
 *       configurable
 *   <li>input value outside deadzone but inside of the operating range: the ouput is scaled and
 *       returned. The scale is calculated based on the output range and the operating range. This
 *       allows a full range of returns even though the range of allowed inputs is a subset of the
 *       full operating range.
 * </ul>
 *
 * <p>Note: While it is not technically invalid to provide inputs that are outside of the specified
 * operating range, any output calculated based on those inputs will continue to have scaling
 * applied to them.
 */
public class SymmetricDeadzoneU<U extends Unit<U>> implements SignalModifier<U, U> {
    private final U m_deadzone;
    private final U m_centerValue;
    private final U m_radius;
    private U m_deadOutput;

    /**
     * Construct a SymmetricDeadzoneU
     *
     * @param centerValue value at the center of the range the deadzone operates on
     * @param radius distance between the center and endpoints of the operating range
     * @param deadzone distance between the center of the operating range and the endpoints of the
     *     deadzone
     * @param deadOutput value to output when the input is inside of the deadzone range
     */
    public SymmetricDeadzoneU(U centerValue, U radius, U deadzone, U deadOutput) {
        m_centerValue = centerValue;
        m_radius = radius;
        m_deadzone = deadzone;
        m_deadOutput = deadOutput;
    }

    /**
     * Construct a SymmetricDeadzoneU defaults deadOuptut to be the same as centerValue
     *
     * @param centerValue value at the center of the range the deadzone operates on
     * @param radius distance between the center and endpoints of the operating range
     * @param deadzone distance between the center of the operating range and the endpoints of the
     *     deadzone
     */
    public SymmetricDeadzoneU(U centerValue, U radius, U deadzone) {
        this(centerValue, radius, deadzone, centerValue);
    }

    /**
     * Construct a SymmetricDeadzoneU
     *
     * @param range Endpoints of the operating range of the deadzone
     * @param deadzone distance between the center of the operating range and the endpoints of the
     *     deadzone
     * @param deadOutput value to output when the input is inside of the deadzone range
     */
    public SymmetricDeadzoneU(RangeU<U> range, U deadzone, U deadOutput) {
        this(
                range.getHigh().add(range.getLow()).div(2.0),
                range.getWidth().div(2.0),
                deadzone,
                deadOutput);
    }

    /**
     * Construct a SymmetricDeadzoneU defaults deadOutput to the center value of range
     *
     * @param range Endpoints of the operating range of the deadzone
     * @param deadzone distance between the center of the operating range and the endpoints of the
     *     deadzone
     */
    public SymmetricDeadzoneU(RangeU<U> range, U deadzone) {
        this(range.getHigh().add(range.getLow()).div(2.0), range.getWidth().div(2.0), deadzone);
    }

    public U getDeadzone() {
        return m_deadzone;
    }

    public U getCenterValue() {
        return m_centerValue;
    }

    public U getRadius() {
        return m_radius;
    }

    public U getDeadOutput() {
        return m_deadOutput;
    }

    public void setDeadOutput(U newDeadOutput) {
        m_deadOutput = newDeadOutput;
    }

    /**
     * Calculate the new output value based on the deadzone configuration.
     *
     * <p>Note: While it is not technically invalid to provide inputs that are outside of the
     * specified operating range, any output calculated based on those inputs will continue to have
     * scaling applied to them.
     *
     * @param input the input value to the deadzone
     * @return output value after deadzone is applied
     */
    @Override
    public U update(U input) {
        // if value is within deadzone, provide output value
        if (m_centerValue.sub(input).abs().lt(m_deadzone)) {
            return m_deadOutput;
        }

        // operate on one half of the range
        U oldMin = m_centerValue.add(m_deadzone); // 0.0 + 0.0 => 0.0
        U oldMax = m_centerValue.add(m_radius); // 0.0 + 1.0 -> 1.0
        U oldRange = oldMax.sub(oldMin); // 1.0 - 0.0 -> 0.0

        U newMin = m_centerValue; // 0.0
        U newMax = m_centerValue.add(m_radius); // 0.0 + 1.0 -> 1.0
        U newRange = newMax.sub(newMin); // 1.0 - 0.0 -> 1.0

        U inputVal = input;
        if (input.lt(m_centerValue)) {
            inputVal = m_centerValue.add(m_centerValue.sub(inputVal));
        }

        U newVal = inputVal.sub(oldMin).mul(newRange.div(oldRange).asNone()).add(newMin);

        // determine if input was in bottom half of range
        if (input.lt(m_centerValue)) {
            return m_centerValue.sub(newVal.sub(m_centerValue));
        } else {
            return newVal;
        }
    }
}
