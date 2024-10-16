/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library.utils;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import java.util.function.Supplier;
import org.growingstems.measurements.Unit;
import org.growingstems.signals.api.SignalModifier;

public class UnitEntry<U extends Unit<U>> {
    protected static <U extends Unit<U>> SignalModifier<U, Double> getToDouble(
            SignalModifier<Double, U> toUnit) {
        var oneUnit = toUnit.update(1.0);
        return u -> u.div(oneUnit).asNone();
    }

    public static class Builder<U extends Unit<U>> {
        protected final SimpleWidget m_widget;
        protected final U m_defaultValue;
        protected final SignalModifier<Double, U> m_toUnit;

        public Builder(
                ShuffleboardTab tab, String title, U defaultValue, SignalModifier<Double, U> toUnit) {
            m_toUnit = toUnit;
            m_defaultValue = defaultValue;
            m_widget = tab.add(title, getToDouble(m_toUnit).update(defaultValue));
        }

        public Builder<U> withPosition(int columnIndex, int rowIndex) {
            m_widget.withPosition(columnIndex, rowIndex);
            return this;
        }

        public Builder<U> withSize(int width, int height) {
            m_widget.withSize(width, height);
            return this;
        }

        public UnitEntry<U> build() {
            return new UnitEntry<U>(m_widget.getEntry(), m_defaultValue, m_toUnit);
        }
    }

    protected final GenericEntry m_entry;
    protected final U m_defaultValue;
    protected final SignalModifier<Double, U> m_toUnit;
    protected final SignalModifier<U, Double> m_toDouble;

    public UnitEntry(GenericEntry entry, U defaultValue, SignalModifier<Double, U> toUnit) {
        m_entry = entry;
        m_defaultValue = defaultValue;
        m_toUnit = toUnit;
        m_toDouble = getToDouble(m_toUnit);
    }

    public U get() {
        return m_toUnit.update(m_entry.getDouble(m_toDouble.update(m_defaultValue)));
    }

    public Supplier<U> getAsSupplier() {
        return () -> get();
    }

    public void set(U displayValue) {
        m_entry.setDouble(m_toDouble.update(displayValue));
    }
}
