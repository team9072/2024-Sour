/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library.actuators;

import java.util.Optional;
import org.growingstems.measurements.Measurements.Current;
import org.growingstems.measurements.Measurements.Temperature;
import org.growingstems.measurements.Measurements.Voltage;

public interface MotorActuator {

    void setNeutralDeadband(double neutralDeadband);

    double getNeutralDeadband();

    void stop();

    void setOpenLoop(double power);

    double getCurrentPower();

    boolean enableCurrentLimiting(boolean enable);

    Optional<Temperature> getTemperature();

    Voltage getSupplyVoltage();

    Voltage getOutputVoltage();

    Optional<Current> getStatorCurrent();

    Optional<Current> getSupplyCurrent();

    boolean getForwardLimitSwitch();

    boolean getReverseLimitSwitch();

    void disable();

    void enable();

    boolean isDisabled();

    void setCoast();

    void setBrake();
}
