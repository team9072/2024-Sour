/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.library.control.SymmetricDeadzoneU;
import frc.library.math.VectorDeadzoneU;
import frc.library.math.VectorDeadzoneU.Type;
import frc.library.math.VectorUModifiers;
import java.util.function.Supplier;
import org.growingstems.math.RangeU;
import org.growingstems.math.Vector2dU;
import org.growingstems.measurements.Angle;
import org.growingstems.measurements.Measurements.AngularVelocity;
import org.growingstems.measurements.Measurements.Frequency;
import org.growingstems.measurements.Measurements.Unitless;
import org.growingstems.measurements.Measurements.Velocity;
import org.growingstems.signals.api.SignalModifier;

public class TeleopControls {
    private final XboxController m_driver;

    // Driver primary controls
    public final Supplier<Vector2dU<Velocity>> drivePowerSupplier;
    public final Supplier<Vector2dU<Velocity>> drivePowerRobotOrientedSupplier;
    public final Supplier<AngularVelocity> turnPowerSupplier;

    public final Trigger resetImu;
    public final Trigger slowMode;

    public final Trigger runSelectedSysId;

    // These are the correct values
    protected static final int dPadUp = 0;
    protected static final int dPadUR = 45;
    protected static final int dPadRight = 90;
    protected static final int dPadDR = 135;
    protected static final int dPadDown = 180;
    protected static final int dPadDL = 225;
    protected static final int dPadLeft = 270;
    protected static final int dPadUL = 315;

    protected static final Frequency k_maxTranslationAcceleration = Frequency.hertz(100.0);
    public static final Velocity k_defaultMaxTranslationSpeed = Velocity.feetPerSecond(15.5);
    public static final AngularVelocity k_defaultMaxRotationSpeed =
            AngularVelocity.revolutionsPerSecond(1.8);
    public static final Velocity k_defaultSlowModeTranslationSpeed = Velocity.feetPerSecond(4.5);
    public static final AngularVelocity k_defaultSlowModeRotationSpeed =
            AngularVelocity.revolutionsPerSecond(0.5);
    protected static final Unitless k_joystickDeadzone = Unitless.none(0.08);
    protected static final double k_translationExpoSetting = 0.9;
    protected static final double k_rotationExpoSetting = 0.9;

    private final RangeU<Unitless> m_negativeOneToOne =
            new RangeU<>(Unitless.none(-1.0), Unitless.none(1.0));
    private final SignalModifier<Vector2dU<Unitless>, Vector2dU<Unitless>> m_vectorClamp =
            VectorUModifiers.magModifier(m_negativeOneToOne::coerceValue);

    private boolean m_slowModeEnabled = false;

    // Translation Modifiers
    private final SignalModifier<Vector2dU<Unitless>, Vector2dU<Unitless>> m_translationDeadzone =
            new VectorDeadzoneU<>(
                    new Vector2dU<>(Unitless.ZERO, Unitless.ZERO),
                    Unitless.none(1.0),
                    k_joystickDeadzone,
                    Unitless.ZERO,
                    Type.CIRCULAR);

    private final SignalModifier<Vector2dU<Unitless>, Vector2dU<Velocity>> m_translationScale =
            VectorUModifiers.magModifier(in -> in.mul(
                    m_slowModeEnabled ? k_defaultSlowModeTranslationSpeed : k_defaultMaxTranslationSpeed));

    private final SignalModifier<Vector2dU<Velocity>, Vector2dU<Velocity>> m_flipRed = v -> {
        if (AllianceUtils.isRed()) {
            return v.rotate(Angle.PI);
        } else {
            return v;
        }
    };

    private final SignalModifier<Vector2dU<Unitless>, Vector2dU<Velocity>>
            m_translationModifierRobotOriented =
                    m_translationDeadzone.append(m_vectorClamp).append(m_translationScale);

    private final SignalModifier<Vector2dU<Unitless>, Vector2dU<Velocity>> m_translationModifier =
            m_translationModifierRobotOriented.append(m_flipRed);

    // Rotation Modifiers
    private final SymmetricDeadzoneU<Unitless> m_rotationDeadzone =
            new SymmetricDeadzoneU<>(Unitless.ZERO, Unitless.none(1.0), k_joystickDeadzone);
    private final SignalModifier<Unitless, AngularVelocity> m_rotationScale =
            in -> in.mul(m_slowModeEnabled ? k_defaultSlowModeRotationSpeed : k_defaultMaxRotationSpeed);

    private final SignalModifier<Unitless, AngularVelocity> m_rotationModifier =
            m_rotationDeadzone.append(m_rotationScale);

    public TeleopControls() {
        m_driver = new XboxController(0);

        // Drive Triggers
        resetImu = new Trigger(m_driver::getYButton);
        slowMode = new Trigger(() -> m_driver.getLeftTriggerAxis() > 0.5);

        // Drive
        Supplier<Vector2dU<Unitless>> translationInput = () ->
                new Vector2dU<>(Unitless.none(-m_driver.getLeftY()), Unitless.none(-m_driver.getLeftX()));

        drivePowerRobotOrientedSupplier = m_translationModifierRobotOriented.provide(translationInput);
        drivePowerSupplier = m_translationModifier.provide(translationInput);
        Supplier<Unitless> rotationInput = () -> Unitless.none(-m_driver.getRightX());
        turnPowerSupplier = m_rotationModifier.provide(rotationInput);

        runSelectedSysId = new Trigger(m_driver::getAButton);
    }

    public Command getEnableSlowModeICommand() {
        return new InstantCommand(() -> m_slowModeEnabled = true);
    }

    public Command getDisableSlowModeICommand() {
        return new InstantCommand(() -> m_slowModeEnabled = false);
    }
}
