/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library.swerve;

import frc.library.actuators.PositionActuator;
import frc.library.actuators.VelocityActuator;
import frc.library.utils.Expo;
import org.growingstems.math.Vector2d;
import org.growingstems.math.Vector2dU;
import org.growingstems.measurements.Angle;
import org.growingstems.measurements.Measurements.Length;
import org.growingstems.measurements.Measurements.Velocity;

/**
 * Represents a swerve module, which is a physical module on a robot that contains a single wheel
 * that can be both powered to create lateral movement along the ground and steered to control the
 * direction of the lateral motion.
 *
 * <p>It is important that the swerve module can be steered using closed loop control utilizing a
 * sensor to sense its steer angle.
 *
 * <p>It is optional, but also very helpful that the wheel itself has a sensor in order to measure
 * the wheel's rotation. This is utilized to predict the individual modules position along the
 * ground.
 */
public class SwerveModule {
    private final String m_name;
    private final Vector2dU<Length> m_modulePosition;

    protected final VelocityActuator<Length, Velocity> m_drive;
    protected final PositionActuator<Angle> m_steer;

    protected Length m_prevDrivePosition = Length.ZERO;
    protected Length m_currentDrivePosition = Length.ZERO;
    protected Angle m_prevSteerAngle = Angle.ZERO;
    protected Angle m_currentSteerAngle = Angle.ZERO;

    protected static final double k_powerDeadband = 0.001;
    protected static final Velocity k_velocityDeadband = Velocity.inchesPerSecond(0.1);

    protected final Expo m_cosineExpo;

    /**
     * Constructs a Swerve Module with a position relative to the drivetrain's origin.
     *
     * @param name The name of the module
     * @param modulePos Position of the swerve module relative to the drivetrain's origin.
     * @param moduleDrive Actuator that represents the drive component of a swerve module.
     * @param moduleSteer Actuator that represents the steer component of a swerve module.
     */
    public SwerveModule(
            String name,
            Vector2dU<Length> modulePos,
            VelocityActuator<Length, Velocity> moduleDrive,
            PositionActuator<Angle> moduleSteer) {
        m_name = name;
        m_modulePosition = modulePos;
        m_drive = moduleDrive;
        m_steer = moduleSteer;
        m_cosineExpo = new Expo(1.0);
    }

    public String getName() {
        return m_name;
    }

    /**
     * Commands the Swerve Module to stop all drive power and holds the the steering angle at the last
     * commanded position.
     */
    public void stop() {
        m_drive.stop();
        m_steer.stop();
    }

    public void setVelocityClosedLoop(Vector2dU<Velocity> vector) {
        if (vector.getMagnitude().gt(k_velocityDeadband)) {
            if (setSteerAngle(vector.getAngle(), true)) {
                m_drive.setVelocityClosedLoop(vector.getMagnitude());
            } else {
                m_drive.setVelocityClosedLoop(vector.getMagnitude().neg());
            }
        } else {
            m_drive.stop();
            m_steer.stop();
        }
    }

    public void setOpenLoop(Vector2d vector) {
        if (vector.getMagnitude() > k_powerDeadband) {
            var error = m_steer.getPositionError();
            double drivePower = vector.getMagnitude();
            if (setSteerAngle(vector.getAngle(), true)) {
                drivePower = -drivePower;
            }
            m_drive.setOpenLoop(drivePower * m_cosineExpo.update(error.cos()));
        } else {
            m_drive.stop();
            m_steer.stop();
        }
    }

    public void setOpenLoop(double power, Angle steerAngle) {
        setSteerAngle(steerAngle, true);
        m_drive.setOpenLoop(power);
    }

    public void setBothOpenLoop(double drivePower, double steerPower) {
        m_drive.setOpenLoop(drivePower);
        m_steer.setOpenLoop(steerPower);
    }

    public Vector2dU<Length> getModuleLocation() {
        return m_modulePosition;
    }

    public VelocityActuator<Length, Velocity> getDriveActuator() {
        return m_drive;
    }

    public PositionActuator<Angle> getSteerActuator() {
        return m_steer;
    }

    public Vector2dU<Length> getForwardKinematics() {
        m_prevDrivePosition = m_currentDrivePosition;
        m_prevSteerAngle = m_currentSteerAngle;

        m_currentDrivePosition = getDriveRelativePosition();
        m_currentSteerAngle = getSteerAngle();

        Vector2dU<Length> vector =
                new Vector2dU<>(m_currentDrivePosition.sub(m_prevDrivePosition), Length.ZERO);
        return vector.rotate(m_currentSteerAngle);
    }

    public Vector2dU<Velocity> getVelocityVector() {
        return Vector2dU.fromPolar(getDriveVelocity(), getSteerAngle());
    }

    /**
     * Gives a relative distance of the drive actuator. This distance is relative to wherever the
     * drive actuator was last zero'd at, which in most cases is when it was powered on.
     *
     * @return relative distance from when the drive actuator was last zero'd.
     */
    public Length getDriveRelativePosition() {
        return m_drive.getPosition();
    }

    public Velocity getDriveVelocity() {
        return m_drive.getVelocity();
    }

    public Angle getSteerAngle() {
        return m_steer.getPosition();
    }

    public Angle getSteerAngleClosedLoopError() {
        return m_steer.getPositionError();
    }

    /**
     * Set's the angle of driver wheel towards the given direction. 0.0 is pointing forward in the +X
     * direction, positive values are counter-clockwise.
     *
     * @param steerGoal Goal to steer the drive wheel's direction. [-Pi, Pi)
     * @param reverseOptimize Reverses the drive wheel instead of rotating 180 degrees.
     * @return True if the drive wheel's direction should be inverted.
     */
    protected boolean setSteerAngle(Angle steerGoal, boolean reverseOptimize) {
        boolean invertMotor = false;
        Angle currentAngle = getSteerAngle();

        Angle amountToTurn = steerGoal.difference(currentAngle);
        Angle positionGoal = currentAngle.add(amountToTurn);
        if (reverseOptimize) {
            if (amountToTurn.abs().gt(Angle.PI_BY_TWO)) {
                // Optimized logic to reduce approaching infinty.
                if (amountToTurn.gt(Angle.ZERO)) {
                    positionGoal = positionGoal.sub(Angle.PI);
                } else {
                    positionGoal = positionGoal.add(Angle.PI);
                }
                invertMotor = true;
            } else {
                invertMotor = false;
            }
        } else {
            invertMotor = false;
        }

        m_steer.setPositionClosedLoop(positionGoal);

        return invertMotor;
    }

    /**
     * Set's the Minimum power that will be applied to the drive wheel. Any value lower will be
     * considered zero power and the heading of the module will be maintained.<br>
     * <br>
     * This helps prevent the wheel from spinning wildly at lower vector magnitudes.
     *
     * @param neutralDeadband Neutral deadband setting.
     */
    public void setDriveNeutralDeadband(double neutralDeadband) {
        m_drive.setNeutralDeadband(neutralDeadband);
    }

    /**
     * Gets the current deadband setting used to reduce the module's drive power to zero.
     *
     * @return Neutral deadband value.
     */
    public double getDriveNeutralDeadband() {
        return m_drive.getNeutralDeadband();
    }

    public void calibrateSteer(Angle physicalAngle) {
        m_steer.calibrateOffset(physicalAngle);
    }
}
