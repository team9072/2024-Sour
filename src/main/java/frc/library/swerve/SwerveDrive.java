/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library.swerve;

import java.util.List;
import java.util.stream.Collectors;
import org.growingstems.math.Vector2d;
import org.growingstems.math.Vector2dU;
import org.growingstems.measurements.Angle;
import org.growingstems.measurements.Measurements.Length;
import org.growingstems.measurements.Measurements.Velocity;

/** Swerve Drive is used to implement a swerve drive from any number of {@link SwerveModule}. */
public class SwerveDrive {
    private static final Length k_maxModuleOffset = Length.inches(30.0);

    private final List<SwerveModule> m_modules;

    /**
     * Creates a SwerveDrive with the given modules.
     *
     * @param modules A list of the modules that make up the swerve drive.
     */
    public SwerveDrive(List<SwerveModule> modules) {
        m_modules = modules;
    }

    /** Commands all the Swerve Modules to stop. */
    public void stop() {
        for (SwerveModule module : m_modules) {
            module.stop();
        }
    }

    public List<SwerveModule> getModules() {
        return m_modules;
    }

    /**
     * Sets the Swerve Drive into open loop control mode. This is the most basic method for
     * controlling the robot. All inputs are normalized from [-1.0, 1.0].
     *
     * @param translation The direction that the robot will be commanded to translate towards. +X is
     *     forward for the robot and +Y is left. This should be a normalized vector.
     * @param rotation The amount of rotation power to be applied to the swerve drive. [-1.0, 1.0]
     */
    public void setOpenLoop(Vector2d translation, double rotation) {
        setModulesOpenLoop(inverseKinematics(translation, rotation));
    }

    /**
     * Given a translation vector, and a rotation power, this method calculates the vectors to be used
     * by the swerve modules given at construction of SwerveDrive. The rotation component will be
     * applied relative to the origin of the drive train. These vectors are normalized.
     *
     * @param translation The translation component used for translating the drivetrain. This should
     *     be a normalized vector.
     * @param rotation The rotation component used for rotating the drivetrain. [-1.0, 1.0]
     * @return List of vectors to be used by the swerve modules. This list is ordered the same as
     *     m_modules's list.
     */
    private List<Vector2d> inverseKinematics(Vector2d translation, double rotation) {
        return inverseKinematics(
                translation, rotation, new Vector2dU<Length>(Length.ZERO, Length.ZERO));
    }

    /**
     * Given a translation vector, and a rotation power, this method calculates the vectors to be used
     * by the swerve modules given at construction of SwerveDrive. These vectors are normalized.
     *
     * @param translation The translation component used for translating the drivetrain. Magnitude
     *     [0.0, 1.0]
     * @param rotation The rotation component used for rotating the drivetrain. [-1.0, 1.0]
     * @param centerOfRotation The point that the drivetrain will rotate about relative to the origin
     *     of the drivetrain.
     * @return List of vectors to be used by the swerve modules. This list is ordered the same as
     *     m_modules's list.
     */
    private List<Vector2d> inverseKinematics(
            Vector2d translation, double rotation, Vector2dU<Length> centerOfRotation) {
        var rotationVectors = m_modules.stream()
                .map(SwerveModule::getModuleLocation)
                .map(modulePos -> centerOfRotation.sub(modulePos))
                .collect(Collectors.toList());

        var vecs = Vector2dU.normalizeGroup(rotationVectors).stream().map(vec -> {
            if (Double.isNaN(vec.getX()) || Double.isNaN(vec.getY())) {
                return new Vector2d();
            } else {
                return vec.rotate(Angle.PI_BY_TWO.neg()).mul(rotation);
            }
        });

        if (Math.abs(translation.getX()) > 1.0e-9 || Math.abs(translation.getY()) > 1.0e-9) {
            vecs = vecs.map(vec -> vec.add(translation));
        }

        var rotationVectorsCollect = vecs.collect(Collectors.toList());

        return Vector2d.normalizeGroup(rotationVectorsCollect, true);
    }

    /**
     * Gets the difference in position since the last time this function was called. This delta in
     * position is robot centric where +X is forwards relative to the module and +Y is left relative
     * to the module. The first time this function is called, it always returns (0.0, 0.0).
     *
     * @return the difference in position since the last time this function was called.
     */
    public Vector2dU<Length> getForwardKinematics() {
        Vector2dU<Length> averageKinematicVector = new Vector2dU<>(Length.ZERO, Length.ZERO);
        int goodModules = 0;
        for (SwerveModule module : m_modules) {
            var forwardsKinematics = module.getForwardKinematics();
            if (forwardsKinematics.getMagnitude().lt(k_maxModuleOffset)) {
                goodModules++;
                averageKinematicVector = averageKinematicVector.add(forwardsKinematics);
            }
        }

        if (goodModules == 0) {
            return new Vector2dU<>(Length.ZERO, Length.ZERO);
        }

        return averageKinematicVector.div(goodModules);
    }

    /**
     * Gets the drive's current velocity as a vector relative to the robot.
     *
     * @return Curent velocity of the drive
     */
    public Vector2dU<Velocity> getVelocityVector() {
        Vector2dU<Velocity> averageVelocityVector = new Vector2dU<>(Velocity.ZERO, Velocity.ZERO);
        for (SwerveModule module : m_modules) {
            var moduleVelocity = module.getVelocityVector();
            averageVelocityVector = averageVelocityVector.add(moduleVelocity);
        }

        return averageVelocityVector.mul(1.0 / (double) m_modules.size());
    }

    private void setModulesOpenLoop(List<Vector2d> moduleVectors) {
        if (moduleVectors.size() != m_modules.size()) {
            return;
        }

        int i = 0;
        for (Vector2d moduleVector : moduleVectors) {
            double power = moduleVector.getMagnitude();
            Angle angle = Angle.atan2(moduleVector.getY(), moduleVector.getX());
            m_modules.get(i).setOpenLoop(new Vector2d(power, angle));
            i++;
        }
    }

    /**
     * Rotates the robot about a fixed location relative to the swerve drives origin at the given
     * power.
     *
     * @param originOfRotation The position relative to <code>SwerveDrive</code>'s origin to rotate
     *     about.
     * @param power The power to be applied for rotating the robot. A positive value is
     *     counter-clockwise. [-1.0, 1.0]
     */
    public void rotateOpenLoop(Vector2dU<Length> originOfRotation, double power) {
        setModulesOpenLoop(inverseKinematics(new Vector2d(), power, originOfRotation));
    }
}
