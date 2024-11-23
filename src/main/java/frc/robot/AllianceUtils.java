/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldDimensions;
import java.util.Optional;
import org.growingstems.math.Pose2dU;
import org.growingstems.math.Vector2dU;
import org.growingstems.measurements.Angle;
import org.growingstems.measurements.Measurements.Length;

public class AllianceUtils {
    public static boolean isRed() {
        return DriverStation.getAlliance().equals(Optional.of(Alliance.Red));
    }

    public static boolean canGetAlliance() {
        return DriverStation.getAlliance().isPresent();
    }

    public static Vector2dU<Length> transform(Vector2dU<Length> position) {
        return new Vector2dU<Length>(
                FieldDimensions.k_fieldLength.sub(position.getX()), position.getY());
    }

    public static Angle transform(Angle heading) {
        return Angle.PI.difference(heading);
    }

    public static Pose2dU<Length> transform(Pose2dU<Length> pose) {
        return new Pose2dU<Length>(transform(pose.getVector()), transform(pose.getRotation()));
    }
}
