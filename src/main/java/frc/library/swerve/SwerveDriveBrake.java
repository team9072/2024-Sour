/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.library.utils.ConversionUtils;
import org.growingstems.measurements.Angle;

public class SwerveDriveBrake extends SwerveRequest.SwerveDriveBrake {
    public enum Method {
        TREAD_TO_CENTER,
        AXLES_TO_CENTER;

        public static final Method k_default = TREAD_TO_CENTER;
    }

    private final Method m_method;

    public SwerveDriveBrake(Method method) {
        m_method = method;
    }

    public SwerveDriveBrake() {
        this(Method.k_default);
    }

    @Override
    public StatusCode apply(
            SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
        var offset =
                switch (m_method) {
                    case TREAD_TO_CENTER -> Angle.degrees(0.0);
                    case AXLES_TO_CENTER -> Angle.degrees(90.0);
                };

        for (int i = 0; i < modulesToApply.length; ++i) {
            var state = new SwerveModuleState(
                    0, parameters.swervePositions[i].getAngle().plus(ConversionUtils.toWpi(offset)));
            modulesToApply[i].apply(state, DriveRequestType, SteerRequestType);
        }

        return StatusCode.OK;
    }
}
