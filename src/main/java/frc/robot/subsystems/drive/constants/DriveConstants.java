/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot.subsystems.drive.constants;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import java.util.List;

public abstract class DriveConstants {
    protected final SwerveModuleConstants m_frontRight;
    protected final SwerveModuleConstants m_frontLeft;
    protected final SwerveModuleConstants m_backLeft;
    protected final SwerveModuleConstants m_backRight;
    protected final int m_pigeon2CanId;

    protected final String k_frontRightName = "Front Right";
    protected final String k_frontLeftName = "Front Left";
    protected final String k_backLeftName = "Back Left";
    protected final String k_backRightName = "Back Right";

    protected DriveConstants(int pigeon2CanId) {
        init();
        m_frontRight = createFrontRightConstants();
        m_frontLeft = createFrontLeftConstants();
        m_backLeft = createBackLeftConstants();
        m_backRight = createBackRightConstants();
        m_pigeon2CanId = pigeon2CanId;
    }

    protected abstract void init();

    protected abstract SwerveModuleConstants createFrontRightConstants();

    protected abstract SwerveModuleConstants createFrontLeftConstants();

    protected abstract SwerveModuleConstants createBackLeftConstants();

    protected abstract SwerveModuleConstants createBackRightConstants();

    public int getPigeon2CanId() {
        return m_pigeon2CanId;
    }

    public SwerveModuleConstants getFrontRightConstants() {
        return m_frontRight;
    }

    public SwerveModuleConstants getFrontLeftConstants() {
        return m_frontLeft;
    }

    public SwerveModuleConstants getBackLeftConstants() {
        return m_backLeft;
    }

    public SwerveModuleConstants getBackRightConstants() {
        return m_backRight;
    }

    public List<SwerveModuleConstants> asList() {
        return List.of(
                getFrontRightConstants(),
                getFrontLeftConstants(),
                getBackLeftConstants(),
                getBackRightConstants());
    }
}
