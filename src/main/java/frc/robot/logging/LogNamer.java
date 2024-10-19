/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot.logging;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;
import org.growingstems.frc.util.WpiTimeSource;
import org.growingstems.measurements.Measurements.Time;
import org.growingstems.util.timer.Timer;

public class LogNamer extends Command {
    private static final ZoneId k_utc = ZoneId.of("US/Eastern");
    private static final DateTimeFormatter k_formatter =
            DateTimeFormatter.ofPattern("yyyy-MM-dd_HH-mm-ss").withZone(k_utc);

    private static final Time k_fmsTimeRequired = Time.seconds(1.0);
    private static final Time k_dsTimeRequired = Time.seconds(1.0);

    private final LogBuilder m_builder;
    private boolean m_isFinished = false;
    private boolean m_renamedToDateTime = false;
    private final Timer m_fmsAttachedTimer;
    private final Timer m_dsAttachedTimer;

    public LogNamer(LogBuilder builder) {
        m_builder = builder;
        var ts = new WpiTimeSource();
        m_fmsAttachedTimer = ts.createTimer();
        m_fmsAttachedTimer.start();
        m_dsAttachedTimer = ts.createTimer();
        m_dsAttachedTimer.start();
    }

    @Override
    public void execute() {
        if (m_isFinished) {
            return;
        }

        if (!DriverStation.isFMSAttached()) {
            m_fmsAttachedTimer.reset();
        } else if (m_fmsAttachedTimer.hasElapsed(k_fmsTimeRequired)) {
            DriverStation.MatchType matchType = DriverStation.getMatchType();
            if (matchType != DriverStation.MatchType.None) {
                char matchTypeChar =
                        switch (matchType) {
                            case Practice -> 'P';
                            case Qualification -> 'Q';
                            case Elimination -> 'E';
                            default -> '_';
                        };
                m_builder.renameTo(k_formatter.format(LocalDateTime.now(k_utc))
                        + "_"
                        + DriverStation.getEventName()
                        + "_"
                        + matchTypeChar
                        + DriverStation.getMatchNumber());
                m_isFinished = true;
                return;
            }
        }

        if (!DriverStation.isDSAttached()) {
            m_dsAttachedTimer.reset();
        } else if (!m_renamedToDateTime && m_dsAttachedTimer.hasElapsed(k_dsTimeRequired)) {
            LocalDateTime now = LocalDateTime.now(k_utc);
            if (now.getYear() > 0) {
                m_builder.renameTo(k_formatter.format(now));
                m_renamedToDateTime = true;
            }
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public boolean isFinished() {
        return m_isFinished;
    }
}
