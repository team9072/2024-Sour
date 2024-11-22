/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library.auto.gschoreo;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.growingstems.measurements.Measurements.Time;

public class GsChorEvent {
    private final Time m_timeStamp;

    // This is a supplier now because each event needs to know how to get a command to run
    // But it cannot just be a Command, since it might get composed somewhere multiple times
    private final Supplier<Command> m_command;

    public GsChorEvent(Time timeStamp, Supplier<Command> command) {
        m_timeStamp = timeStamp;
        m_command = command;
    }

    public Time getTimestamp() {
        return m_timeStamp;
    }

    public Command getCommand() {
        return m_command.get();
    }
}
