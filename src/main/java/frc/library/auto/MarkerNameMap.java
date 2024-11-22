/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.HashMap;
import java.util.function.Supplier;

public class MarkerNameMap {
    private static final HashMap<String, Supplier<Command>> m_nameCommands = new HashMap<>();

    public static void register(String name, Supplier<Command> command) {
        m_nameCommands.put(name, command);
    }

    public static boolean hasCommand(String name) {
        return m_nameCommands.containsKey(name);
    }

    public static Supplier<Command> getCommand(String name) {
        try {
            // Wrap it so we can reuse the command object in the map if needed
            return m_nameCommands.get(name);
        } catch (Exception e) {
            System.out.println("WARNING: Attempted to parse an unset named command: " + name);
            System.out.println(e);
        }
        return () -> new InstantCommand();
    }
}
