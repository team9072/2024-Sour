/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library.algorithms;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.apache.commons.math3.util.Pair;

/**
 * This is an algorithm that can partition a potentially unconnected into fully connected disjointed
 * subgraphs
 *
 * <p>Functionally, this can be used with WPI's command system. The bipartite graph formed is
 * between Commands, and their Requirements. With a set of commands (The set of U) and their linked
 * Subsystem dependencies, this algorithm will split commands into groups based off of which
 * commands need to be run in sequence. Thus, groups that result from this algorithm can be run in
 * parallel safely
 */
public interface BipartiteCollapse<U, V> {
    public static <U, V> Set<Pair<Set<U>, Set<V>>> collapse(List<Pair<U, Set<V>>> linkSets) {
        // Turn the input pairings, and turn it into a dynamic list
        // This will be modified
        var baseList = new ArrayList<>(linkSets.stream()
                .map(in -> new Pair<Set<U>, Set<V>>(
                        new HashSet<U>(Set.of(in.getFirst())), new HashSet<V>(in.getSecond())))
                .toList());

        // We cover each U - V relation
        for (int i = 0; i < baseList.size(); ) {
            // This is the ith U - V relation
            var vals = baseList.get(i);

            // List to mark indexs
            var matchVals = new ArrayList<Pair<Set<U>, Set<V>>>();
            // We scan every relation over the i-th
            for (int j = i + 1; j < baseList.size(); j++) {
                // If the jth V-set is not disjoint, mark it for a merge with the ith
                if (!Collections.disjoint(baseList.get(i).getSecond(), baseList.get(j).getSecond())) {
                    matchVals.add(baseList.get(j));
                }
            }
            // If all remaining sets are disjoint, we can continue to the next disjoint set
            // Effectively restarting the algorithm, we continue until we excape the root list
            // After escape, we have found all components of the bipartite graph between
            // The elements of U and V
            if (matchVals.isEmpty()) {
                i++;
                continue;
            }
            // Get values of the matched indicies
            matchVals.forEach(in -> {
                vals.getFirst().addAll(in.getFirst());
                vals.getSecond().addAll(in.getSecond());
            });
            baseList.removeAll(matchVals);
        }
        return new HashSet<>(baseList);
    }
}
