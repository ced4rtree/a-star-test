package frc.robot.overwatch;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Deque;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.overwatch.OverwatchConstants.OverwatchPos;

public final class Graph {
    public enum Node implements OverwatchPos {
        HOME       (-Math.PI/2,   1.2),
        CORAL_PICK (-Math.PI/2,   1.1),
        L2_PREP    (Math.PI/6,    0.2),
        FOO        (-5*Math.PI/6, 0.8),
        BAR        (-1*Math.PI/6,    0.8),
        ;

        // to be amended to during initialization (see static block below)
        // initializing now messes up the jvm since Node isn't a class yet, has to stay null for now
        private EnumSet<Node> safeClockwiseNeighbors;
        private EnumSet<Node> safeCounterClockwiseNeighbors;
        private final double liftHeightMeters;
        private final Rotation2d pivotAngle;

        Node(double pivotAngleRads, double liftHeightMeters) {
            this.pivotAngle = Rotation2d.fromRadians(pivotAngleRads);
            this.liftHeightMeters = liftHeightMeters;
        }

        @Override
        public double liftHeightMeters() {
            return liftHeightMeters;
        }

        @Override
        public Rotation2d pivotAngle() {
            return pivotAngle;
        }

        static {
            // add all neighbors whose edge doesn't intersect the unsafe zone
            for (Node node : Node.values()) {
                node.safeClockwiseNeighbors
                  = getSafeNeighbors(node, RotationalDirection.CLOCKWISE);
                node.safeCounterClockwiseNeighbors
                  = getSafeNeighbors(node, RotationalDirection.COUNTER_CLOCKWISE);
            }
        }

        /**
         * Return all nodes that are safe to travel directly to, given that you
         * are travelling directly from this node.
         */
        public EnumSet<Node> getNeighbors(RotationalDirection dir) {
            return switch (dir) {
                case CLOCKWISE -> safeClockwiseNeighbors;
                case COUNTER_CLOCKWISE -> safeCounterClockwiseNeighbors;
            };
        }

        /**
         * Return the amount of time it takes to travel between this node and
         * the specified node.
         */
        public double travelTime(Node node, RotationalDirection dir) {
            return Graph.travelTime(this, node, dir);
        }

        public String toString() {
            return name() + " " + string();
        }
    }

    /**
     * Data type used for representing a query between the starting and ending
     * nodes on a path. Exists just to make the type of the cache a little
     * easier to read
     */
    public static record PathQuery(OverwatchPos startNode, Node goalNode, RotationalDirection dir) {}

    public enum RotationalDirection {
        CLOCKWISE,
        COUNTER_CLOCKWISE,
        ;
    }

    private static Map<PathQuery, List<OverwatchPos>> aStarCache
        = new HashMap<>();

    /**
     * Return the amount of time it takes to travel between the start node
     * and the goal node.
     */
    public static double travelTime(
        OverwatchPos startNode,
        OverwatchPos goalNode_,
        RotationalDirection dir
    ) {
        OverwatchPos goalNode = repositionNode(startNode, goalNode_, dir);
        double nodeToNodeDist = startNode.distanceFrom(goalNode);
        var initialState = new TrapezoidProfile.State();
        var finalState = new TrapezoidProfile.State(nodeToNodeDist, 0.0);
        OverwatchConstants.MOTION_PROFILE.calculate(0.02, initialState, finalState);

        return OverwatchConstants.MOTION_PROFILE.totalTime();
    }

    /**
     * Returns the gscore for the final node in the provided set
     */
    private static double cumulativeGScore(List<OverwatchPos> nodes, RotationalDirection dir) {
        double gscore = 0.0;

        for (int i = 0; i < nodes.size() - 1; i++) {
            var node = nodes.get(i);
            gscore += travelTime(node, nodes.get(i+1), dir);
        }
        return gscore;
    }

    private static double calculateHScore(
        OverwatchPos from,
        OverwatchPos goalNode,
        RotationalDirection dir
    ) {
        return travelTime(from, goalNode, dir);
    }

    /**
     * Calculate the time-optimal path from this node to the supplied node
     * using A* to traverse the graph
     *
     * The G-score is the cost (time) of travelling directly from one node
     * to another.
     * The H-score is the heuristic (in this case, time from a node to the
     * goal node) associated with a node.
     * The F-score is the sum of the G and H scores, and is used to decide
     * which node to go to.
     * A* will continuously pathfind by travelling to each node with the
     * smallest F-score until it reaches the goal
     */
    public static Optional<List<OverwatchPos>> pathfind(
        OverwatchPos startNode,
        Node goalNode,
        RotationalDirection dir
    ) {
        PathQuery pathQuery = new PathQuery(startNode, goalNode, dir);
        if (aStarCache.containsKey(pathQuery)) {
            return Optional.of(aStarCache.get(pathQuery));
        }

        if (goalNode.equals(startNode)) {
            return Optional.of(List.of());
        }

        var openList = new ArrayList<OverwatchPos>();
        openList.add(startNode);
        var closedList = new ArrayList<OverwatchPos>();
        Map<OverwatchPos, OverwatchPos> cameFrom = new HashMap<>();
        Map<OverwatchPos, Double> gscore = new HashMap<>();
        Map<OverwatchPos, Double> hscore = new HashMap<>();

        gscore.put(startNode, 0.0);
        hscore.put(startNode, calculateHScore(
            startNode,
            goalNode,
            dir
        ));

        OverwatchPos current = startNode;
        while (!openList.isEmpty()) {
            for (OverwatchPos node : openList) {
                Double currentFScore =
                    gscore.getOrDefault(current, Double.MAX_VALUE)
                    + hscore.getOrDefault(current, Double.MAX_VALUE);
                Double otherFScore =
                    gscore.getOrDefault(node, Double.MAX_VALUE)
                    + hscore.getOrDefault(node, Double.MAX_VALUE);
                if (otherFScore < currentFScore) {
                    current = node;
                }
            }

            openList.remove(current);
            closedList.add(current);

            EnumSet<Node> neighbors;
            if (current.getClass() == Node.class) {
                neighbors = ((Node)current).getNeighbors(dir);
            } else {
                neighbors = getSafeNeighbors(current, dir);
            }

            for (OverwatchPos neighbor : neighbors) {
                if (neighbor == goalNode) {
                    cameFrom.put(goalNode, current);
                    var ret = reconstructPath(cameFrom, goalNode);
                    aStarCache.put(pathQuery, ret);
                    return Optional.of(ret);
                }

                if (closedList.contains(neighbor)) {
                    continue;
                }

                var tentativeGScore = cumulativeGScore(reconstructPath(cameFrom, neighbor), dir);
                var tentativeHScore = calculateHScore(neighbor, goalNode, dir);
                var tentativeFScore = tentativeGScore + tentativeHScore;
                var currentFScore =
                    gscore.getOrDefault(neighbor, Double.MAX_VALUE)
                    + hscore.getOrDefault(neighbor, Double.MAX_VALUE);

                if ((openList.contains(neighbor) || closedList.contains(neighbor))
                    && tentativeFScore > currentFScore
                ) {
                    continue;
                } else {
                    if (!openList.contains(neighbor)) {
                        openList.add(neighbor);
                    }
                    gscore.put(neighbor, tentativeGScore);
                    hscore.put(neighbor, tentativeHScore);
                    cameFrom.put(neighbor, current);
                }
            }

            closedList.add(current);
        }

        return Optional.empty();
    }

    private static List<OverwatchPos> reconstructPath(Map<OverwatchPos, OverwatchPos> cameFrom, OverwatchPos lastNode) {
        Deque<OverwatchPos> totalPath = new ArrayDeque<>();
        totalPath.add(lastNode);
        var current = lastNode;
        while (cameFrom.containsKey(current)) {
            current = cameFrom.get(current);
            totalPath.addFirst(current);
        }
        return totalPath.stream().toList();
    }
    
    /**
     * an array of vertices on the state graph of the superstructure defining
     * the area on that graph that is unsafe for the superstructure to travel in.
     *
     * It is expected that these vertices are sorted by pivot angle in ascending
     * order.
     */
    public static final OverwatchPos[] UNSAFE_ZONE;
    static {
        ArrayList<OverwatchPos> unsafeZone = new ArrayList<>();
        for (var pos : new OverwatchPos[] {
            OverwatchPos.of(Rotation2d.k180deg.unaryMinus(), 0),
            OverwatchPos.of(Rotation2d.kCW_90deg, 1.05),
            OverwatchPos.of(Rotation2d.kZero, 0),
        }) {
            unsafeZone.add(pos);
        }

        // unsafeZone gets mutated a lot, nice to have a copy of the original
        var unsafeZoneReference = unsafeZone.toArray(OverwatchPos[]::new);

        // wrap the unsafe zone around the periodic boundary
        for (var pos : unsafeZoneReference) {
            unsafeZone.add(OverwatchPos.of(
                // can't do .plus() because that'll clamp the rotation between -pi and pi
                Rotation2d.fromRadians(pos.pivotAngle().getRadians() + 2*Math.PI),
                pos.liftHeightMeters()
            ));
        }

        for (int i = unsafeZoneReference.length-1; i >= 0; i--) {
            var pos = unsafeZoneReference[i];
            unsafeZone.addFirst(OverwatchPos.of(
                Rotation2d.fromRadians(pos.pivotAngle().getRadians() - 2*Math.PI),
                pos.liftHeightMeters()
            ));
        }

        UNSAFE_ZONE = unsafeZone.toArray(OverwatchPos[]::new);
    }

    public static Translation2d nodeToTranslation2d(Node node) {
        return new Translation2d(
            node.pivotAngle().getRadians(),
            node.liftHeightMeters()
        );
    }

    public static Translation2d[] nodeSetToTransArr(EnumSet<Node> nodes) {
        Translation2d[] ret = new Translation2d[nodes.size()];
        for (int i = 0; i < nodes.size(); i++) {
            List<Node> nodesList = nodes.stream()
                .collect(Collectors.toList());
            OverwatchPos pos = nodesList.get(i);
            ret[i] = new Translation2d(pos.pivotAngle().getRadians(), pos.liftHeightMeters());
        }
        return ret;
    }

    public static Translation2d[] superstructurePosArrToTransArr(OverwatchPos[] positions) {
        Translation2d[] ret = new Translation2d[positions.length];
        for (int i = 0; i < positions.length; i++) {
            var pos = positions[i];
            ret[i] = new Translation2d(pos.pivotAngle().getRadians(), pos.liftHeightMeters());
        }
        return ret;
    }

    /**
     * Return an {@link OverwatchPos} that is shifted 2*pi to the left or right
     * if such a transformation is needed in order to be able to draw a straight
     * line from base to node while maintaining the direction specified.
     */
    public static OverwatchPos repositionNode(
        OverwatchPos base,
        OverwatchPos node,
        RotationalDirection dir
    ) {
        double nodeAngle = node.pivotAngle().getRadians();
        double baseAngle = base.pivotAngle().getRadians();

        OverwatchPos ret = node;

        if (dir == RotationalDirection.CLOCKWISE && baseAngle > nodeAngle) {
            ret = OverwatchPos.of(
                Rotation2d.fromRadians(nodeAngle + 2*Math.PI),
                node.liftHeightMeters()
            );
        }

        if (dir == RotationalDirection.COUNTER_CLOCKWISE && baseAngle < nodeAngle) {
            ret = OverwatchPos.of(
                Rotation2d.fromRadians(nodeAngle - 2*Math.PI),
                node.liftHeightMeters()
            );
        }

        return ret;
    }

    private static EnumSet<Node> getSafeNeighbors(OverwatchPos pos, RotationalDirection dir) {
        var ret = EnumSet.noneOf(Node.class);

        // add all neighbors whose edge doesn't intersect the unsafe zone
        for (Node unmodifiedNode : Node.values()) {

            OverwatchPos node = repositionNode(pos, unmodifiedNode, dir);

            boolean doIntersect = false;
            for (int i = 0; i < UNSAFE_ZONE.length - 1; i++) {
                OverwatchPos[][] segments = {
                    { pos, node },
                    { UNSAFE_ZONE[i], UNSAFE_ZONE[i+1] }
                };

                if (doIntersect(segments)) {
                    doIntersect = true;
                    break;
                }
            }

            if (!doIntersect) {
                ret.add(unmodifiedNode);
            }
        }

        return ret;
    }

    /* https://www.geeksforgeeks.org/dsa/check-if-two-given-line-segments-intersect/ */
    // function to check if point q lies on line segment 'pr'
    private static boolean onSegment(
        OverwatchPos p,
        OverwatchPos q,
        OverwatchPos r
    ) {
        double px = p.pivotAngle().getRadians();
        double py = p.liftHeightMeters();

        double qx = q.pivotAngle().getRadians();
        double qy = q.liftHeightMeters();

        double rx = r.pivotAngle().getRadians();
        double ry = r.liftHeightMeters();

        return (qx <= Math.max(px, rx) && 
                qx >= Math.min(px, rx) &&
                qy <= Math.max(py, ry) && 
                qy >= Math.min(py, ry));
    }

    // function to find orientation of ordered triplet (p, q, r)
    // 0 --> p, q and r are collinear
    // 1 --> Clockwise
    // 2 --> Counterclockwise
    public static int orientation(
        OverwatchPos p,
        OverwatchPos q,
        OverwatchPos r
    ) {
        double px = p.pivotAngle().getRadians();
        double py = p.liftHeightMeters();

        double qx = q.pivotAngle().getRadians();
        double qy = q.liftHeightMeters();

        double rx = r.pivotAngle().getRadians();
        double ry = r.liftHeightMeters();

        double val = (qy - py) * (rx - qx) -
                  (qx - px) * (ry - qy);

        if (val == 0) return 0;

        return (val > 0) ? 1 : 2;
    }

    // function to check if two line segments intersect
    public static boolean doIntersect(OverwatchPos[][] points) {

        int o1 = orientation(points[0][0], points[0][1], points[1][0]);
        int o2 = orientation(points[0][0], points[0][1], points[1][1]);
        int o3 = orientation(points[1][0], points[1][1], points[0][0]);
        int o4 = orientation(points[1][0], points[1][1], points[0][1]);

        return (o1 == 0 && onSegment(points[0][0], points[1][0], points[0][1]))
               || (o2 == 0 && onSegment(points[0][0], points[1][1], points[0][1]))
               || (o3 == 0 && onSegment(points[1][0], points[0][0], points[1][1]))
               || (o4 == 0 && onSegment(points[1][0], points[0][1], points[1][1]))
               || (o1 != o2 && o3 != o4);
    }
}
