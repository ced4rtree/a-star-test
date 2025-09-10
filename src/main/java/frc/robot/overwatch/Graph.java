package frc.robot.overwatch;

import java.util.ArrayDeque;
import java.util.ArrayList;
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
import frc.robot.overwatch.Overwatch.RotationType;
import frc.robot.overwatch.OverwatchConstants.OverwatchPos;

public final class Graph {
    public enum Node implements OverwatchPos {
        HOME       (-Math.PI/2,   1.2),
        CORAL_PICK (-Math.PI/2,   1.1),
        L2_PREP    (Math.PI/6,    0.2),
        FOO        (-5*Math.PI/6, 0.8),
        BAR        (Math.PI/6,    0.8),
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
                node.safeClockwiseNeighbors = getSafeNeighbors(new DirectionalPos(
                    node,
                    RotationalDirection.CLOCKWISE
                ));
                node.safeCounterClockwiseNeighbors = getSafeNeighbors(new DirectionalPos(
                    node,
                    RotationalDirection.COUNTER_CLOCKWISE
                ));
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
            return Graph.travelTime(new DirectionalPos(this, dir), node);
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
    public static record PathQuery(OverwatchPos startNode, Node goalNode, RotationType rotationType) {}

    /**
     * Wrapper around {@link OverwatchPos} and a {@link RotationalDirection} to
     * make a node distinct from another node approached from the other
     * direction. dir notes the direction this pos will follow after this move,
     * not the dir that led to this pos.
     */
    public static record DirectionalPos(OverwatchPos pos, RotationalDirection dir) {
        public String toString() {
            return pos.string() + ", dir: " + dir;
        }
    }

    public enum RotationalDirection {
        CLOCKWISE,
        COUNTER_CLOCKWISE,
        ;
    }

    private static Map<PathQuery, List<DirectionalPos>> aStarCache
        = new HashMap<>();

    /**
     * Return the amount of time it takes to travel between the start node
     * and the goal node.
     */
    public static double travelTime(DirectionalPos startNode, OverwatchPos goalNode_) {
        OverwatchPos goalNode = repositionNode(startNode, goalNode_);
        double nodeToNodeDist = startNode.pos().distanceFrom(goalNode);
        var initialState = new TrapezoidProfile.State();
        var finalState = new TrapezoidProfile.State(nodeToNodeDist, 0.0);
        OverwatchConstants.MOTION_PROFILE.calculate(0.02, initialState, finalState);

        return OverwatchConstants.MOTION_PROFILE.totalTime();
    }

    /**
     * Returns the gscore for the final node in the provided set
     */
    private static double cumulativeGScore(List<DirectionalPos> nodes) {
        double gscore = 0.0;

        for (int i = 0; i < nodes.size() - 1; i++) {
            var node = nodes.get(i);
            gscore += travelTime(node, nodes.get(i+1).pos());
        }
        return gscore;
    }

    private static double calculateHScore(
        DirectionalPos from,
        OverwatchPos goalNode,
        RotationType rotationType
    ) {
        // give any non-preferred movement of the arm a huge penalty
        double rotationPenalty = 0;
        if ((rotationType == RotationType.CLOCKWISE
             && from.dir() == RotationalDirection.COUNTER_CLOCKWISE)
            || (rotationType == RotationType.COUNTER_CLOCKWISE
                && from.dir() == RotationalDirection.CLOCKWISE)
        ) {
            rotationPenalty = Double.MAX_VALUE;
        }

        return travelTime(from, goalNode) + rotationPenalty;
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
    public static Optional<List<DirectionalPos>> pathfind(
        OverwatchPos startNode,
        Node goalNode,
        RotationType rotationType
    ) {
        PathQuery pathQuery = new PathQuery(startNode, goalNode, rotationType);
        if (aStarCache.containsKey(pathQuery)) {
            System.out.println("Cached!");
            return Optional.of(aStarCache.get(pathQuery));
        }

        if (goalNode.equals(startNode)) {
            return Optional.of(List.of());
        }

        var openList = new ArrayList<DirectionalPos>();
        var initialDirection = switch (rotationType) {
            case CLOCKWISE -> RotationalDirection.CLOCKWISE;
            case COUNTER_CLOCKWISE -> RotationalDirection.COUNTER_CLOCKWISE;
            default -> RotationalDirection.CLOCKWISE;
        };
        var initialDirectionalPos = new DirectionalPos(startNode, initialDirection);
        openList.add(initialDirectionalPos);
        var closedList = new ArrayList<DirectionalPos>();
        Map<OverwatchPos, DirectionalPos> cameFrom = new HashMap<>();
        Map<DirectionalPos, Double> gscore = new HashMap<>();
        Map<DirectionalPos, Double> hscore = new HashMap<>();

        gscore.put(initialDirectionalPos, 0.0);
        hscore.put(initialDirectionalPos, calculateHScore(
            initialDirectionalPos,
            goalNode,
            rotationType
        ));

        DirectionalPos current = initialDirectionalPos;
        while (!openList.isEmpty()) {
            for (DirectionalPos node : openList) {
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
            if (current.pos().getClass() == Node.class) {
                neighbors = ((Node)current.pos()).getNeighbors(current.dir());
            } else {
                neighbors = getSafeNeighbors(current);
            }

            for (OverwatchPos neighbor : neighbors) {
                for (var dir : RotationalDirection.values()) {
                    if (neighbor == goalNode && dir == current.dir()) {
                        cameFrom.put(goalNode, current);
                        var ret = reconstructPath(cameFrom, goalNode);
                        aStarCache.put(pathQuery, ret);
                        return Optional.of(ret);
                    }

                    var directionalNeighbor = new DirectionalPos(neighbor, dir);

                    if (closedList.contains(directionalNeighbor)) {
                        continue;
                    }

                    if (!openList.contains(directionalNeighbor)) {
                        openList.add(directionalNeighbor);
                        hscore.put(directionalNeighbor, calculateHScore(directionalNeighbor, goalNode, rotationType));
                        var path = reconstructPath(cameFrom, neighbor);
                        gscore.put(directionalNeighbor, cumulativeGScore(path));
                    } else {
                        DirectionalPos oldConnection = cameFrom.get(neighbor);
                        cameFrom.put(neighbor, current);
                        var tentativeGScore = cumulativeGScore(reconstructPath(cameFrom, neighbor));
                        var currentGScore = gscore.getOrDefault(directionalNeighbor, Double.MAX_VALUE);
                        if (tentativeGScore < currentGScore) {
                            gscore.put(directionalNeighbor, tentativeGScore);
                        } else {
                            // restore old connection since it was broken a few lines ago
                            cameFrom.put(neighbor, oldConnection);
                        }
                    }
                }
            }
        }

        return Optional.empty();
    }

    private static List<DirectionalPos> reconstructPath(Map<OverwatchPos, DirectionalPos> cameFrom, OverwatchPos lastNode) {
        Deque<DirectionalPos> totalPath = new ArrayDeque<>();
        var lastPos = new DirectionalPos(lastNode, RotationalDirection.CLOCKWISE);
        totalPath.add(lastPos);
        var current = lastPos;
        while (cameFrom.containsKey(current.pos())) {
            current = cameFrom.get(current.pos());
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
     * line from base to node while maintaining the direction specified by base.
     */
    public static OverwatchPos repositionNode(DirectionalPos base, OverwatchPos node) {
        double nodeAngle = node.pivotAngle().getRadians();
        double baseAngle = base.pos().pivotAngle().getRadians();

        OverwatchPos ret = node;

        if (base.dir() == RotationalDirection.CLOCKWISE
            && baseAngle > nodeAngle
        ) {
            ret = OverwatchPos.of(
                Rotation2d.fromRadians(nodeAngle + 2*Math.PI),
                node.liftHeightMeters()
            );
        } else if (base.dir() == RotationalDirection.COUNTER_CLOCKWISE
                   && baseAngle < nodeAngle
        ) {
            ret = OverwatchPos.of(
                Rotation2d.fromRadians(nodeAngle - 2*Math.PI),
                node.liftHeightMeters()
            );
        }

        return ret;
    }

    private static EnumSet<Node> getSafeNeighbors(DirectionalPos pos) {
        var ret = EnumSet.noneOf(Node.class);

        // add all neighbors whose edge doesn't intersect the unsafe zone
        for (Node unmodifiedNode : Node.values()) {

            OverwatchPos node = repositionNode(pos, unmodifiedNode);

            for (int i = 0; i < UNSAFE_ZONE.length - 1; i++) {
                OverwatchPos[][] segments = {
                    { pos.pos(), node },
                    { UNSAFE_ZONE[i], UNSAFE_ZONE[i+1] }
                };

                if (doIntersect(segments)) {
                    break;
                } else if (node != pos.pos()) {
                    ret.add(unmodifiedNode);
                }
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
