package frc.robot.overwatch;

import java.util.EnumSet;
import java.util.List;
import java.util.stream.Collectors;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.overwatch.OverwatchConstants.SuperstructurePosition;

public final class Graph {
    public enum Node {
        HOME(new SuperstructurePosition(-Math.PI/2, 1.2)),
        CORAL_PICK(new SuperstructurePosition(-Math.PI/2, 1.1)),
        L2_PREP(new SuperstructurePosition(Math.PI/6, 0.2)),
        FOO(new SuperstructurePosition(-5*Math.PI/6, 0.8)),
        BAR(new SuperstructurePosition(Math.PI/6, 0.8))
        ;

        public final SuperstructurePosition position;

        // to be amended to during initialization (see static block below)
        // initializing now messes up the jvm since Node isn't a class yet, has to stay null for now
        private EnumSet<Node> safeNeighbors;

        Node(SuperstructurePosition position) {
            this.position = position;
        }

        static {
            for (Node node : Node.values()) {
                node.safeNeighbors = EnumSet.noneOf(Node.class);
            }

            // add all neighbors whose edge doesn't intersect the unsafe zone
            for (Node nodeA : Node.values()) {
                for (Node nodeB : Node.values()) {
                    for (int i = 0; i < UNSAFE_ZONE.length - 1; i++) {
                        SuperstructurePosition[][] segments = {
                            { nodeA.position, nodeB.position },
                            { UNSAFE_ZONE[i], UNSAFE_ZONE[i+1] }
                        };

                        if (doIntersect(segments)) {
                            break;
                        } else {
                            nodeA.safeNeighbors.add(nodeB);
                            nodeB.safeNeighbors.add(nodeA);
                        }
                    }
                }
            }
        }

        public EnumSet<Node> getNeighbors() {
            return safeNeighbors;
        }
    }

    /**
     * an array of vertices on the state graph of the superstructure defining
     * the area on that graph that is unsafe for the superstructure to travel in.
     *
     * It is expected that these vertices are sorted by pivot angle in ascending
     * order.
     */
    public static final SuperstructurePosition[] UNSAFE_ZONE = {
        new SuperstructurePosition(-Math.PI, 0),
        new SuperstructurePosition(-Math.PI/2, 1.05),
        new SuperstructurePosition(0, 0),
    };

    public static Translation2d nodeToTranslation2d(Node node) {
        return new Translation2d(
            node.position.pivotAngleRads(),
            node.position.liftHeightMeters()
        );
    }

    public static Translation2d[] nodeSetToTransArr(EnumSet<Node> nodes) {
        Translation2d[] ret = new Translation2d[nodes.size()];
        for (int i = 0; i < nodes.size(); i++) {
            List<Node> nodesList = nodes.stream()
                .collect(Collectors.toList());
            SuperstructurePosition pos = nodesList.get(i).position;
            ret[i] = new Translation2d(pos.pivotAngleRads(), pos.liftHeightMeters());
        }
        return ret;
    }

    public static Translation2d[] superstructurePosArrToTransArr(SuperstructurePosition[] positions) {
        Translation2d[] ret = new Translation2d[positions.length];
        for (int i = 0; i < positions.length; i++) {
            var pos = positions[i];
            ret[i] = new Translation2d(pos.pivotAngleRads(), pos.liftHeightMeters());
        }
        return ret;
    }

    /* https://www.geeksforgeeks.org/dsa/check-if-two-given-line-segments-intersect/ */
    // function to check if point q lies on line segment 'pr'
    private static boolean onSegment(
        SuperstructurePosition p,
        SuperstructurePosition q,
        SuperstructurePosition r
    ) {
        double px = p.pivotAngleRads();
        double py = p.liftHeightMeters();

        double qx = q.pivotAngleRads();
        double qy = q.liftHeightMeters();

        double rx = r.pivotAngleRads();
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
        SuperstructurePosition p,
        SuperstructurePosition q,
        SuperstructurePosition r
    ) {
        double px = p.pivotAngleRads();
        double py = p.liftHeightMeters();

        double qx = q.pivotAngleRads();
        double qy = q.liftHeightMeters();

        double rx = r.pivotAngleRads();
        double ry = r.liftHeightMeters();

        double val = (qy - py) * (rx - qx) -
                  (qx - px) * (ry - qy);

        if (val == 0) return 0;

        return (val > 0) ? 1 : 2;
    }

    // function to check if two line segments intersect
    public static boolean doIntersect(SuperstructurePosition[][] points) {

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
