package frc.robot.overwatch;

import java.util.EnumSet;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructArrayTopic;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.overwatch.Graph.Node;
import frc.robot.overwatch.Graph.RotationalDirection;

public class GraphVisualizer {
    Translation2d[] safeWaypointNeighbors;
    Translation2d[] unsafeWaypointNeighbors;
    Translation2d[] allWaypoints;
    StructArrayPublisher<Translation2d> safeWaypointPub;
    StructArrayPublisher<Translation2d> unsafeWaypointPub;
    StructArrayPublisher<Translation2d> allWaypointPub;
    StructPublisher<Translation2d> positionPub;

    public GraphVisualizer() {
        safeWaypointNeighbors = Graph.nodeSetToTransArr(getAllNeighbors(Node.HOME));
        unsafeWaypointNeighbors = Graph.superstructurePosArrToTransArr(Graph.UNSAFE_ZONE);
        allWaypoints = Graph.nodeSetToTransArr(EnumSet.allOf(Node.class));

        var inst = NetworkTableInstance.getDefault();
        StructArrayTopic<Translation2d> unsafeTopic = inst.getStructArrayTopic("Robot/overwatch/unsafeVertices", Translation2d.struct);
        StructArrayTopic<Translation2d> safeTopic = inst.getStructArrayTopic("Robot/overwatch/safeNeighbors", Translation2d.struct);
        StructArrayTopic<Translation2d> allTopic = inst.getStructArrayTopic("Robot/overwatch/allWaypoints", Translation2d.struct);
        unsafeWaypointPub = unsafeTopic.publish();
        safeWaypointPub = safeTopic.publish();
        allWaypointPub = allTopic.publish();
        positionPub = inst.getStructTopic("Robot/overwatch/position", Translation2d.struct).publish();
    }

    public void visualize(Node currentNode, double pivotAngle, double liftHeight) {
        safeWaypointNeighbors = Graph.nodeSetToTransArr(getAllNeighbors(currentNode));
        unsafeWaypointPub.set(unsafeWaypointNeighbors);
        safeWaypointPub.set(safeWaypointNeighbors);
        allWaypointPub.set(allWaypoints);
        positionPub.set(new Translation2d(pivotAngle, liftHeight));
    }

    private EnumSet<Node> getAllNeighbors(Node node) {
        var neighbors = Node.HOME.getNeighbors(RotationalDirection.CLOCKWISE);
        neighbors.addAll(Node.HOME.getNeighbors(RotationalDirection.COUNTER_CLOCKWISE));
        return neighbors;
    }
}
