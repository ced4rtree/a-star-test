package frc.robot.overwatch;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.overwatch.Graph.Node;

public final class OverwatchConstants {
    public static final TrapezoidProfile MOTION_PROFILE = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(1, 2)
    );
    

    public interface OverwatchPos {
        // value getters
        // override with functions returning their values
        public double liftHeightMeters();
        public double pivotAngleRads();

        public static OverwatchPos of(double pivotAngleRads, double liftHeightMeters) {
            return new OverwatchPos() {
                public double liftHeightMeters() {
                    return liftHeightMeters;
                }

                public double pivotAngleRads() {
                    return pivotAngleRads;
                }
            };
        }

        public default double distanceFrom(OverwatchPos other) {
            return Math.hypot(
                other.liftHeightMeters() - this.liftHeightMeters(),
                other.pivotAngleRads() - this.pivotAngleRads()
            );
        }

        public default double heightDelta(OverwatchPos other) {
            return liftHeightMeters() - other.liftHeightMeters();
        }

        public default double angleDelta(OverwatchPos other) {
            return pivotAngleRads() - other.pivotAngleRads();
        }

        public default Node closestNode() {
            return closestNode(Double.MAX_VALUE, Double.MAX_VALUE).get();
        }

        public default Optional<Node> closestNode(double angleToleranceRads, double heightToleranceMeters) {
            Node ret = Node.HOME;
            for (Node node : Node.values()) {
                double currentNodeDist = Math.hypot(
                    ret.liftHeightMeters() - liftHeightMeters(),
                    ret.pivotAngleRads() - pivotAngleRads()
                );
                double otherNodeDist = Math.hypot(
                    node.liftHeightMeters() - liftHeightMeters(),
                    node.pivotAngleRads() - pivotAngleRads()
                );

                if (otherNodeDist < currentNodeDist) {
                    ret = node;
                }
            }

            boolean liftHeightClose = MathUtil.isNear(
                ret.liftHeightMeters(),
                liftHeightMeters(),
                heightToleranceMeters
            );
            boolean pivotAngleClose = MathUtil.isNear(
                ret.pivotAngleRads(),
                pivotAngleRads(),
                angleToleranceRads
            );
            
            if (liftHeightClose && pivotAngleClose) {
                return Optional.of(ret);
            } else {
                return Optional.empty();
            }
        }
    }
}
