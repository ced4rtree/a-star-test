package frc.robot.overwatch;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
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
        public Rotation2d pivotAngle();

        public static OverwatchPos of(Rotation2d pivotAngle, double liftHeightMeters) {
            return new OverwatchPos() {
                public double liftHeightMeters() {
                    return liftHeightMeters;
                }

                public Rotation2d pivotAngle() {
                    return pivotAngle;
                }
            };
        }

        /**
         * Return the distance from one node to another on the graph.
         *
         * This will not take continuity into account. For example, the angular
         * distance between -170 and 170 degrees will be assumed as 340 degrees,
         * not 20 degrees.
         *
         * @param other The node to measure distance from.
         */
        public default double distanceFrom(OverwatchPos other) {
            var liftDelta = this.heightDelta(other);
            var angleDelta = this.angleDelta(other);
            return Math.hypot(
                liftDelta,
                angleDelta.getRadians()
            );
        }

        public default double heightDelta(OverwatchPos other) {
            return liftHeightMeters() - other.liftHeightMeters();
        }

        public default Rotation2d angleDelta(OverwatchPos other) {
            return Rotation2d.fromRadians(
                // can't use Rotation2d.minus() since that wraps the output between -pi and pi
                pivotAngle().getRadians() - other.pivotAngle().getRadians()
            );
        }

        public default Node closestNode() {
            return closestNode(Double.MAX_VALUE, Double.MAX_VALUE).get();
        }

        public default Optional<Node> closestNode(double angleToleranceRads, double heightToleranceMeters) {
            Node ret = Node.HOME;
            for (Node node : Node.values()) {
                double currentNodeDist = distanceFrom(ret);
                double otherNodeDist = distanceFrom(node);

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
                ret.pivotAngle().getRadians(),
                pivotAngle().getRadians(),
                angleToleranceRads
            );
            
            if (liftHeightClose && pivotAngleClose) {
                return Optional.of(ret);
            } else {
                return Optional.empty();
            }
        }

        public default String string() {
            return "theta: " + pivotAngle().getRadians() + ", height: " + liftHeightMeters();
        }
    }
}
