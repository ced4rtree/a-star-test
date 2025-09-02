package frc.robot.overwatch;

public final class OverwatchConstants {
    public record SuperstructurePosition(
        double liftHeightMeters,
        double pivotAngleRads
    ) {
        public double distanceFrom(SuperstructurePosition other) {
            return Math.hypot(
                other.liftHeightMeters() - this.liftHeightMeters(),
                other.pivotAngleRads() - this.pivotAngleRads()
            );
        }

        public double heightDelta(SuperstructurePosition other) {
            return liftHeightMeters() - other.liftHeightMeters();
        }

        public double angleDelta(SuperstructurePosition other) {
            return pivotAngleRads() - other.pivotAngleRads();
        }

    }
}
