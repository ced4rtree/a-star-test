package frc.robot.overwatch;

import frc.robot.overwatch.OverwatchConstants.SuperstructurePosition;

public class Graph {
    public enum Node {
        HOME(new SuperstructurePosition(1.1, -Math.PI/2)),
        CORAL_PICK(new SuperstructurePosition(1.0, -Math.PI/2)),
        L2_PREP(new SuperstructurePosition(0.2, Math.PI/6)),
        ;

        public final SuperstructurePosition position;
        Node(SuperstructurePosition position) {
            this.position = position;
        }
    }
}
