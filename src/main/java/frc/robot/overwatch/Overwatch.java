package frc.robot.overwatch;

import java.util.List;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.overwatch.Graph.Node;
import frc.robot.overwatch.Graph.RotationalDirection;
import frc.robot.overwatch.OverwatchConstants.OverwatchPos;

@Logged
public class Overwatch extends SubsystemBase {
    public enum RotationType {
        CLOCKWISE,
        COUNTER_CLOCKWISE,
        SHORTEST
    }

    public Pivot pivot = new Pivot();
    public Lift lift = new Lift();

    private OverwatchPos previousDestination =
        OverwatchPos.of(pivot.getAngle(), lift.getHeightMeters());
    private Node finalDestination = Node.HOME;

    public final Trigger atGlobalSetpoint = lift.atFinalSetpoint.and(pivot.atFinalSetpoint);
    private final Trigger atTentativeSetpoint
        = lift.atTentativeSetpoint.and(pivot.atTentativeSetpoint);

    private Mechanism2d visualization;
    private MechanismLigament2d liftLigament;
    private MechanismLigament2d pivotLigament;

    private GraphVisualizer graphVisualizer;

    public Overwatch() {
        visualization = new Mechanism2d(5.0, 5.0);
        var liftRoot = visualization.getRoot("Overwatch", 2.5, 0.1);
        liftLigament = liftRoot.append(new MechanismLigament2d("Lift", 0, 90));
        pivotLigament = liftLigament.append(new MechanismLigament2d("Pivot", 1, -90));

        SmartDashboard.putData("Robot/overwatch/visualization", visualization);

        setDefaultCommand(this.run(() -> {
            pivot.applyVolts(0);
            lift.applyVolts(0);
        }));

        graphVisualizer = new GraphVisualizer();
    }

    @Override
    public void periodic() {
        liftLigament.setLength(lift.getHeightMeters());
        pivotLigament.setAngle(pivot.getAngle().minus(Rotation2d.kCCW_90deg));

        if (RobotBase.isSimulation()) {
            graphVisualizer.visualize(
                finalDestination,
                pivot.getAngle().getRadians(),
                lift.getHeightMeters()
            );
        }
    }

    private void followEdge(OverwatchPos from, OverwatchPos to, RotationalDirection dir, double t) {
        OverwatchPos goal = Graph.repositionNode(from, to, dir);

        TrapezoidProfile.State setpoint = OverwatchConstants.MOTION_PROFILE.calculate(
            t,
            new TrapezoidProfile.State(),
            new TrapezoidProfile.State(
                goal.distanceFrom(previousDestination),
                0
            )
        );

		double edgeAngle = Math.atan2(
            goal.heightDelta(previousDestination),
            goal.angleDelta(previousDestination).getRadians()
        );

        var liftHeight = previousDestination.liftHeightMeters()
                         + setpoint.position * Math.sin(edgeAngle);
        lift.goTo(liftHeight);

        var pivotAngle = previousDestination.pivotAngle()
            .plus(Rotation2d.fromRadians(setpoint.position * Math.cos(edgeAngle)));
        pivot.goTo(pivotAngle);

        if (atTentativeSetpoint.getAsBoolean()) {
            // don't use goal here, since that's repositioned on the graph
            previousDestination = to;
        }
    }

    public Command goTo(Node node, RotationType rotationType) {
        Timer timer = new Timer();

        return this.runOnce(() -> {
            List<OverwatchPos> path = List.of();
            RotationalDirection dir;

            OverwatchPos currentPosition;
            if (atTentativeSetpoint.getAsBoolean()) {
                currentPosition = previousDestination;
            } else {
                currentPosition = OverwatchPos.of(
                    pivot.getAngle(),
                    lift.getHeightMeters()
                );
                previousDestination = currentPosition;
            }

            dir = switch (rotationType) {
                case CLOCKWISE -> RotationalDirection.CLOCKWISE;
                case COUNTER_CLOCKWISE -> RotationalDirection.COUNTER_CLOCKWISE;
                default -> {
                    // choose whichever direction has the shortest time when
                    // going straight from node A to node B
                    double travelTimeClockwise = Graph.travelTime(
                        currentPosition,
                        node,
                        RotationalDirection.CLOCKWISE
                    );
                    double travelTimeCounterClockwise = Graph.travelTime(
                        currentPosition,
                        node,
                        RotationalDirection.COUNTER_CLOCKWISE
                    );

                    if (travelTimeClockwise > travelTimeCounterClockwise) {
                        yield RotationalDirection.COUNTER_CLOCKWISE;
                    } else {
                        yield RotationalDirection.CLOCKWISE;
                    }
                }
            };
            path = Graph.pathfind(currentPosition, node, dir)
                .orElseThrow();
            finalDestination = node;
            lift.setFinalSetpoint(node.liftHeightMeters());
            pivot.setFinalSetpoint(node.pivotAngle());

            Command followEdgeCommand = Commands.none();
            for (int i = 0; i < path.size() - 1; i++) {
                var currentPos = path.get(i);
                var nextPos = path.get(i+1);
                followEdgeCommand = followEdgeCommand.andThen(
                    this.run(() -> followEdge(currentPos, nextPos, dir, timer.get()))
                        .until(atTentativeSetpoint)
                        .beforeStarting(() -> {
                            timer.restart();
                            lift.setTentativeSetpoint(nextPos.liftHeightMeters());
                            pivot.setTentativeSetpoint(nextPos.pivotAngle());
                    })
                );
            }
            CommandScheduler.getInstance().schedule(followEdgeCommand);
        });
    }
}
