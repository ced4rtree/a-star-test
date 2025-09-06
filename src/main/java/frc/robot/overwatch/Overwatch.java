package frc.robot.overwatch;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructArrayTopic;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.overwatch.Graph.Node;
import frc.robot.overwatch.OverwatchConstants.OverwatchPos;

@Logged
public class Overwatch extends SubsystemBase {
    public Pivot pivot = new Pivot();
    public Lift lift = new Lift();

    private TrapezoidProfile.State previousSetpoint
        = new TrapezoidProfile.State();
    private OverwatchPos previousDestination =
        OverwatchPos.of(pivot.getAngleRads(), lift.getHeightMeters());
    // the nodes that will move the lift & pivot while avoiding the unsafe zone
    private List<OverwatchPos> path = List.of(Node.HOME);
    private Node finalDestination = Node.HOME;

    public final Trigger atGlobalSetpoint = lift.atFinalSetpoint.and(pivot.atFinalSetpoint);
    private final Trigger atTentativeSetpoint
        = lift.atTentativeSetpoint.and(pivot.atTentativeSetpoint);

    private Mechanism2d visualization;
    private MechanismLigament2d liftLigament;
    private MechanismLigament2d pivotLigament;

    Translation2d[] safeWaypointNeighbors;
    Translation2d[] unsafeWaypointNeighbors;
    Translation2d[] allWaypoints;
    StructArrayPublisher<Translation2d> safeWaypointPub;
    StructArrayPublisher<Translation2d> unsafeWaypointPub;
    StructArrayPublisher<Translation2d> allWaypointPub;
    StructPublisher<Translation2d> positionPub;
    SendableChooser<Node> nodeChooser;

    public Overwatch() {
        visualization = new Mechanism2d(5.0, 5.0);
        var liftRoot = visualization.getRoot("Overwatch", 2.5, 0.1);
        liftLigament = liftRoot.append(new MechanismLigament2d("Lift", 0, 90));
        pivotLigament = liftLigament.append(new MechanismLigament2d("Pivot", 1, -90));

        SmartDashboard.putData("Visualization", visualization);

        nodeChooser = new SendableChooser<>();
        for (Node node : Node.values()) {
            nodeChooser.addOption(node.name(), node);
        }
        nodeChooser.setDefaultOption("FOO", Node.FOO);
        SmartDashboard.putData(nodeChooser);
        // safeWaypointNeighbors = Graph.nodeSetToTransArr(nodeChooser.getSelected().getNeighbors());
        safeWaypointNeighbors = Graph.nodeSetToTransArr(finalDestination.getNeighbors());
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

        setDefaultCommand(this.run(() -> {
            pivot.applyVolts(0);
            lift.applyVolts(0);
        }));
    }

    @Override
    public void periodic() {
        liftLigament.setLength(lift.getHeightMeters());
        pivotLigament.setAngle(Rotation2d.fromRadians(pivot.getAngleRads() - Math.PI/2));

        // safeWaypointNeighbors = Graph.nodeSetToTransArr(nodeChooser.getSelected().getNeighbors());
        safeWaypointNeighbors = Graph.nodeSetToTransArr(finalDestination.getNeighbors());
        unsafeWaypointPub.set(unsafeWaypointNeighbors);
        safeWaypointPub.set(safeWaypointNeighbors);
        allWaypointPub.set(allWaypoints);
        positionPub.set(new Translation2d(pivot.getAngleRads(), lift.getHeightMeters()));
    }

    private void followEdge(OverwatchPos end, double t) {
        // System.out.println("Time: " + t);
        TrapezoidProfile.State setpoint = OverwatchConstants.MOTION_PROFILE.calculate(
            t,
            new TrapezoidProfile.State(),
            new TrapezoidProfile.State(
                end.distanceFrom(previousDestination),
                0
            )
        );

        double edgeAngle = Math.atan2(
            end.heightDelta(previousDestination),
            end.angleDelta(previousDestination)
        );

        var liftHeight = previousDestination.liftHeightMeters()
                         + setpoint.position * Math.sin(edgeAngle);
        lift.goTo(liftHeight);

        var pivotAngle = previousDestination.pivotAngleRads()
                         + setpoint.position * Math.cos(edgeAngle);
        pivot.goTo(pivotAngle);

        System.out.println("Setpoint position: " + setpoint.position);
        System.out.println("blah: " + end.distanceFrom(previousDestination));
        System.out.println("Target angle: " + pivotAngle + ", Target height: " + liftHeight);

        if (atTentativeSetpoint.getAsBoolean()) {
            previousDestination = end;
        }
    }

    public Command goTo(Node node) {
        Timer timer = new Timer();
        return this.runOnce(() -> {
            if (atTentativeSetpoint.getAsBoolean()) {
                path = Graph.pathfind(finalDestination, node)
                    .orElseThrow();
            } else {
                var currentPos = OverwatchPos.of(
                    pivot.getAngleRads(),
                    lift.getHeightMeters()
                );
                path = Graph.pathfind(currentPos, node)
                    .orElseThrow();
                previousDestination = OverwatchPos.of(
                    pivot.getAngleRads(),
                    lift.getHeightMeters()
                );
            }
            for (var pos : path) {
                System.out.println("x: " + pos.pivotAngleRads() + " y: " + pos.liftHeightMeters());
            }
            finalDestination = node;
            lift.setFinalSetpoint(node.liftHeightMeters());
            pivot.setFinalSetpoint(node.pivotAngleRads());
        }).andThen(this.runOnce(() -> {
            Command followEdgeCommand = Commands.none();
            for (OverwatchPos pos : path) {
                followEdgeCommand = followEdgeCommand.andThen(
                    this.run(() -> followEdge(pos, timer.get()))
                        .until(atTentativeSetpoint)
                        .beforeStarting(() -> {
                            previousSetpoint = new TrapezoidProfile.State();
                            timer.restart();
                            lift.setTentativeSetpoint(pos.liftHeightMeters());
                            pivot.setTentativeSetpoint(pos.pivotAngleRads());
                        })
                );
            }
            CommandScheduler.getInstance().schedule(followEdgeCommand);
        }));
    }
}
