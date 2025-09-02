package frc.robot.overwatch;

import java.util.Optional;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.overwatch.Graph.Node;
import frc.robot.overwatch.OverwatchConstants.SuperstructurePosition;

@Logged
public class Overwatch extends SubsystemBase {
    private Pivot pivot = new Pivot();
    private Lift lift = new Lift();

    private TrapezoidProfile motionProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(1.0, 2.0)
    );
    private TrapezoidProfile.State previousSetpoint
        = new TrapezoidProfile.State();
    // Not a Node in order to make the logic around starting in an arbitrary position easier
    private SuperstructurePosition previousDestination =
        new SuperstructurePosition(lift.getHeightMeters(), pivot.getAngleRads());
    private Node destinationNode;

    public final Trigger atSetpoint = lift.atSetpoint.and(pivot.atSetpoint);

    private Mechanism2d visualization;
    private MechanismLigament2d liftLigament;
    private MechanismLigament2d pivotLigament;

    public Overwatch() {
        visualization = new Mechanism2d(5.0, 5.0);
        var liftRoot = visualization.getRoot("Overwatch", 2.5, 0.1);
        liftLigament = liftRoot.append(new MechanismLigament2d("Lift", 0, 90));
        pivotLigament = liftLigament.append(new MechanismLigament2d("Pivot", 1, -90));

        SmartDashboard.putData("Visualization", visualization);
    }

    @Override
    public void periodic() {
        liftLigament.setLength(lift.getHeightMeters());
        pivotLigament.setAngle(Rotation2d.fromRadians(pivot.getAngleRads() - Math.PI/2));
    }

    private void followEdge(double t) {
        TrapezoidProfile.State setpoint = motionProfile.calculate(
            t,
            previousSetpoint,
            new TrapezoidProfile.State(
                destinationNode.position.distanceFrom(previousDestination),
                0
            )
        );

        double edgeAngle = Math.atan2(
            destinationNode.position.heightDelta(previousDestination),
            destinationNode.position.angleDelta(previousDestination)
        );
        lift.goTo(previousDestination.liftHeightMeters()
                  + setpoint.position * Math.sin(edgeAngle)
        );
        pivot.goTo(previousDestination.pivotAngleRads()
                   + setpoint.position * Math.cos(edgeAngle)
        );

        if (atSetpoint.getAsBoolean()) {
            previousDestination = destinationNode.position;
        }
    }

    public Command goTo(Node node) {
        Timer timer = new Timer();
        return this.runOnce(() -> {
            timer.restart();
            destinationNode = node;
            lift.setFinalSetpoint(node.position.liftHeightMeters());
            pivot.setFinalSetpoint(node.position.pivotAngleRads());
        }).andThen(this.run(() -> followEdge(timer.get())));
    }
}
