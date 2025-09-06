package frc.robot.overwatch;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

@Logged
public class Lift extends SubsystemBase {
    private DCMotorSim sim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(2), 
            .097, // ~ 10lb 10in rod pivoting around one end
            5
        ),
        DCMotor.getKrakenX60(2),
        0, 0
    );

    private double pos;
    private double finalSetpoint;
    private double tentativeSetpoint;

    public final Trigger atFinalSetpoint =
        new Trigger(() -> MathUtil.isNear(finalSetpoint, pos, 0.1));
    public final Trigger atTentativeSetpoint =
        new Trigger(() -> MathUtil.isNear(tentativeSetpoint, pos, 0.1));

    private PIDController pidController;

    public Lift() {
        pidController = new PIDController(3.0, 0, 0.1);
        sim.setState(2.0*(2*Math.PI), 0.0);
        pos = sim.getAngularPositionRotations();
    }

    @Override
    public void periodic() {
        sim.update(0.02);
        pos = sim.getAngularPositionRotations();
    }

    public double getHeightMeters() {
        return pos;
    }

    protected void applyVolts(double volts) {
        sim.setInput(volts);
    }

    protected void goTo(double heightMeters) {
        var feedback = pidController.calculate(pos, heightMeters);
        sim.setInput(feedback);
    }

    protected void setFinalSetpoint(double heightMeters) {
        finalSetpoint = heightMeters;
    }

    protected void setTentativeSetpoint(double heightMeters) {
        tentativeSetpoint = heightMeters;
    }
}
