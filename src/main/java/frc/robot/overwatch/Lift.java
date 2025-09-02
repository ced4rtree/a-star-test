package frc.robot.overwatch;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
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
        .001, .001
    );

    private double pos;
    private double setpoint;

    public final Trigger atSetpoint =
        new Trigger(() -> MathUtil.isNear(setpoint, pos, 0.1));

    private PIDController pidController;

    public Lift() {
        pidController = new PIDController(50.0, 0, 3.0);
    }

    @Override
    public void periodic() {
        sim.update(0.02);
        pos = sim.getAngularPositionRotations();
    }

    public double getHeightMeters() {
        return pos;
    }

    protected void goTo(double heightMeters) {
        pidController.setSetpoint(heightMeters);
        var feedback = pidController.calculate(pos);
        sim.setInput(feedback);
    }

    protected void setFinalSetpoint(double heightMeters) {
        setpoint = heightMeters;
    }
}
