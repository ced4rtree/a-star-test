package frc.robot.overwatch;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;

@Logged
public class Pivot extends SubsystemBase {
    private SingleJointedArmSim sim = new SingleJointedArmSim(
        DCMotor.getKrakenX60(1),
        100,
        SingleJointedArmSim.estimateMOI(1.0, 20.0),
        1.0,
        -Double.MAX_VALUE,
        Double.MAX_VALUE,
        false,
        -Math.PI/2,
        0.0001, 0.0001
    );

    private PIDController pidController;

    private double pos;
    private double setpoint;

    public final Trigger atSetpoint =
        // new Trigger(() -> {
        //     // Takes care of the discontinuity around theta = pi or theta = -pi
        //     double delta = pos - setpoint;
        //     double continuousDelta = (delta + Math.PI) % 2*Math.PI - Math.PI;
        //     return MathUtil.isNear(0, continuousDelta, 0.02);
        // });
        new Trigger(() -> MathUtil.isNear(setpoint, pos, 0.02));

    public Pivot() {
        pidController = new PIDController(40.0, 0.0, 2.2);
        pidController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void periodic() {
        sim.update(.02);
        pos = MathUtil.angleModulus(sim.getAngleRads());
        // pos = sim.getAngleRads();
    }

    public double getAngleRads() {
        return pos;
    }

    protected void goTo(double angleRads) {
        pidController.setSetpoint(angleRads);
        var feedback = pidController.calculate(pos);
        sim.setInput(feedback);
    }

    protected void setFinalSetpoint(double angleRads) {
        setpoint = MathUtil.angleModulus(angleRads);
    }
}
