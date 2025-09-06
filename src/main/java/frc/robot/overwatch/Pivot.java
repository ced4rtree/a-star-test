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
        0, 0
    );

    private PIDController pidController;

    private double pos;
    private double finalSetpoint;
    private double tentativeSetpoint;

    public final Trigger atFinalSetpoint =
        // new Trigger(() -> {
        //     // Takes care of the discontinuity around theta = pi or theta = -pi
        //     double delta = pos - setpoint;
        //     double continuousDelta = (delta + Math.PI) % 2*Math.PI - Math.PI;
        //     return MathUtil.isNear(0, continuousDelta, 0.02);
        // });
        new Trigger(() -> MathUtil.isNear(finalSetpoint, pos, 0.02));

    public final Trigger atTentativeSetpoint
        = new Trigger(() -> MathUtil.isNear(tentativeSetpoint, pos, 0.02));

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

    protected void applyVolts(double volts) {
        sim.setInput(volts);
    }

    protected void goTo(double angleRads) {
        var feedback = pidController.calculate(pos, angleRads);
        sim.setInput(feedback);
    }

    protected void setFinalSetpoint(double angleRads) {
        finalSetpoint = MathUtil.angleModulus(angleRads);
    }

    protected void setTentativeSetpoint(double angleRads) {
        tentativeSetpoint = MathUtil.angleModulus(angleRads);
    }
}
