package frc.robot.overwatch;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
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

    private Rotation2d pos = new Rotation2d();
    private double posWrapped;
    private Rotation2d finalSetpoint = new Rotation2d();
    private Rotation2d tentativeSetpoint = new Rotation2d();

    public final Trigger atFinalSetpoint
        = new Trigger(() -> MathUtil.isNear(finalSetpoint.minus(pos).getRadians(), 0, 0.02));

    public final Trigger atTentativeSetpoint
        = new Trigger(() -> MathUtil.isNear(tentativeSetpoint.minus(pos).getRadians(), 0, 0.02));

    public Pivot() {
        pidController = new PIDController(40.0, 0.0, 2.2);
        pidController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void periodic() {
        sim.update(.02);
        pos = Rotation2d.fromRadians(sim.getAngleRads());
        posWrapped = MathUtil.angleModulus(pos.getRadians());
        // pos = sim.getAngleRads();
    }

    public Rotation2d getAngle() {
        return pos;
    }

    protected void applyVolts(double volts) {
        sim.setInput(volts);
    }

    protected void goTo(Rotation2d angle) {
        var feedback = pidController.calculate(pos.getRadians(), angle.getRadians());
        sim.setInput(feedback);
    }

    protected void setFinalSetpoint(Rotation2d angle) {
        finalSetpoint = angle;
    }

    protected void setTentativeSetpoint(Rotation2d angle) {
        tentativeSetpoint = angle;
    }
}
