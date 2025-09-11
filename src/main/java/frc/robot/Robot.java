// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.overwatch.Overwatch;
import frc.robot.overwatch.Graph.Node;
import frc.robot.overwatch.Overwatch.RotationType;

@Logged
public class Robot extends TimedRobot {
    Overwatch overwatch;

    CommandXboxController controller = new CommandXboxController(0);

    public Robot() {
        overwatch = new Overwatch();
        
        Epilogue.bind(this);

        controller.a().onTrue(overwatch.goTo(Node.HOME, RotationType.SHORTEST));
        controller.b().onTrue(overwatch.goTo(Node.FOO, RotationType.SHORTEST));
        controller.x().onTrue(overwatch.goTo(Node.BAR, RotationType.SHORTEST));
        controller.y().onTrue(overwatch.goTo(Node.HOME, RotationType.COUNTER_CLOCKWISE));

        DriverStation.silenceJoystickConnectionWarning(true);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
