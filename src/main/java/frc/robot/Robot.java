package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.chassis.Chassis;
import frc.robot.chassis.ChassisDrive;
import frc.robot.chassis.ChassisOdometry;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;
import me.wobblyyyy.pathfinder2.Pathfinder;
import me.wobblyyyy.pathfinder2.geometry.Angle;
import me.wobblyyyy.pathfinder2.geometry.PointXYZ;
import me.wobblyyyy.pathfinder2.geometry.Translation;
import me.wobblyyyy.pathfinder2.trajectory.LinearTrajectory;
import me.wobblyyyy.pathfinder2.trajectory.Trajectory;

public class Robot extends TimedRobot {
    private ChassisDrive drive;
    private ChassisOdometry odometry;
    private Chassis chassis;
    private Pathfinder pathfinder;

    private Joystick leftJoystick;
    private Joystick rightJoystick;

    private final AtomicReference<Double> multiplier = new AtomicReference<>(
        0d
    );

    @Override
    public void robotInit() {
        drive = new ChassisDrive();
        odometry = new ChassisOdometry(drive);
        chassis = new Chassis(drive, odometry);
        pathfinder = new Pathfinder(chassis, Constants.TURN_COEFFICIENT);

        leftJoystick = new Joystick(Constants.LEFT_JOYSTICK_PORT);
        rightJoystick = new Joystick(Constants.RIGHT_JOYSTICK_PORT);

        Supplier<Boolean> slowButton = () ->
            leftJoystick.getRawButton(Constants.MULT_SLOW_BUTTON);
        Supplier<Boolean> normalButton = () ->
            leftJoystick.getRawButton(Constants.MULT_NORMAL_BUTTON);
        Supplier<Boolean> fastButton = () ->
            leftJoystick.getRawButton(Constants.MULT_FAST_BUTTON);

        // add bindings:
        //
        // "slow" button      -     set the speed multiplier to 0.25
        // "normal" button    -     set the speed multiplier to 0.5
        // "fast" button      -     set the speed multiplier to 1.0
        //
        // so long as pathfinder.tick() is called regularly, these bindings
        // will be handled automatically
        //
        // the "bindButtonPress" method accepts two parameters:
        // - Consumer<Boolean>                   (the button)
        // - Runnable                            (action to bind to the button)
        pathfinder
            .getListenerManager()
            .bindButtonPress(slowButton, () -> multiplier.set(0.25))
            .bindButtonPress(slowButton, () -> multiplier.set(0.5))
            .bindButtonPress(slowButton, () -> multiplier.set(1.0));
    }

    @Override
    public void robotPeriodic() {
        pathfinder.tick();
    }

    @Override
    public void autonomousInit() {
        pathfinder.clear();

        Trajectory a = new LinearTrajectory(
            new PointXYZ(10, 10, Angle.fromDeg(45)),
            0.5,
            2.0,
            Angle.fromDeg(5)
        );

        Trajectory b = new LinearTrajectory(
            new PointXYZ(20, 10, Angle.fromDeg(90)),
            0.75,
            2.0,
            Angle.fromDeg(5)
        );

        pathfinder
            .followTrajectory(a)
            .followTrajectory(b)
            .splineTo(
                new PointXYZ(20, 15, Angle.fromDeg(0)),
                new PointXYZ(20, 15, Angle.fromDeg(0)),
                new PointXYZ(20, 15, Angle.fromDeg(0)),
                new PointXYZ(20, 15, Angle.fromDeg(0))
            )
            .tickUntil(10_000);
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        pathfinder.clear();
    }

    @Override
    public void teleopPeriodic() {
        // vx: leftwards/rightwards
        // vy: forwards/backwards
        // vz: rotation around center of robot

        double vx = leftJoystick.getX();
        double vy = leftJoystick.getY() * -1;
        double vz = rightJoystick.getX();

        Translation translation = new Translation(vx, vy, vz)
        .multiply(multiplier.get());

        pathfinder.setTranslation(translation);
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}
}
