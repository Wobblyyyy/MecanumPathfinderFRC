package frc.robot.chassis;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SerialPort.Port;
import frc.robot.Constants;
import me.wobblyyyy.pathfinder2.geometry.PointXYZ;
import me.wobblyyyy.pathfinder2.kauailabs.AHRSGyro;
import me.wobblyyyy.pathfinder2.robot.AbstractOdometry;
import me.wobblyyyy.pathfinder2.robot.sensors.Gyroscope;
import me.wobblyyyy.pathfinder2.time.Time;
import me.wobblyyyy.pathfinder2.wpilib.WPIAdapter;

public class ChassisOdometry extends AbstractOdometry {
    private final MecanumDriveOdometry odometry;
    private final Gyroscope gyroscope;

    private final RelativeEncoder frontLeftEncoder;
    private final RelativeEncoder frontRightEncoder;
    private final RelativeEncoder backLeftEncoder;
    private final RelativeEncoder backRightEncoder;

    private PointXYZ position = PointXYZ.ZERO;
    private double lastUpdateTimeMs = 0;

    public ChassisOdometry(ChassisDrive drive) {
        this(drive, new AHRSGyro(Port.kMXP));
    }

    public ChassisOdometry(ChassisDrive drive, Gyroscope gyroscope) {
        odometry =
            new MecanumDriveOdometry(drive.getKinematics(), new Rotation2d());
        this.gyroscope = gyroscope;

        this.frontLeftEncoder = drive.getFrontLeft().getSpark().getEncoder();
        this.frontRightEncoder = drive.getFrontRight().getSpark().getEncoder();
        this.backLeftEncoder = drive.getBackLeft().getSpark().getEncoder();
        this.backRightEncoder = drive.getBackRight().getSpark().getEncoder();
    }

    private static final double velocityInchesPerSec(double rotationsPerMin) {
        double rotationsPerSec = rotationsPerMin / 60;
        double inchesPerSec = rotationsPerSec * Constants.WHEEL_CIRCUMFERENCE;

        return inchesPerSec;
    }

    private static final double velocityInchesPerSec(RelativeEncoder encoder) {
        return velocityInchesPerSec(encoder.getVelocity());
    }

    private MecanumDriveWheelSpeeds getWheelSpeeds() {
        double frontLeftSpeed = velocityInchesPerSec(frontLeftEncoder);
        double frontRightSpeed = velocityInchesPerSec(frontRightEncoder);
        double backLeftSpeed = velocityInchesPerSec(backLeftEncoder);
        double backRightSpeed = velocityInchesPerSec(backRightEncoder);

        return new MecanumDriveWheelSpeeds(
            frontLeftSpeed,
            frontRightSpeed,
            backLeftSpeed,
            backRightSpeed
        );
    }

    private boolean shouldUpdate() {
        double currentTimeMs = Time.ms();
        double elapsedTimeMs = currentTimeMs - lastUpdateTimeMs;

        return elapsedTimeMs > Constants.ODOMETRY_UPDATE_INTERVAL_MS;
    }

    private void update() {
        Rotation2d gyroAngle = WPIAdapter.rotationFromAngle(
            gyroscope.getAngle()
        );
        MecanumDriveWheelSpeeds wheelSpeeds = getWheelSpeeds();
        Pose2d pose = odometry.update(gyroAngle, wheelSpeeds);
        position = WPIAdapter.pointXYZFromPose(pose);
    }

    @Override
    public PointXYZ getRawPosition() {
        if (shouldUpdate()) {
            update();
        }

        return position;
    }
}
