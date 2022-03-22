package frc.robot.chassis;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import frc.robot.Constants;
import me.wobblyyyy.pathfinder2.geometry.Translation;
import me.wobblyyyy.pathfinder2.revrobotics.SparkMaxMotor;
import me.wobblyyyy.pathfinder2.robot.ImprovedAbstractDrive;
import me.wobblyyyy.pathfinder2.wpilib.WPIAdapter;

public class ChassisDrive extends ImprovedAbstractDrive {
    private final MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
        Constants.FL_POS,
        Constants.FR_POS,
        Constants.BL_POS,
        Constants.BR_POS
    );

    private final SparkMaxMotor frontLeft;
    private final SparkMaxMotor frontRight;
    private final SparkMaxMotor backLeft;
    private final SparkMaxMotor backRight;

    public ChassisDrive() {
        this(
            SparkMaxMotor.brushless(Constants.FL_MOTOR_ID),
            SparkMaxMotor.brushless(Constants.FR_MOTOR_ID),
            SparkMaxMotor.brushless(Constants.BL_MOTOR_ID),
            SparkMaxMotor.brushless(Constants.BR_MOTOR_ID)
        );
    }

    public ChassisDrive(
        SparkMaxMotor frontLeft,
        SparkMaxMotor frontRight,
        SparkMaxMotor backLeft,
        SparkMaxMotor backRight
    ) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
    }

    @Override
    public void abstractSetTranslation(Translation translation) {
        ChassisSpeeds chassisSpeeds = WPIAdapter.speedsFromTranslation(
            translation
        );
        MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(
            chassisSpeeds
        );

        frontLeft.setPower(wheelSpeeds.frontLeftMetersPerSecond);
        frontRight.setPower(wheelSpeeds.frontRightMetersPerSecond);
        backLeft.setPower(wheelSpeeds.rearLeftMetersPerSecond);
        backRight.setPower(wheelSpeeds.rearRightMetersPerSecond);
    }

    public MecanumDriveKinematics getKinematics() {
        return kinematics;
    }

    public SparkMaxMotor getFrontLeft() {
        return frontLeft;
    }

    public SparkMaxMotor getFrontRight() {
        return frontRight;
    }

    public SparkMaxMotor getBackLeft() {
        return backLeft;
    }

    public SparkMaxMotor getBackRight() {
        return backRight;
    }
}
