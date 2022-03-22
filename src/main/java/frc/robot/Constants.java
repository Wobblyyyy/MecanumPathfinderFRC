package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public class Constants {
    public static final int FL_MOTOR_ID = 1;
    public static final int FR_MOTOR_ID = 0;
    public static final int BL_MOTOR_ID = 3;
    public static final int BR_MOTOR_ID = 2;

    public static final double CHASSIS_SIZE_X = 12.0;
    public static final double CHASSIS_SIZE_Y = 12.0;

    public static final Translation2d FL_POS = new Translation2d(
        -CHASSIS_SIZE_X,
        CHASSIS_SIZE_Y
    );
    public static final Translation2d FR_POS = new Translation2d(
        CHASSIS_SIZE_X,
        CHASSIS_SIZE_Y
    );
    public static final Translation2d BL_POS = new Translation2d(
        -CHASSIS_SIZE_X,
        -CHASSIS_SIZE_Y
    );
    public static final Translation2d BR_POS = new Translation2d(
        CHASSIS_SIZE_X,
        -CHASSIS_SIZE_Y
    );

    public static final double ODOMETRY_UPDATE_INTERVAL_MS = 5;
    public static final double WHEEL_CIRCUMFERENCE = 0.5;

    public static final double TURN_COEFFICIENT = -0.05;

    public static final int LEFT_JOYSTICK_PORT = 0;
    public static final int RIGHT_JOYSTICK_PORT = 1;

    public static final int MULT_SLOW_BUTTON = 0;
    public static final int MULT_NORMAL_BUTTON = 1;
    public static final int MULT_FAST_BUTTON = 2;
}
