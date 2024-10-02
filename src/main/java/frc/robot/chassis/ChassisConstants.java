package frc.robot.chassis;

import com.pathplanner.lib.util.PIDConstants;

public class ChassisConstants {
    public static final double GEAR_RATIO = 1;
    public static final double WHEEL_CIRCUMFERENCE_METERS = 1;

    public static final double CHASSIS_WIDTH_METERS = 1;
    public static final PIDConstants DRIVE_PID = new PIDConstants(1,0,0);
}
