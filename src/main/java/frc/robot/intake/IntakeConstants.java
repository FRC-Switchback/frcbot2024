package frc.robot.intake;

public class IntakeConstants {
    // MAKE SURE THE INTAKE CAN NOT GO PAST ITS 0
    // IF IT GOES PAST ITS 0 PID WILL FREAK OUT(sorry for all caps)
    public static final double DOWN_SETPOINT = 0;// tune irl
    public static final double STOW_SETPOINT = 0;// tune irl
    public static final double SETPOINT_TOLERANCE=40;// tune this until all the sequential commands run
                                                     // and don't get stuck bc of the tolerances

    public static final double[] ACTUATOR_PID = {0.1, 0.1, 0.1};// tune this irl
}
