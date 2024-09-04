package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TankSubsystem;


public class DriveTestCommand extends Command {

    private final TankSubsystem chassis = TankSubsystem.getInstance();
    private final double[] targetDistances;
    private final double speed;

    public DriveTestCommand(double distance, double speed) {
        addRequirements(chassis);
        double[] startDistance = chassis.getDrivePosition();
        this.targetDistances = new double[]{startDistance[0] + distance, startDistance[1] + distance};
        this.speed = speed;
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {

    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        chassis.drive(speed, speed);
    }

    /**
     * <p>
     * Returns whether this command has finished. Once a command finishes -- indicated by
     * this method returning true -- the scheduler will call its {@link #end(boolean)} method.
     * </p><p>
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand}
     * for such an operation.
     * </p>
     *
     * @return whether this command has finished.
     */
    @Override
    public boolean isFinished() {
        double[] drivePosition = chassis.getDrivePosition();
        return (drivePosition[0] >= targetDistances[0] && drivePosition[1] >= targetDistances[1]);
    }

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally -- that is it is called when {@link #isFinished()} returns
     * true -- or when  it is interrupted/canceled. This is where you may want to
     * wrap up loose ends, like shutting off a motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        chassis.drive(0, 0); // just to make sure it stops correctly
    }
}
