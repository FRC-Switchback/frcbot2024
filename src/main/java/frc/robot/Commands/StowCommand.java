package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.IntakeSubsystem;
import frc.robot.ShooterSubsystem;

public class StowCommand extends Command{
    private final IntakeSubsystem subsystem;
    private final ShooterSubsystem shooter;
    public StowCommand(IntakeSubsystem subsystem,ShooterSubsystem shooter){
        this.subsystem=subsystem;
        this.shooter=shooter;
    }

    @Override
    public void execute(){
        subsystem.stow();
        shooter.shooterSpeedAmp();
    }
}
