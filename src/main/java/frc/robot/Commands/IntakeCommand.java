package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.IntakeSubsystem;
import frc.robot.ShooterSubsystem;

public class IntakeCommand extends Command{

    private final IntakeSubsystem intakeSubsystem;
    private final ShooterSubsystem shooter;

    public IntakeCommand(IntakeSubsystem intakeSubsystem,ShooterSubsystem shooter){
        this.intakeSubsystem = intakeSubsystem; 
        this.shooter=shooter;
        addRequirements(intakeSubsystem);
    }

    @Override 
    public boolean isFinished(){ 
        return !intakeSubsystem.hasNote(); 
    }

    @Override
    public void end(boolean interrupted){
      CommandScheduler.getInstance().schedule(new StowCommand(intakeSubsystem,shooter));
    }
}