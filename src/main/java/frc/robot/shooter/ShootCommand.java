package frc.robot.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.intake.IntakeCommands;
import frc.robot.intake.IntakeSubsystem;

import java.util.logging.Logger;

public class ShootCommand extends Command{

    private final IntakeSubsystem intakeSubsystem;
    private final ShooterSubsystem shooter;

    public ShootCommand(IntakeSubsystem intakeSubsystem,ShooterSubsystem shooter){
        this.intakeSubsystem = intakeSubsystem; 
        this.shooter=shooter;
        addRequirements();
    }

    @Override 
    public void initialize(){
        if (!intakeSubsystem.hasNote()) cancel();
        intakeSubsystem.passthrough();
    }

    @Override 
    public boolean isFinished(){ 
        return !intakeSubsystem.hasNote();
    }

    @Override
    public void end(boolean interrupted){
        CommandScheduler.getInstance().schedule(IntakeCommands.STOW);
    }
}