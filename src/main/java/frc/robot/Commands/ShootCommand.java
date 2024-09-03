package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.StowCommand;
import frc.robot.IntakeSubsystem;
import frc.robot.ShooterSubsystem;

public class ShootCommand extends Command{

    private final IntakeSubsystem intakeSubsystem;
    private final ShooterSubsystem shooter;
    private boolean stop=false;

    public ShootCommand(IntakeSubsystem intakeSubsystem,ShooterSubsystem shooter){
        this.intakeSubsystem = intakeSubsystem; 
        this.shooter=shooter;
        addRequirements();
    }

    @Override 
    public void initialize(){
      intakeSubsystem.outtake();
      if(intakeSubsystem.hasNote())stop =true;
      
    }

    @Override 
    public boolean isFinished(){ 
        if(intakeSubsystem.hasNote()) return true;
        return stop;
    }

    @Override
    public void end(boolean interrupted){
            CommandScheduler.getInstance().schedule(new StowCommand(intakeSubsystem,shooter));
    }
}