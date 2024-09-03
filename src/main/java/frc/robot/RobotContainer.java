package frc.robot;



// import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.commands.FollowPathCommand;
// import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.DriveCommand;
import frc.robot.Commands.IntakeCommand;
import frc.robot.Commands.ShootCommand;
import frc.robot.Commands.ShooterAmpSpeed;
import frc.robot.Commands.ShooterFullSpeed;
import frc.robot.Commands.StowCommand;


public class RobotContainer {
  public static CommandXboxController driverController= new CommandXboxController(0); 
  public static CommandXboxController coDriverController= new CommandXboxController(1); 

  //SendableChooser<Command> AutoChooser = new SendableChooser<>();

  //SUBSYSTEM
  TankSubsystem drive=new TankSubsystem();
  ShooterSubsystem shooter=new ShooterSubsystem();
  IntakeSubsystem intake=new IntakeSubsystem();
  //COMMANDS
 DriveCommand driveCommand=new DriveCommand(driverController,drive);
 ShootCommand shootandout= new ShootCommand(intake,shooter);
 IntakeCommand intakeCommand=new IntakeCommand(intake,shooter);
 ShooterAmpSpeed shooterAmpSpeed=new ShooterAmpSpeed(shooter);
 ShooterFullSpeed shooterFullSpeed=new ShooterFullSpeed(shooter);
 StowCommand stowCommand=new StowCommand(intake,shooter);
  //TRIGGERS 
  

  public RobotContainer() {
    registerNamedCommands();
    configureBindings();
     drive.init();
     drive.setDefaultCommand(driveCommand);
    //SmartDashboard.putData("Autos", AutoChooser);
    
  }

  private void configureBindings() {
    coDriverController.rightTrigger().onTrue(shootandout);//right trigger shoots the note
    coDriverController.leftTrigger().onTrue(intakeCommand);//READ: left trigger intakes, since there is no way to know if we have a note please press stow as soon as the note is in enough
    coDriverController.b().onTrue(new ParallelCommandGroup(stowCommand, shooterAmpSpeed));//b stows and brings shooter to the slower amp speed
    coDriverController.a().onTrue(shooterAmpSpeed);//a sets flywheel to amp speed
    coDriverController.y().onTrue(shooterFullSpeed);//y sets full shooter speed

  }

  public void registerNamedCommands() {

  }

  public void setAutoCommands(){

  }

  // public Command getAutonomousCommand() {
  //   //return AutoChooser.getSelected(); 
  // }
  

}
