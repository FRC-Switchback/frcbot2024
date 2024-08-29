package frc.robot;



// import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.commands.FollowPathCommand;
// import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.DriveCommand;


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


  //TRIGGERS 
  

  public RobotContainer() {
    registerNamedCommands();
    configureBindings();
     drive.init();
     drive.setDefaultCommand(driveCommand);
    //SmartDashboard.putData("Autos", AutoChooser);
    
  }

  private void configureBindings() {
    
  }

  public void registerNamedCommands() {

  }

  public void setAutoCommands(){

  }

  // public Command getAutonomousCommand() {
  //   //return AutoChooser.getSelected(); 
  // }
  

}
