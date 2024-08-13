package frc.robot;


import java.util.function.BooleanSupplier;

// import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.commands.FollowPathCommand;
// import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.


wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.DriveCommand;


public class RobotContainer {

  public static CommandXboxController driverController= new CommandXboxController(0); 
  public static CommandXboxController coDriverController= new CommandXboxController(1); 

  SendableChooser<Command> AutoChooser = new SendableChooser<>();

  //SUBSYSTEM
  TankSubsystem drive=new TankSubsystem();

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
