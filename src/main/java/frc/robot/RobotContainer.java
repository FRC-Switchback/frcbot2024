package frc.robot;



// import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.commands.FollowPathCommand;
// import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.chassis.DriveCommand;
import frc.robot.chassis.TankSubsystem;
import frc.robot.intake.IntakeCommands;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.shooter.ShootCommand;
import frc.robot.shooter.ShooterCommands;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.chassis.DriveTestCommand;


public class RobotContainer {
    private static final CommandXboxController driverController= new CommandXboxController(0);
    private static final CommandXboxController coDriverController= new CommandXboxController(1);

    //SendableChooser<Command> AutoChooser = new SendableChooser<>();

    //SUBSYSTEM
    TankSubsystem drive = TankSubsystem.getInstance();
    ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    IntakeSubsystem intake = IntakeSubsystem.getInstance();
    //COMMANDS
    Command driveCommand = new DriveCommand(driverController::getLeftY,driverController::getRightY);
    Command shootCommand = new ShootCommand(intake,shooter);
    Command intakeCommand = IntakeCommands.INTAKE;
    Command shooterAmpSpeed = ShooterCommands.SHOOTER_AMP_SPEED;
    Command shooterFullSpeed = ShooterCommands.SHOOTER_FULL_SPEED;
    Command stowCommand = IntakeCommands.STOW;
    //TRIGGERS
  

    public RobotContainer() {
        registerNamedCommands();
        configureBindings();
        drive.init();
        drive.setDefaultCommand(driveCommand);
        //SmartDashboard.putData("Autos", AutoChooser);
        SmartDashboard.putNumber("Drive Test Command Distance (rotations)", 0.5); // change this in Smart Dashboard to test accuracy over different distances
        SmartDashboard.putNumber("Drive Test Command Speed (percentage)", 0.5); // change this in Smart Dashboard to test accuracy over different speeds
        SmartDashboard.putData("Drive Test Command",
                new ProxyCommand(() -> new DriveTestCommand(
                        SmartDashboard.getNumber("Drive Test Command Distance (rotations)", 0.5),
                        SmartDashboard.getNumber("Drive Test Command Speed (percentage)", 0.5))));
    }

    private void configureBindings() {
        coDriverController.rightTrigger(0.5).onTrue(shootCommand);//right trigger shoots the note
        coDriverController.leftTrigger().whileTrue(intakeCommand);
        coDriverController.b().onTrue(new ParallelCommandGroup(stowCommand, shooterAmpSpeed));//b stows and brings shooter to the slower amp speed
                                                                                              // this shouldn't be needed since the intake command already stows when a note is detected
        coDriverController.a().onTrue(shooterAmpSpeed);//a sets flywheel to amp speed
        coDriverController.y().onTrue(shooterFullSpeed);//y sets full shooter speed
    }

    public void registerNamedCommands() {

    }

//    public Command getAutonomousCommand() {
//        //return AutoChooser.getSelected();
//    }


}
