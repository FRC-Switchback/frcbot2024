package frc.robot;


// import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.commands.FollowPathCommand;
// import com.pathplanner.lib.commands.PathPlannerAuto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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

    SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

    //SUBSYSTEM
    TankSubsystem drive = TankSubsystem.getInstance();
    ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    IntakeSubsystem intake = IntakeSubsystem.getInstance();
    //COMMANDS
    Command driveCommand = new DriveCommand(driverController::getLeftY,driverController::getRightY);
    Command shootCommand = new ShootCommand(intake,shooter);
    Command intakeCommand = IntakeCommands.INTAKE;
    Command autoIntakeCommand = IntakeCommands.AUTO_INTAKE;
    Command disableFlywheels = ShooterCommands.SHOOTER_OFF;
    Command shooterAmpSpeed = ShooterCommands.SHOOTER_AMP_SPEED;
    Command shooterFullSpeed = ShooterCommands.SHOOTER_FULL_SPEED;
    Command stowCommand = IntakeCommands.STOW;
    Command ejectCommand = IntakeCommands.EJECT;

    public RobotContainer() {
        registerNamedCommands();
        configureBindings();
        initializeSubsystems();
        configureAutoBuilder();
        drive.setDefaultCommand(driveCommand);
        SmartDashboard.putData("Autos", autoChooser);
        //SmartDashboard.putData("Autos", AutoChooser);
        SmartDashboard.putNumber("Drive Test Command Distance (rotations)", 0.5); // change this in Smart Dashboard to test accuracy over different distances
        SmartDashboard.putNumber("Drive Test Command Speed (percentage)", 0.5); // change this in Smart Dashboard to test accuracy over different speeds
        SmartDashboard.putData("Drive Test Command",
                new ProxyCommand(() -> new DriveTestCommand(
                        SmartDashboard.getNumber("Drive Test Command Distance (rotations)", 0.5),
                        SmartDashboard.getNumber("Drive Test Command Speed (percentage)", 0.5))));
    }

    void configureBindings() {
        coDriverController.rightBumper().onTrue(shootCommand);//right trigger shoots the note
        coDriverController.leftBumper().whileTrue(intakeCommand.andThen(stowCommand));
        coDriverController.y().onTrue(new ParallelCommandGroup(stowCommand, shooterAmpSpeed));//b stows and brings shooter to the slower amp speed
                                                                                              // this shouldn't be needed since the intake command already stows when a note is detected
        coDriverController.b().onTrue(disableFlywheels);//a sets flywheel to amp speed
        coDriverController.a().whileTrue(shooterFullSpeed).onFalse(shooterAmpSpeed);//y sets full shooter speed
        coDriverController.x().whileTrue(ejectCommand);
    }



    void initializeSubsystems() {
        drive.init();
        intake.init();
        shooter.init();
    }

    void registerNamedCommands() {
        NamedCommands.registerCommand("Shoot", shootCommand);
        NamedCommands.registerCommand("Intake", autoIntakeCommand);
    }

    void configureAutoBuilder() {
        AutoBuilder.configureRamsete(drive::getPose,
                drive::setPose,
                drive::getSpeeds,
                drive::driveChassisSpeeds,
                new ReplanningConfig(),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
                },
                drive);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public PathPlannerAuto getSelectedAuto() {
        return (PathPlannerAuto) autoChooser.getSelected();
    }
}
