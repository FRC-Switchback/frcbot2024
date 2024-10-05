package frc.robot;


// import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.commands.FollowPathCommand;
// import com.pathplanner.lib.commands.PathPlannerAuto;

import com.ctre.phoenix6.jni.SignalLoggerJNI;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.chassis.DriveCommand;
import frc.robot.chassis.TankSubsystem;
import frc.robot.intake.IntakeCommands;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.shooter.ShootCommand;
import frc.robot.shooter.ShooterCommands;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.chassis.DriveTestCommand;
import org.w3c.dom.ls.LSOutput;


public class RobotContainer {
    private static final CommandXboxController driverController= new CommandXboxController(0);
    private static final CommandXboxController coDriverController= new CommandXboxController(1);

    SendableChooser<Command> autoChooser;
    //SUBSYSTEM
    TankSubsystem drive = TankSubsystem.getInstance();
    ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    IntakeSubsystem intake = IntakeSubsystem.getInstance();
    //COMMANDS
    Command driveCommand = new DriveCommand(driverController::getLeftY,driverController::getRightY);
    Command shootCommand = new ShootCommand(intake,shooter);

    public RobotContainer() {
        configureAutoBuilder();
        registerNamedCommands();
        configureBindings();
        initializeSubsystems();
        drive.setDefaultCommand(driveCommand);
        //SmartDashboard.putData("Autos", AutoChooser);
        SmartDashboard.putNumber("Drive Test Command Distance (rotations)", 0.5); // change this in Smart Dashboard to test accuracy over different distances
        SmartDashboard.putNumber("Drive Test Command Speed (percentage)", 0.5); // change this in Smart Dashboard to test accuracy over different speeds
        SmartDashboard.putData("Drive Test Command",
                new ProxyCommand(() -> new DriveTestCommand(
                        SmartDashboard.getNumber("Drive Test Command Distance (rotations)", 0.5),
                        SmartDashboard.getNumber("Drive Test Command Speed (percentage)", 0.5))));
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Autos", autoChooser);
    }

    void configureBindings() {
        coDriverController.rightBumper().onTrue(shootCommand);//right trigger shoots the note
        coDriverController.leftBumper().whileTrue(IntakeCommands.INTAKE.get()).onFalse(IntakeCommands.STOW.get());
        coDriverController.rightBumper().onTrue(IntakeCommands.STOW.get());
        coDriverController.y().onTrue(new ParallelCommandGroup(IntakeCommands.STOW.get(), ShooterCommands.SHOOTER_AMP_SPEED.get()));//y stows and brings shooter to the slower amp speed
                                                                                                                                    // this shouldn't be needed since the intake command already stows when a note is detected
        coDriverController.b().onTrue(ShooterCommands.SHOOTER_OFF.get());//a sets flywheel to amp speed
        coDriverController.a().whileTrue(ShooterCommands.SHOOTER_FULL_SPEED.get()).onFalse(ShooterCommands.SHOOTER_AMP_SPEED.get());//y sets full shooter speed
        coDriverController.x().whileTrue(IntakeCommands.EJECT.get());

//        SysIdRoutine intakeRoutine = intake.getsysIdRoutine();
//        driverController.povLeft().whileTrue(intakeRoutine.quasistatic(SysIdRoutine.Direction.kForward));
//        driverController.povRight().whileTrue(intakeRoutine.quasistatic(SysIdRoutine.Direction.kReverse));
//        driverController.povUp().whileTrue(intakeRoutine.dynamic(SysIdRoutine.Direction.kForward));
//        driverController.povDown().whileTrue(intakeRoutine.dynamic(SysIdRoutine.Direction.kReverse));
    }



    void initializeSubsystems() {
        drive.init();
        intake.init();
        shooter.init();
    }

    void registerNamedCommands() {
        NamedCommands.registerCommand("Shoot", shootCommand);
        NamedCommands.registerCommand("Intake", IntakeCommands.AUTO_INTAKE.get());
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
