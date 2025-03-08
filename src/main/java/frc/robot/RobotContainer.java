// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.ClimbConstants.CLIMB_SPEED;
import static frc.robot.Constants.ReefLevels.A_TILT_HIGH_POSITION;

import java.io.File;
import java.io.IOException;
import java.sql.Driver;
import java.util.Set;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

    // Replace with CommandPS4Controller or CommandJoystick if needed
    final CommandPS5Controller driverPS5 = new CommandPS5Controller(0);
    final CommandPS5Controller oppsPS5 = new CommandPS5Controller(1);
    // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve/neo"));
    private final ArmSubsystem arm = new ArmSubsystem();
    private final ClimbSubsystem climb = new ClimbSubsystem();
    private final ScoringApp scoringApp = ScoringApp.getInstance();

    private SendableChooser<Command> autoChooser;

    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled
     * by angular velocity.
     */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
            () -> -driverPS5.getLeftY(),
            () -> -driverPS5.getLeftX())
            .withControllerRotationAxis(() -> -driverPS5.getRightX())
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        // Configure the trigger bindings
        try {
            configureBindings();
        } catch (FileVersionException | IOException | ParseException e) {
            DriverStation.reportError("Could not configure button bindings. (Most likely a Pathplanner path issue!!!)",
                    e.getStackTrace());
            e.printStackTrace();
        }
        autoChooser = AutoBuilder.buildAutoChooser("AUTO-LEAVE");
        SmartDashboard.putData("Auto Chooser", autoChooser);

        NamedCommands.registerCommand("ScoreL2", arm.scoreL2().asProxy());
        NamedCommands.registerCommand("IntakeCoral", new ProxyCommand(
                arm.extendCoralStation().andThen(arm.dumIntakeCoral().withTimeout(2)).andThen(arm.homeEverything()))
        );

    }

    private void configureBindings() throws FileVersionException, IOException, ParseException {

        Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

        // Driver PS5

        driverPS5.cross().onTrue(arm.homeEverything());
        driverPS5.create().onTrue(Commands.runOnce(drivebase::zeroGyro));
        driverPS5.options().onTrue(Commands.runOnce(drivebase::lock, drivebase));

        driverPS5.povUp().whileTrue(arm.shootCoral());
        driverPS5.povDown().whileTrue(arm.dumIntakeCoral());
        driverPS5.povRight().whileTrue(arm.shootAlgae());
        driverPS5.povLeft().onTrue(arm.intakeAlgae());

        driverPS5.triangle().onTrue(arm.extendCoralStation());
        driverPS5.square().onTrue(arm.coralTo(5.8));
        driverPS5.circle().onTrue(arm.algaeL2());

        // CORAL AUTOSCORE
        driverPS5.R1().whileTrue(Commands.defer(() -> {
            try {
                System.out.println("Reef side " + scoringApp.getReefSide());
                System.out.println("Coral Level " + scoringApp.getCoralLevel());
                return drivebase.getPath("CORAL-" + scoringApp.getReefSide())
                        .andThen(arm.scoreCoral(scoringApp.getCoralLevel()));
            } catch (FileVersionException | IOException | ParseException e) {
                Elastic.sendNotification(new Notification().withLevel(NotificationLevel.ERROR)
                        .withTitle("Could not load pathplanner path for coral auto-scoring.")
                        .withDescription("Please use manual scoring.")
                        .withDisplaySeconds(10));
                DriverStation.reportError("Could not load pathplanner path!!", e.getStackTrace());
                e.printStackTrace();
                return Commands.none();
            }
        }, Set.of()).handleInterrupt(() -> {
            arm.homeEverything().schedule();
        }));

        // ALGAE AUTOSCORE
        driverPS5.L1().whileTrue(Commands.defer(
                () -> {
                    try {
                        System.out.println("Reef side " + scoringApp.getReefSide());
                        System.out.println("Algae Level " + scoringApp.getAlgaeLevel());
                        return drivebase.getPath("ALGAE-" + scoringApp.getReefSide())
                                .andThen(arm.ioAlgae(scoringApp.getAlgaeLevel()));
                    } catch (FileVersionException | IOException | ParseException e) {
                        Elastic.sendNotification(new Notification().withLevel(NotificationLevel.ERROR)
                                .withTitle("Could not load pathplanner path for algae auto-scoring.")
                                .withDescription("Please use manual scoring.")
                                .withDisplaySeconds(10));
                        DriverStation.reportError("Could not load pathplanner path!!", e.getStackTrace());
                        e.printStackTrace();
                        return Commands.none();
                    }
                }, Set.of()).handleInterrupt(() -> {
                    arm.homeEverything().schedule();
                }));
        // CORAL STATION
        driverPS5.R2().whileTrue(Commands.defer(
                () -> {
                    try {
                        System.out.println("Coral Station" + scoringApp.getCoralStation());
                        return drivebase.getPath("STATION-" + scoringApp.getCoralStation());
                        // .andThen(arm.extendCoralStation());
                    } catch (FileVersionException | IOException | ParseException e) {
                        Elastic.sendNotification(new Notification().withLevel(NotificationLevel.ERROR)
                                .withTitle("Could not load pathplanner path for coral station.")
                                .withDescription("Please use alignment.")
                                .withDisplaySeconds(10));
                        DriverStation.reportError("Could not load pathplanner path!!", e.getStackTrace());
                        e.printStackTrace();
                        return Commands.none();
                    }
                }, Set.of()).handleInterrupt(() -> {
                    arm.homeEverything().schedule();
                }));
        // driverPS5.R1().onTrue(arm.scoreL3());

        oppsPS5.create().onTrue(Commands.runOnce(drivebase::zeroGyro));
        oppsPS5.povLeft().onTrue(arm.processor());
        oppsPS5.povDown().onTrue(arm.extendL1());
        oppsPS5.povRight().onTrue(arm.extendL2());
        oppsPS5.povUp().onTrue(arm.extendL3());

        oppsPS5.circle().onTrue(arm.algaeL2());
        oppsPS5.triangle().onTrue(arm.algaeL3());
        oppsPS5.square().onTrue(arm.algaeTo(A_TILT_HIGH_POSITION));
        oppsPS5.cross().onTrue(arm.homeEverything());

        oppsPS5.R2().whileTrue(climb.simpleClimb());
        oppsPS5.L2().whileTrue(climb.simpleUnClimb());

        //FOR TESTING!!

        // oppsPS5.R2().whileTrue(climb.climbAtSpeed(() -> {
        //     return ((oppsPS5.getR2Axis() + 1.0) * 0.5) * CLIMB_SPEED;
        // }));
        // oppsPS5.L2().whileTrue(climb.unclimbAtSpeed(() -> {
        //     return ((oppsPS5.getL2Axis() + 1.0) * 0.5) * CLIMB_SPEED;
        // }));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return ((Command) autoChooser.getSelected());
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }
}
