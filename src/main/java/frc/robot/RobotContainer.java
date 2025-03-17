// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.ReefLevels.A_TILT_HIGH_POSITION;

import java.io.File;
import java.io.IOException;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.*;
import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
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
            () -> -driverPS5.getLeftY() * (isSlowMode() ? 0.3 : 1),
            () -> -driverPS5.getLeftX() * (isSlowMode() ? 0.3 : 1))
            .withControllerRotationAxis(() -> -driverPS5.getRightX() * (isSlowMode() ? 0.3 : 1))
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);


    public boolean isSlowMode() {
        return driverPS5.L1().getAsBoolean();
    }

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        NamedCommands.registerCommand("AlignL2_I", getAutoAlignScoreCommand("I"));
        NamedCommands.registerCommand("AlignL2_J", getAutoAlignScoreCommand("J"));
        NamedCommands.registerCommand("HomeEverything", arm.homeEverything());
        NamedCommands.registerCommand("ExtendL2", arm.extendL2().asProxy());
        NamedCommands.registerCommand("ExtendStation", arm.extendCoralStation().asProxy());
        NamedCommands.registerCommand("ScoreL2", arm.scoreL2(false).asProxy());
        NamedCommands.registerCommand("IntakeCoral",
                                                                                                        //schedule command runs in the background
                arm.extendCoralStation().andThen(arm.dumIntakeCoral().withTimeout(1)).andThen(arm.homeEverything().asProxy(), Commands.print("COMPLETED INTAKE!"))
        );
        // SmartDashboard.putData("Home Everything", Commands.runOnce)        
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
        SmartDashboard.putData("Zero Coral", arm.zeroCoralWrist().ignoringDisable(true));
    }

    private Command getAutoAlignScoreCommand(String side) {
        return Commands.defer(() -> drivebase.alignToAndExtend(side, arm.extendL2()).andThen(arm.scoreL2(false)), Set.of());
    }

    private void configureBindings() throws FileVersionException, IOException, ParseException {

        Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

        // Driver PS5

        driverPS5.create().onTrue(Commands.runOnce(drivebase::zeroGyro));
        driverPS5.options().onTrue(Commands.runOnce(drivebase::lock, drivebase));


        //L1 IS USED FOR THE SLOW MODE
        driverPS5.L1().whileTrue(arm.dumIntakeCoral());
        driverPS5.L2().whileTrue(arm.dumIntakeAlgae());


        driverPS5.cross().onTrue(arm.homeEverything().ignoringDisable(true));
        driverPS5.triangle().onTrue(arm.extendCoralStation());
        driverPS5.square().onTrue(arm.coralTo(5.8));
        driverPS5.circle().whileTrue(arm.shootCoral());

        driverPS5.povLeft().whileTrue(arm.shootAlgae());
        driverPS5.povRight().onTrue(arm.algaeL2());
        driverPS5.povUp().onTrue(arm.algaeL3());
        driverPS5.povDown().onTrue(arm.homeWithAlgae());

        // CORAL AUTOSCORE (extending rn)
        driverPS5.R1().whileTrue(Commands.defer(() -> {
            try {
                System.out.println("Reef side " + scoringApp.getReefSide());
                System.out.println("Coral Level " + scoringApp.getCoralLevel());
                return drivebase.alignToAndExtend(scoringApp.getReefSide(), arm.extendTo(scoringApp.getCoralLevel()))
                        .andThen(arm.extendTo(scoringApp.getCoralLevel()));
                // arm.scoreCoral(scoringApp.getCoralLevel())
            } catch (FileVersionException e) {
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

        // CORAL STATION
        driverPS5.R2().whileTrue(Commands.defer(() -> {
                    try {
                        System.out.println("Coral Station" + scoringApp.getCoralStation());
                        return drivebase.getPath("STATION-" + scoringApp.getCoralStation());
                        // .andThen(arm.extendCoralStation());a
                    } catch (FileVersionException | IOException | ParseException e) {
                        Elastic.sendNotification(new Notification().withLevel(NotificationLevel.ERROR)
                                .withTitle("Could not load pathplanner path for coral station.")
                                .withDescription("Please use alignment.")
                                .withDisplaySeconds(10));
                        DriverStation.reportError("Could not load pathplanner path!!", e.getStackTrace());
                        e.printStackTrace();
                        return Commands.none();
                    }
                }, Set.of()).andThen(arm.extendCoralStation()).handleInterrupt(() -> {
                    arm.homeEverything().schedule();
        }));

        oppsPS5.options().onTrue(Commands.runOnce(drivebase::zeroGyro));

        oppsPS5.povDown().onTrue(arm.processor());
        oppsPS5.povLeft().onTrue(arm.algaeTo(A_TILT_HIGH_POSITION));
        oppsPS5.povRight().onTrue(arm.algaeL2());
        oppsPS5.povUp().onTrue(arm.algaeL3());

        oppsPS5.circle().onTrue(arm.extendL2());
        oppsPS5.triangle().onTrue(arm.extendL3());
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
