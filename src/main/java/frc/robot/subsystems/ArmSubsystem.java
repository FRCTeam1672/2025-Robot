package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static frc.robot.Constants.HomeConstants.ALGAE_HOME_POSITION;
import static frc.robot.Constants.HomeConstants.CORAL_HOME_POSITION;
import static frc.robot.Constants.HomeConstants.CORAL_STOW_POSITION;
import static frc.robot.Constants.HomeConstants.ELEVATOR_HOME_POSITION;
import static frc.robot.Constants.IOSpeeds.ALGAE_INTAKE_SPEED;
import static frc.robot.Constants.IOSpeeds.ALGAE_SHOOT_SPEED;
import static frc.robot.Constants.IOSpeeds.CORAL_INTAKE_SPEED;
import static frc.robot.Constants.IOSpeeds.CORAL_SHOOT_SPEED;
import static frc.robot.Constants.ElevPID.A_WRIST_D;
import static frc.robot.Constants.ElevPID.A_WRIST_I;
import static frc.robot.Constants.ElevPID.A_WRIST_P;
import static frc.robot.Constants.ElevPID.C_WRIST_D;
import static frc.robot.Constants.ElevPID.C_WRIST_I;
import static frc.robot.Constants.ElevPID.C_WRIST_P;
import static frc.robot.Constants.ElevPID.ELEVATOR_D;
import static frc.robot.Constants.ElevPID.ELEVATOR_I;
import static frc.robot.Constants.ElevPID.ELEVATOR_P;
import static frc.robot.Constants.ReefLevels.A_IO_POSITION;
import static frc.robot.Constants.ReefLevels.A_TILT_HIGH_POSITION;
import static frc.robot.Constants.ReefLevels.C_L1_POSITION;
import static frc.robot.Constants.ReefLevels.C_L2_POSITION;
import static frc.robot.Constants.ReefLevels.C_L3_POSITION;
import static frc.robot.Constants.ReefLevels.C_STATION_POSITION;
import static frc.robot.Constants.ReefLevels.E_AL2_POSITION;
import static frc.robot.Constants.ReefLevels.E_AL3_POSITION;
import static frc.robot.Constants.ReefLevels.E_L1_POSITION;
import static frc.robot.Constants.ReefLevels.E_L2_POSITION;
import static frc.robot.Constants.ReefLevels.E_L3_POSITION;
import static frc.robot.Constants.ReefLevels.E_OFFSET;
import static frc.robot.Constants.ReefLevels.E_PROCESSOR_POSITION;
import static frc.robot.Constants.ReefLevels.E_STATION_POSITION;
import static frc.robot.Constants.Tolerances.ELEVATOR_TOLERANCE;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.Elastic;

public class ArmSubsystem extends SubsystemBase {
    private final SparkMax lElevator = new SparkMax(51, MotorType.kBrushless);
    private final SparkMax rElevator = new SparkMax(52, MotorType.kBrushless);

    private final SparkMax coralWrist = new SparkMax(31, MotorType.kBrushless);
    private final SparkMax coralShooter = new SparkMax(32, MotorType.kBrushless);

    // Algae
    private final SparkMax algaeWrist = new SparkMax(41, MotorType.kBrushless);
    private final SparkMax lAlgaeIntake = new SparkMax(42, MotorType.kBrushless);
    private final SparkMax rAlgaeIntake = new SparkMax(43, MotorType.kBrushless);

    private double algaeWristPosition = ALGAE_HOME_POSITION;
    private double coralWristPosition = CORAL_STOW_POSITION;
    private double elevatorPosition = ELEVATOR_HOME_POSITION;

    private double elevOffset = 0;

    private final Trigger badElevTrigger = new Trigger(() -> !isElevatorGood());
    private final Trigger badAlgaeTrigger = new Trigger(() -> !isAlgaeGood());

    public boolean elevOverride = false;
    public boolean algaeOverride = false;

    public ArmSubsystem() {
        badElevTrigger.onTrue(Commands.runOnce(() -> {
            Elastic.sendNotification(
                    new Elastic.Notification(Elastic.Notification.NotificationLevel.ERROR, "Unbalanced Elevators",
                            "Elevator motors have gone out of sync, stopped elevators to not break elevator.").withDisplaySeconds(10));
        }));
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(20);
        coralShooter.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        lAlgaeIntake.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        config.inverted(true);
        rAlgaeIntake.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        //TODO: CHANEG
        config.smartCurrentLimit(40);
        config.idleMode(IdleMode.kBrake);
        config.inverted(false);

        config.closedLoop.pid(ELEVATOR_P, ELEVATOR_I, ELEVATOR_D);
        config.closedLoop.maxOutput(0.45);
        config.closedLoop.minOutput(-0.45);
        rElevator.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        config.inverted(true);
        lElevator.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //TODO CHANGE
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(20);
        config.closedLoop.pid(C_WRIST_P, C_WRIST_I, C_WRIST_D);
        config.closedLoop.maxOutput(0.1);
        config.closedLoop.minOutput(-0.15);
        coralWrist.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config.smartCurrentLimit(30);
        config.inverted(false);
        config.closedLoop.maxOutput(0.3);
        config.closedLoop.minOutput(-0.2);
        config.closedLoop.pid(A_WRIST_P, A_WRIST_I, A_WRIST_D);
        algaeWrist.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // setDefaultCommand(run(
        //         () -> {
        //             coralShooter.set(-0.07);
        //         }
        // ));
    }

    public Command addElevOffset() {
        return Commands.runOnce(() -> {
            elevOffset += E_OFFSET;
        });
    }
    public Command subtractElevOffset() {
        return Commands.runOnce(() -> {
            elevOffset -= E_OFFSET;
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("coral/Coral Shooter Velocity", coralShooter.getEncoder().getVelocity());
        SmartDashboard.putNumber("algae/lAlgae Intake Velocity", lAlgaeIntake.getEncoder().getVelocity());
        SmartDashboard.putNumber("algae/rAlgae Intake Velocity", rAlgaeIntake.getEncoder().getVelocity());
        SmartDashboard.putNumber("coral/Coral Wrist Angle", coralWrist.getEncoder().getPosition());
        SmartDashboard.putNumber("algae/Algae Wrist Angle", algaeWrist.getEncoder().getPosition());
        SmartDashboard.putNumber("elevator/lElevator Height", lElevator.getEncoder().getPosition());
        SmartDashboard.putNumber("elevator/rElevator Height", rElevator.getEncoder().getPosition());
        SmartDashboard.putNumber("coral/Coral Wrist Setpoint", coralWristPosition);
        SmartDashboard.putNumber("algae/Algae Wrist Setpoint", algaeWristPosition);
        SmartDashboard.putNumber("elevator/Elevator Setpoint", elevatorPosition);
        SmartDashboard.putNumber("elevator/Elevator Offset Amount", elevOffset);

        SmartDashboard.putBoolean("elevator/Elevator Safety", badElevTrigger.getAsBoolean());
        SmartDashboard.putBoolean("algae/Algae Safety", badAlgaeTrigger.getAsBoolean());

        SmartDashboard.putBoolean("elevator/Elevator atPosition", isElevatorAtPosition());
        SmartDashboard.putBoolean("algae/Algae atPosition", isAlgaeAtPosition());
        SmartDashboard.putBoolean("coral/Coral atPosition", isCoralAtPosition());

        SmartDashboard.putNumber("algae/lAlgae.get", lAlgaeIntake.get());
        SmartDashboard.putNumber("algae/rAlgae.get", rAlgaeIntake.get());


        coralWrist.getClosedLoopController().setReference(coralWristPosition, ControlType.kPosition);
        algaeWrist.getClosedLoopController().setReference(algaeWristPosition, ControlType.kPosition);

        if (!badElevTrigger.getAsBoolean() && !elevOverride) {
            lElevator.getClosedLoopController().setReference(elevatorPosition + elevOffset, ControlType.kPosition);
            rElevator.getClosedLoopController().setReference(elevatorPosition + elevOffset, ControlType.kPosition);
        } else {
            lElevator.stopMotor();
            rElevator.stopMotor();
        }

        if (badAlgaeTrigger.getAsBoolean() && !algaeOverride) {
            algaeWrist.stopMotor();
        }
    }

    public Command zeroCoralWrist() {
        return Commands.runOnce(() -> {
            coralWrist.getEncoder().setPosition(0);
        });
    }

    public boolean isElevatorGood() {
        return Math
                .abs(lElevator.getEncoder().getPosition() - rElevator.getEncoder().getPosition()) <= ELEVATOR_TOLERANCE;
    }

    public boolean isAlgaeGood() {
        return algaeWrist.getEncoder().getPosition() <= 11;
    }

    public boolean isCoralIntaked() {
        return coralShooter.get() > 0.0 && MathUtil.isNear(0, coralShooter.getEncoder().getVelocity(), 2000);
    }

    public boolean isAlgaeIntaked() {
        return lAlgaeIntake.get() > 0.0 && MathUtil.isNear(0, lAlgaeIntake.getEncoder().getVelocity(), 2000);
    }

    public boolean isElevatorHomed() {
        return MathUtil.isNear(ELEVATOR_HOME_POSITION + elevOffset,
                (lElevator.getEncoder().getPosition() + rElevator.getEncoder().getPosition()) / 2.0, 0.5);
    }

    public boolean isCoralHomed() {
        return MathUtil.isNear(CORAL_HOME_POSITION, coralWrist.getEncoder().getPosition(), 0.5);
    }

    public boolean isAlgaeHomed() {
        return MathUtil.isNear(ALGAE_HOME_POSITION, algaeWrist.getEncoder().getPosition(), 0.5);
    }

    public boolean isElevatorAtPosition() {
        return MathUtil.isNear(elevatorPosition + elevOffset,
                (lElevator.getEncoder().getPosition() + rElevator.getEncoder().getPosition()) / 2.0, 0.5);
    }

    public boolean isCoralAtPosition() {
        return MathUtil.isNear(coralWristPosition, coralWrist.getEncoder().getPosition(), 0.5);
    }

    public boolean isAlgaeAtPosition() {
        return MathUtil.isNear(algaeWristPosition, algaeWrist.getEncoder().getPosition(), 0.5);
    }

    public Command homeElevator() {
        return Commands.runOnce(() -> {
            elevatorPosition = ELEVATOR_HOME_POSITION;
        }).andThen(Commands.waitUntil(this::isElevatorHomed));
    }

    public Command homeCoral() {
        return Commands.runOnce(() -> {
            coralWristPosition = CORAL_HOME_POSITION;
        }).andThen(Commands.waitUntil(this::isCoralHomed));
    }

    public Command homeAlgae() {
        return Commands.runOnce(() -> {
            algaeWristPosition = ALGAE_HOME_POSITION;
        }).andThen(Commands.waitUntil(this::isAlgaeHomed));
    }

    public Command homeEverything() {
        return Commands.parallel(homeCoral(), homeElevator(), homeAlgae()).andThen(coralTo(CORAL_STOW_POSITION));
    }

    public Command homeWithAlgae() {
        return Commands.parallel(homeCoral(), homeElevator()).andThen(coralTo(CORAL_STOW_POSITION));
    }

    public Command intakeCoral() {
        return run(() -> {
            coralShooter.set(CORAL_INTAKE_SPEED);
        }).until(this::isCoralIntaked);
    }

    public Command dumIntakeCoral() {
        return Commands.run(() -> {
            coralShooter.set(CORAL_INTAKE_SPEED);
        }).handleInterrupt(coralShooter::stopMotor);
    }

    public Command intakeAlgae() {
        return Commands.run(() -> {
            lAlgaeIntake.set(ALGAE_INTAKE_SPEED);
            rAlgaeIntake.set(ALGAE_INTAKE_SPEED);
        }).until(this::isAlgaeIntaked);
    }

    public Command dumIntakeAlgae() {
        return Commands.run(() -> {
            lAlgaeIntake.set(ALGAE_INTAKE_SPEED);
            rAlgaeIntake.set(ALGAE_INTAKE_SPEED);
        }).handleInterrupt(() -> {
            rAlgaeIntake.stopMotor();
            lAlgaeIntake.stopMotor();
        });
    }

    public Command shootCoral() {
        return Commands.run(() -> {
            coralShooter.set(CORAL_SHOOT_SPEED);
        }).handleInterrupt(coralShooter::stopMotor);
    }

    public Command shootAlgae() {
        return Commands.run(() -> {
            lAlgaeIntake.set(ALGAE_SHOOT_SPEED);
            rAlgaeIntake.set(ALGAE_SHOOT_SPEED);
        }).handleInterrupt(() -> {
            lAlgaeIntake.stopMotor();
            rAlgaeIntake.stopMotor();
        });
    }

    public Command extendElevatorTo(double pos) {
        return coralTo(CORAL_HOME_POSITION).andThen(Commands.runOnce(() -> {
            elevatorPosition = pos;
        }).andThen(Commands.waitUntil(this::isElevatorAtPosition)));
    }

    public Command coralTo(double pos) {
        return Commands.runOnce(() -> {
            coralWristPosition = pos;
        }).andThen(Commands.waitUntil(this::isCoralAtPosition));
    }

    public Command algaeTo(double pos) {
        return Commands.runOnce(() -> {
            algaeWristPosition = pos;
        }).andThen(Commands.waitUntil(this::isAlgaeAtPosition));
    }

    public Command extendL1() {
        return extendElevatorTo(E_L1_POSITION).andThen(coralTo(C_L1_POSITION));
    }

    public Command scoreL1() {
        return extendL1().andThen(Commands.waitSeconds(0.5).andThen(shootCoral().withTimeout(0.7))).andThen(homeEverything());
    }

    public Command extendL2() {
        return Commands.parallel(
            extendElevatorTo(E_L2_POSITION),
            homeAlgae(),
            Commands.waitUntil(() -> isCoralAtPosition()).andThen(coralTo(C_L2_POSITION))
        );
    }

    public Command extendTo(int x) {
        return switch (x) {
            case 1 -> extendL1();
            case 2 -> extendL2();
            case 3 -> extendL3();
            default -> Commands.none();
        };
    }

    public Command scoreL2(boolean home) {
        return shootCoral().withTimeout(0.4).andThen(homeEverything().onlyIf(() -> home));
    }

    public Command extendL3() {
        return Commands.parallel(
             extendElevatorTo(E_L3_POSITION),
             homeAlgae(),
             Commands.waitUntil(() -> isCoralAtPosition()).andThen(coralTo(C_L3_POSITION))
         );
    }

    public Command scoreL3(boolean home) {
        return extendL3().andThen(Commands.waitSeconds(0.3).andThen(shootCoral().withTimeout(0.5))).andThen(homeEverything()).onlyIf(() -> home);
    }

    public Command scoreCoral(int level, boolean home) {
        switch (level) {
            case 1:
                return scoreL1();
            case 2:
                return scoreL2(home);
            case 3:
                return scoreL3(home);
            default:
                return Commands.none();
        }
    }

    // public Command intakeStation() {
    //     return extendElevatorTo(E_PROCESSOR_POSITION)
    // }

    public Command algaeL2() {
        return extendElevatorTo(E_AL2_POSITION).alongWith(algaeTo(A_TILT_HIGH_POSITION));
    }

    public Command algaeL3() {
        return extendElevatorTo(E_AL3_POSITION).alongWith(algaeTo(A_TILT_HIGH_POSITION));
    }

    public Command processor() {
        return extendElevatorTo(E_PROCESSOR_POSITION).alongWith(algaeTo(A_IO_POSITION));
    }

    public Command extendCoralStation() {
        return extendElevatorTo(E_STATION_POSITION).andThen(coralTo(C_STATION_POSITION));
    }

    public Command ioAlgae(int level) {
        switch (level) {
            case 0:
                return processor();
            case 2:
                return algaeL2();
            case 3:
                return algaeL3();
            default:
                return Commands.none();
        }
    }
}
