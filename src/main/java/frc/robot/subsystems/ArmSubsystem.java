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
import static frc.robot.Constants.PIDConstants.A_WRIST_D;
import static frc.robot.Constants.PIDConstants.A_WRIST_I;
import static frc.robot.Constants.PIDConstants.A_WRIST_P;
import static frc.robot.Constants.PIDConstants.ELEVATOR_D;
import static frc.robot.Constants.PIDConstants.ELEVATOR_I;
import static frc.robot.Constants.PIDConstants.ELEVATOR_P;
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
    private SparkMax lElevator = new SparkMax(51, MotorType.kBrushless);
    private SparkMax rElevator = new SparkMax(52, MotorType.kBrushless);

    // Algae
    private SparkMax algaeWrist = new SparkMax(41, MotorType.kBrushless);
    private SparkMax lAlgaeIntake = new SparkMax(42, MotorType.kBrushless);
    private SparkMax rAlgaeIntake = new SparkMax(43, MotorType.kBrushless);

    private SparkMaxConfig config = new SparkMaxConfig();

    private double algaeWristPosition = ALGAE_HOME_POSITION;
    private double elevatorPosition = ELEVATOR_HOME_POSITION;

    private Trigger badElevTrigger = new Trigger(() -> !isElevatorGood());
    private Trigger badAlgaeTrigger = new Trigger(() -> !isAlgaeGood());

    private CoralSubsystem coral = new CoralSubsystem();

    public ArmSubsystem() {
        badElevTrigger.onTrue(Commands.runOnce(() -> {
            Elastic.sendNotification(
                    new Elastic.Notification(Elastic.Notification.NotificationLevel.ERROR, "Unbalanced Elevators",
                            "Elevator motors have gone out of sync, stopped elevators to not break elevator.")
                            .withDisplaySeconds(10));
        }));
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(20);
        lAlgaeIntake.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        config.inverted(true);
        rAlgaeIntake.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config.smartCurrentLimit(40);
        config.idleMode(IdleMode.kBrake);
        config.inverted(false);

        config.closedLoop.pid(ELEVATOR_P, ELEVATOR_I, ELEVATOR_D);
        config.closedLoop.maxOutput(0.45);
        config.closedLoop.minOutput(-0.45);
        rElevator.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        config.inverted(true);
        lElevator.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config.idleMode(IdleMode.kBrake);

        config.smartCurrentLimit(30);
        config.inverted(false);
        config.closedLoop.maxOutput(0.22);
        config.closedLoop.minOutput(-0.2);
        config.closedLoop.pid(A_WRIST_P, A_WRIST_I, A_WRIST_D);
        algaeWrist.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("algae/lAlgae Intake Velocity", lAlgaeIntake.getEncoder().getVelocity());
        SmartDashboard.putNumber("algae/rAlgae Intake Velocity", rAlgaeIntake.getEncoder().getVelocity());
        SmartDashboard.putNumber("algae/Algae Wrist Angle", algaeWrist.getEncoder().getPosition());
        SmartDashboard.putNumber("elevator/lElevator Height", lElevator.getEncoder().getPosition());
        SmartDashboard.putNumber("elevator/rElevator Height", rElevator.getEncoder().getPosition());
        SmartDashboard.putNumber("algae/Algae Wrist Setpoint", algaeWristPosition);
        SmartDashboard.putNumber("elevator/Elevator Setpoint", elevatorPosition);

        SmartDashboard.putBoolean("elevator/Elevator Safety", badElevTrigger.getAsBoolean());
        SmartDashboard.putBoolean("algae/Algae Safety", badAlgaeTrigger.getAsBoolean());

        SmartDashboard.putBoolean("elevator/Elevator atPosition", isElevatorAtPosition());
        SmartDashboard.putBoolean("algae/Algae atPosition", isAlgaeAtPosition());

        SmartDashboard.putNumber("algae/lAlgae.get", lAlgaeIntake.get());
        SmartDashboard.putNumber("algae/rAlgae.get", rAlgaeIntake.get());
        
        algaeWrist.getClosedLoopController().setReference(algaeWristPosition, ControlType.kPosition);

        if (!badElevTrigger.getAsBoolean()) {
            lElevator.getClosedLoopController().setReference(elevatorPosition, ControlType.kPosition);
            rElevator.getClosedLoopController().setReference(elevatorPosition, ControlType.kPosition);
        } else {
            lElevator.stopMotor();
            rElevator.stopMotor();
        }

        if (badAlgaeTrigger.getAsBoolean()) {
            algaeWrist.stopMotor();
        }
    }

    public boolean isElevatorGood() {
        return Math
                .abs(lElevator.getEncoder().getPosition() - rElevator.getEncoder().getPosition()) <= ELEVATOR_TOLERANCE;
    }

    public boolean isAlgaeGood() {
        return algaeWrist.getEncoder().getPosition() <= 8;
     }

    public boolean isCoralIntaked() {
        return coral.isCoralIntaked();
    }

    public boolean isAlgaeIntaked() {
        return lAlgaeIntake.get() > 0.0 && MathUtil.isNear(0, lAlgaeIntake.getEncoder().getVelocity(), 2000);
    }

    public boolean isElevatorHomed() {
        return MathUtil.isNear(ELEVATOR_HOME_POSITION,
                (lElevator.getEncoder().getPosition() + rElevator.getEncoder().getPosition()) / 2.0, 0.5);
    }

    public boolean isCoralHomed() {
        return coral.isCoralHomed();
    }

    public boolean isAlgaeHomed() {
        return MathUtil.isNear(ALGAE_HOME_POSITION, algaeWrist.getEncoder().getPosition(), 0.5);
    }

    public boolean isElevatorAtPosition() {
        return MathUtil.isNear(elevatorPosition,
                (lElevator.getEncoder().getPosition() + rElevator.getEncoder().getPosition()) / 2.0, 0.5);
    }

    public boolean isCoralAtPosition() {
        return coral.isCoralAtPosition();
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
        return coral.homeCoral();
    }

    public Command homeAlgae() {
        return Commands.runOnce(() -> {
            algaeWristPosition = ALGAE_HOME_POSITION;
        }).andThen(Commands.waitUntil(this::isAlgaeHomed));
    }

    public Command homeEverything() {
        return Commands.parallel(homeCoral(), homeElevator(), homeAlgae()).andThen(coralTo(CORAL_STOW_POSITION));
    }

    public Command homeNotAlgae() {
        return Commands.parallel(homeCoral(), homeElevator()).andThen(coralTo(CORAL_STOW_POSITION));
    }

    public Command intakeCoral() {
        return coral.intakeCoral();
    }

    public Command dumIntakeCoral() {
        return coral.dumIntakeCoral();
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
        return coral.shootCoral();
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
        return coral.coralTo(pos);
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
        return extendL1().andThen(Commands.waitSeconds(0.5).andThen(shootCoral().withTimeout(0.7)))
                .andThen(homeEverything());
    }

    public Command extendL2() {
        return extendElevatorTo(E_L2_POSITION).andThen(coralTo(C_L2_POSITION));
    }

    public Command extendTo(int x) {
        switch (x) {
            case 1:
                return extendL1();
            case 2:
                return extendL2();
            case 3:
                return extendL3();
            default:
                return Commands.none();
        }
    }

    public void zeroEncoders() {

    }

    public Command scoreL2() {
        return extendL2().andThen(Commands.waitSeconds(0.5).andThen(shootCoral().withTimeout(0.7)))
                .andThen(homeEverything());
    }

    public Command extendL3() {
        return extendElevatorTo(E_L3_POSITION).andThen(coralTo(C_L3_POSITION));
    }

    public Command scoreL3() {
        return extendL3().andThen(Commands.waitSeconds(0.3).andThen(shootCoral().withTimeout(0.5)))
                .andThen(homeEverything());
    }

    public Command scoreCoral(int level) {
        switch (level) {
            case 1:
                return scoreL1();
            case 2:
                return scoreL2();
            case 3:
                return scoreL3();
            default:
                return Commands.none();
        }
    }

    // public Command intakeStation() {
    // return extendElevatorTo(E_PROCESSOR_POSITION)
    // }

    public Command algaeL2() {
        return extendElevatorTo(E_AL2_POSITION).andThen(algaeTo(A_TILT_HIGH_POSITION));
    }

    public Command algaeL3() {
        return extendElevatorTo(E_AL3_POSITION).andThen(algaeTo(A_TILT_HIGH_POSITION));
    }

    public Command processor() {
        return extendElevatorTo(E_PROCESSOR_POSITION).andThen(algaeTo(A_IO_POSITION));
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
