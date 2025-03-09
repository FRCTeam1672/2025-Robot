// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static frc.robot.Constants.HomeConstants.CORAL_HOME_POSITION;
import static frc.robot.Constants.HomeConstants.CORAL_STOW_POSITION;
import static frc.robot.Constants.IOSpeeds.CORAL_IDLE_SPEED;
import static frc.robot.Constants.IOSpeeds.CORAL_INTAKE_SPEED;
import static frc.robot.Constants.IOSpeeds.CORAL_SHOOT_SPEED;
import static frc.robot.Constants.PIDConstants.C_WRIST_D;
import static frc.robot.Constants.PIDConstants.C_WRIST_I;
import static frc.robot.Constants.PIDConstants.C_WRIST_P;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSubsystem extends SubsystemBase {

  private SparkMax coralWrist = new SparkMax(31, MotorType.kBrushless);
  private SparkMax coralShooter = new SparkMax(32, MotorType.kBrushless);

  private SparkMaxConfig config = new SparkMaxConfig();

  private double coralWristPosition = CORAL_STOW_POSITION;

  public CoralSubsystem() {
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(20);
    coralShooter.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(20);
    config.closedLoop.pid(C_WRIST_P, C_WRIST_I, C_WRIST_D);
    config.closedLoop.maxOutput(0.15);
    config.closedLoop.minOutput(-0.15);
    coralWrist.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    setDefaultCommand(run(() -> coralShooter.set(CORAL_IDLE_SPEED)));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("coral/Coral Wrist Angle", coralWrist.getEncoder().getPosition());
    SmartDashboard.putNumber("coral/Coral Shooter Velocity", coralShooter.getEncoder().getVelocity());
    SmartDashboard.putNumber("coral/Coral Wrist Setpoint", coralWristPosition);
    SmartDashboard.putBoolean("coral/Coral atPosition", isCoralAtPosition());

    coralWrist.getClosedLoopController().setReference(coralWristPosition, ControlType.kPosition);
  }

  public boolean isCoralIntaked() {
    return coralShooter.get() > 0.0 && MathUtil.isNear(0, coralShooter.getEncoder().getVelocity(), 2000);
  }

  public boolean isCoralHomed() {
    return MathUtil.isNear(CORAL_HOME_POSITION, coralWrist.getEncoder().getPosition(), 0.5);
  }

  public boolean isCoralAtPosition() {
    return MathUtil.isNear(coralWristPosition, coralWrist.getEncoder().getPosition(), 0.5);
  }

  public Command homeCoral() {
    return runOnce(() -> {
      coralWristPosition = CORAL_HOME_POSITION;
    }).andThen(Commands.waitUntil(this::isCoralHomed));
  }

  public Command intakeCoral() {
    return run(() -> {
      coralShooter.set(CORAL_INTAKE_SPEED);
    }).until(this::isCoralIntaked);
  }

  public Command dumIntakeCoral() {
    return run(() -> {
      coralShooter.set(CORAL_INTAKE_SPEED);
    }).handleInterrupt(coralShooter::stopMotor);
  }

  
  public Command shootCoral() {
    return run(() -> {
        coralShooter.set(CORAL_SHOOT_SPEED);
    }).handleInterrupt(() -> {
        coralShooter.stopMotor();
    });
}

public Command coralTo(double pos) {
  return runOnce(() -> {
      coralWristPosition = pos;
  }).andThen(Commands.waitUntil(this::isCoralAtPosition));
}


}
