// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PracticeMotor;

public class PIDCommand extends Command {
  private final PracticeMotor motorSub;
  private final PIDController practicePID;
  private final double goal;
  private boolean done;

  public PIDCommand(double setPoint, PracticeMotor m_PracticeMotor) {
    this.motorSub = m_PracticeMotor;
    this.goal = setPoint;
    this.practicePID = new PIDController(0.03, 0.01, 0.003);

    addRequirements(motorSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    practicePID.setSetpoint(goal);
    practicePID.reset();
    practicePID.setTolerance(0.25);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = practicePID.calculate(motorSub.shooterAngle(), goal);
    done = practicePID.atSetpoint();

    motorSub.raisePID(speed);

     SmartDashboard.putBoolean("Arm Tolerance Check", practicePID.atSetpoint());
    SmartDashboard.putNumber("Arm Tolerance", practicePID.getPositionError());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    motorSub.raisePID(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done || motorSub.hardLimit() || motorSub.softLimit();
  }
}
