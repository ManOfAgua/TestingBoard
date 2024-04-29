// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PracticeMotor;

public class MotorCommand extends Command {
private PracticeMotor motorSub;
static boolean toggle = false;
  /** Creates a new MotorCommand. */
  public MotorCommand(PracticeMotor m_limitSwitch) {
    this.motorSub = m_limitSwitch;
    addRequirements(m_limitSwitch);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    motorSub.raise();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    motorSub.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return motorSub.softLimit()||motorSub.hardLimit();

  }
}
