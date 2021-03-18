/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team1918.robot.commands.shooter;

import frc.team1918.robot.Constants;
import frc.team1918.robot.Helpers;
import frc.team1918.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * A command that ...
 */
public class shooter_shootLine extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem m_shooter;
  // private int debug_ticks;

  /**
   * @param subsystem The subsystem used by this command.
   */
  public shooter_shootLine(ShooterSubsystem subsystem) {
    m_shooter = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Helpers.Debug.debug("Shooter: Shoot from Line");
    m_shooter.runFeeder(true);
    m_shooter.raiseHood(Constants.Shooter.SHOOT_LINE_HOOD);
    m_shooter.setShooterSpeed(Constants.Shooter.SHOOT_LINE_RPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // debug_ticks = Helpers.Debug.debug("Shooter (Line): Current RPM="+m_shooter.getShooterSpeed()+" Target="+Constants.Shooter.SHOOTER_LINE_RPM, debug_ticks);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.runFeeder(false);
    m_shooter.raiseHood(!Constants.Air.AIR_HOOD_UP);
    m_shooter.setShooterSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
