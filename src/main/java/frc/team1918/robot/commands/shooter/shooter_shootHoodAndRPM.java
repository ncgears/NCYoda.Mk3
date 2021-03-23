/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team1918.robot.commands.shooter;

import frc.team1918.robot.Helpers;
import frc.team1918.robot.subsystems.ShooterSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * A command that ...
 */
public class shooter_shootHoodAndRPM extends CommandBase {
  @SuppressWarnings({"unused","PMD.UnusedPrivateField", "PMD.SingularField"}) //Dont add "unused" under normal operation
  private final ShooterSubsystem m_shooter;
  private final BooleanSupplier m_hoodUp;
  private final DoubleSupplier m_speed;

  /**
   * @param subsystem The subsystem used by this command.
   */
  public shooter_shootHoodAndRPM(ShooterSubsystem subsystem, BooleanSupplier hoodUp, DoubleSupplier speed) {
    m_shooter = subsystem;
    m_hoodUp = hoodUp;
    m_speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Helpers.Debug.debug("Shooter: Shoot with settings: Hood Up="+m_hoodUp+" RPM="+m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
