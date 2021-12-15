/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team1918.robot.commandgroups;

// import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.team1918.robot.subsystems.DriveSubsystem;
import frc.team1918.robot.commands.drive.drive_lockDriveControls;
import frc.team1918.robot.commands.drive.drive_resetAbsEnc;
import frc.team1918.robot.commands.drive.drive_moveAllToHomes;
import frc.team1918.robot.commands.drive.drive_resetGyro;
import frc.team1918.robot.commands.drive.drive_resetOdometry;

public class cg_drive_autoHome extends SequentialCommandGroup {
  private final DriveSubsystem m_drive;

  /**
   * This command group is the sequence for homing the swerve modules to their respective home positions. 
   * This depends on having been previously calibrated, which saves the home position values to the roborio in a file defined by Constants.DriveTrain.DT_HOMES_FILE
   * <ol>
   * <li>lock drive controls</li>
   * <li>move all swerver modules to home</li>
   * <li>unlock drive controls</li>
   * </ol>
   * <br>
   * @param subsystem The subsystem this command will run on.
  */
  public cg_drive_autoHome(DriveSubsystem subsystem) {
    m_drive = subsystem;
    addRequirements(m_drive);

    /**
     * Creates a sequential command group with the objects to run in sequence.
     * Can also include complex things like other command groups or parallel command groups
     */
    addCommands(
        //this is a comma separated list of commands, thus, the last one should not have a comma
        new drive_lockDriveControls(m_drive, () -> true),
        //new drive_resetAbsEnc(m_drive),
        //new drive_moveAllToHomes(m_drive),
        new drive_resetGyro(m_drive),
        new drive_resetOdometry(m_drive),
        new drive_lockDriveControls(m_drive, () -> false)
    );
  }
}