
package frc.team1918.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import constants and subsystem
import frc.team1918.robot.Constants;
import frc.team1918.robot.Helpers;
import frc.team1918.robot.subsystems.ClimberSubsystem;

/**
 * A command that runs the climb actions. This passes the OI inputs on to the appropriate climber system
 */
public class climber_defaultClimb extends CommandBase {
  private final ClimberSubsystem m_climber;

  /**
   * Creates a new climber_defaultClimb.
   *
   * @param subsystem The climber subsystem this command wil run on.
   * @param climb The control input for raising/lowering the climber
   */
  public climber_defaultClimb(ClimberSubsystem subsystem) {
    m_climber = subsystem;
    addRequirements(m_climber);
  }

  @Override
  public void execute() {
    if (!m_climber.isAllowedToClimb()){
      if (Helpers.OI.getClimbAxisValue(Constants.Climber.CLIMBER_USE_DEADBAND) > 0) {
        Helpers.Debug.debug("Climber: Raise Climber");
        m_climber.setClimberDirection("up");
      } else if (Helpers.OI.getClimbAxisValue(Constants.Climber.CLIMBER_USE_DEADBAND) < 0) {
        Helpers.Debug.debug("Climber: Lower Climber");
        m_climber.setClimberDirection("down");
      } else {
        Helpers.Debug.debug("Climber: Stop Climber");
        m_climber.setClimberDirection("stop");
      }
    }
  }
}