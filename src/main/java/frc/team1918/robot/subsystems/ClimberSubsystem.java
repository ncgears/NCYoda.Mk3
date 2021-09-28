
package frc.team1918.robot.subsystems;

import frc.team1918.robot.Constants;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  private Solenoid m_antigrav; //collector solenoid 1
  /**
   * Creates a new ExampleSubsystem.
   */
  public ClimberSubsystem() {
    m_antigrav = new Solenoid(Constants.Air.AIR_ANTIGRAV_ID);
  }

  public void SetAntigrav(boolean IsActive) {
    if(IsActive) { 
      m_antigrav.set(Constants.Air.AIR_ANTIGRAV_ENGAGED); 
    } else { 
      m_antigrav.set(!Constants.Air.AIR_ANTIGRAV_ENGAGED); 
    };
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
