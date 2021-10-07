
package frc.team1918.robot.subsystems;

import frc.team1918.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  private Solenoid m_antigrav; //climber antigrav solenoid
  private WPI_TalonSRX m_climber1; //climber talon 1
  private WPI_TalonSRX m_climber2; //climber talon 2
  private double m_speed1;
  private double m_speed2;
  private boolean m_antigrav_engaged = false;
  private DigitalInput m_limit_top = new DigitalInput(Constants.Climber.CLIMBER_LIMIT_TOP);;
  private DigitalInput m_limit_bottom = new DigitalInput(Constants.Climber.CLIMBER_LIMIT_BOTTOM);;
  /**
   * Creates a new ExampleSubsystem.
   */
  public ClimberSubsystem() {
    m_antigrav = new Solenoid(Constants.Air.AIR_ANTIGRAV_ID);
  }

  public void setAntigrav(boolean IsActive) {
    if(IsActive) { 
      m_antigrav.set(Constants.Air.AIR_ANTIGRAV_ENGAGED); 
    } else { 
      m_antigrav.set(!Constants.Air.AIR_ANTIGRAV_ENGAGED); 
    }
    m_antigrav_engaged = IsActive;
  }

  public boolean isAntigravEngaged() {
    return m_antigrav_engaged;
  }

  public boolean isAllowedToClimb() {
    return true; //should check FMS for time to see if in last 30 seconds
  }

  public void setClimberDirection(String direction) {
    switch(direction) {
      case "up":
        //climber up (robot down)
        m_speed1 = Constants.Climber.CLIMBER_SPEED;
        m_speed2 = Constants.Climber.CLIMBER_SPEED;
        m_speed1 *= (Constants.Climber.CLIMBER_1_isInverted) ? -1.0 : 1.0;
        m_speed2 *= (Constants.Climber.CLIMBER_2_isInverted) ? -1.0 : 1.0;
        
        m_climber1.set(m_speed1 * 1.0);
        m_climber2.set(m_speed2 * 1.0);

        break;
      case "down":
        //climber down (robot up)
        m_speed1 = Constants.Climber.CLIMBER_SPEED;
        m_speed2 = Constants.Climber.CLIMBER_SPEED;
        m_speed1 *= (Constants.Climber.CLIMBER_1_isInverted) ? -1.0 : 1.0;
        m_speed2 *= (Constants.Climber.CLIMBER_2_isInverted) ? -1.0 : 1.0;
        
        m_climber1.set(m_speed1 * -1.0);
        m_climber2.set(m_speed2 * -1.0);
        break;
      case "stop":
      default:
        //stop climber
        m_climber1.set(0);
        m_climber2.set(0);
        break;
    }
  }

  public boolean IsLimitTop() {
    return m_limit_top.get();
  }

  public boolean IsLimitBottom() {
    return m_limit_bottom.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
