
package frc.team1918.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1918.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class CollectorSubsystem extends SubsystemBase {
  private WPI_TalonSRX coll; //collector controller
  private Solenoid m_coll1; //collector solenoid 1
  private Solenoid m_coll2; //collector solenoid 2
  private boolean m_collector_down = false;
  /**
   * Creates a new ExampleSubsystem.
   */
  public CollectorSubsystem() {
    coll = new WPI_TalonSRX(Constants.Collector.COLLECTOR_MC_ID);
    m_coll1 = new Solenoid(Constants.Air.AIR_COLLECTOR1_ID);
    m_coll2 = new Solenoid(Constants.Air.AIR_COLLECTOR2_ID);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setIntakeSpeed(double speed) {
    coll.set(ControlMode.PercentOutput, speed);
  }

  public boolean isCollectorDown() {
    return m_collector_down;
  }

  public void setCollectorPosition(String position) {
    switch(position) {
      case "down":
        //put collector down
        m_coll1.set(Constants.Air.AIR_COLL1_DOWN);
        m_coll2.set(Constants.Air.AIR_COLL2_DOWN);
        m_collector_down = true;
        break;
      case "mid-down":
      case "mid-up":
      case "up":
      case "stow":
      default:
        //put collector up
        m_coll1.set(!Constants.Air.AIR_COLL1_DOWN);
        m_coll2.set(!Constants.Air.AIR_COLL2_DOWN);
        m_collector_down = false;
        break;
    }
  }
}
