
package frc.team1918.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1918.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class CollectorSubsystem extends SubsystemBase {
  private WPI_TalonSRX coll; //collector controller
  /**
   * Creates a new ExampleSubsystem.
   */
  public CollectorSubsystem() {
    coll = new WPI_TalonSRX(Constants.Collector.COLLECTOR_MC_ID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
