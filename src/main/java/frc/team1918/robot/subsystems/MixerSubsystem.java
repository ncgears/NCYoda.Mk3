
package frc.team1918.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1918.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class MixerSubsystem extends SubsystemBase {
  private WPI_TalonSRX mixer; //mixer controller
  /**
   * Creates a new ExampleSubsystem.
   */
  public MixerSubsystem() {
    mixer = new WPI_TalonSRX(Constants.Mixer.MIXER_MC_ID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setMixerSpeed(double speed) {
    mixer.set(ControlMode.PercentOutput, speed);
  }
}
