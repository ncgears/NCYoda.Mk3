
package frc.team1918.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1918.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {
  private WPI_TalonSRX feed1; //first stage feed controller
  private WPI_TalonSRX feed2; //second stage feed controller
  private CANSparkMax shoot; //shooter controller

  /**
   * Creates a new ExampleSubsystem.
   */
  public ShooterSubsystem() {
    //Reset the talonsrx controllers to avoid any leftovers in flash and configure them as desired
    feed1 = new WPI_TalonSRX(Constants.Shooter.SHOOTER_FEED1_MC_ID);
    feed2 = new WPI_TalonSRX(Constants.Shooter.SHOOTER_FEED2_MC_ID);
    feed1.configFactoryDefault();
    feed2.configFactoryDefault();
    feed1.set(ControlMode.PercentOutput, 0);
    feed2.set(ControlMode.PercentOutput, 0);
    feed1.setNeutralMode(NeutralMode.Brake);
    feed2.setNeutralMode(NeutralMode.Brake);
    feed1.setInverted(Constants.Shooter.SHOOTER_FEED1_INVERT);
    feed2.setInverted(Constants.Shooter.SHOOTER_FEED2_INVERT);
    //Setup the SparkMAX controller as desired
    shoot = new CANSparkMax(Constants.Shooter.SHOOTER_SHOOT_MC_ID, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
