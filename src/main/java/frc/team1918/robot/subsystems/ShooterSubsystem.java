
package frc.team1918.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1918.robot.Constants;
import frc.team1918.robot.Dashboard;
import frc.team1918.robot.Helpers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {
  private WPI_TalonSRX feed1; // first stage feed controller
  private WPI_TalonSRX feed2; // second stage feed controller
  private CANSparkMax shoot; // shooter controller
  private double m_shooter_rpm = 0.0; // Current shooter speed
  private double m_shooter_oldrpm = 0.0; // Old shooter speed
  private CANPIDController m_pidController;
  private CANEncoder m_encoder;
  private Solenoid m_hood;
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
    feed1.setNeutralMode(NeutralMode.Coast);
    feed2.setNeutralMode(NeutralMode.Coast);
    feed1.setInverted(Constants.Shooter.SHOOTER_FEED1_INVERT);
    feed2.setInverted(Constants.Shooter.SHOOTER_FEED2_INVERT);
    //Setup the SparkMAX controller as desired
    shoot = new CANSparkMax(Constants.Shooter.SHOOTER_SHOOT_MC_ID, MotorType.kBrushless);
    shoot.restoreFactoryDefaults();
    shoot.setInverted(Constants.Shooter.SHOOTER_SHOOT_INVERT);
    shoot.setIdleMode(IdleMode.kCoast);
    m_pidController = shoot.getPIDController();
    m_encoder = shoot.getEncoder();
    m_pidController.setP(Constants.Shooter.SHOOTER_PID_P); //PID P
    m_pidController.setI(Constants.Shooter.SHOOTER_PID_I); //PID I
    m_pidController.setD(Constants.Shooter.SHOOTER_PID_D); //PID D
    m_pidController.setIZone(Constants.Shooter.SHOOTER_PID_IZONE); //IZone
    m_pidController.setFF(Constants.Shooter.SHOOTER_PID_FF); //Feed forward
    m_pidController.setOutputRange(-0.1, 1); //-10% allowed for speed down of shooter
    //Setup the solenoid
    m_hood = new Solenoid(Constants.Air.AIR_HOOD_ID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_shooter_rpm != m_shooter_oldrpm) {
      m_pidController.setReference(m_shooter_rpm, ControlType.kVelocity); //Set the target
      m_shooter_oldrpm=m_shooter_rpm;
    }
    Dashboard.Shooter.setCurrentSpeed(getShooterSpeed());
  }

  public double getShooterSpeed() {
    return Helpers.General.roundDouble(m_encoder.getVelocity(),0);
  }

  public void setShooterSpeed(double RPM) {
    m_shooter_rpm = Math.min(RPM, Constants.Shooter.SHOOTER_MAX_RPM);
  }

  public void setShooterVbus(double Vbus) {
    shoot.set(Vbus);
  }

  public void increaseShooterSpeed() {
    m_shooter_rpm = Math.min(m_shooter_rpm + Constants.Shooter.SHOOTER_SPEED_INCREMENT, Constants.Shooter.SHOOTER_MAX_RPM);
  }

  public void decreaseShooterSpeed() {
    m_shooter_rpm = Math.max(m_shooter_rpm - Constants.Shooter.SHOOTER_SPEED_INCREMENT, Constants.Shooter.SHOOTER_MIN_RPM);
  }

  public void setShooterSpeedFromDashboard() {
    setShooterSpeed(Dashboard.Shooter.getTargetSpeed(0));
  }

  public void raiseHood(boolean up) {
    //send command to air to put hood up or down based on boolean
    m_hood.set(up);
    Dashboard.Shooter.setHoodPosition(up);
  }

  public void runFeeder(boolean run) {
    //run the feeder based on boolean
    if (run && !Constants.Shooter.SHOOTER_FEED_DISABLED) {
      feed1.set(ControlMode.PercentOutput, Constants.Shooter.SHOOTER_FEED1_SPEED);
      feed2.set(ControlMode.PercentOutput, Constants.Shooter.SHOOTER_FEED2_SPEED);
    } else {
      feed1.set(ControlMode.PercentOutput, 0);
      feed2.set(ControlMode.PercentOutput, 0);
    }
  }

}
