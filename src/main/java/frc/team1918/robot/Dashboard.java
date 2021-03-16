//OI = Operator Interface
package frc.team1918.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * This class reads and writes values to/from the SmartDashboard
 */
public class Dashboard {
    //These define the names of the controls on the dashboard
    private static final String dash_shooter_speed = "ShootSpeed"; //Current shooter speed
    private static final String dash_shooter_target = "Shooter Target Speed"; //Target shooter speed
    private static final String dash_hood_pos = "HoodPosition"; //Hood position (up/down)
    private static final String dash_gyro_angle = "GyroAngle"; //Gyro angle

    public static final class Shooter {
        public static final void setCurrentSpeed(double speed) {
            SmartDashboard.putNumber(dash_shooter_speed,speed);
        }
        public static final void setTargetSpeed(double speed) {
            SmartDashboard.putNumber(dash_shooter_target,speed);
        }
        public static final double getTargetSpeed(double default_val) {
            return (double) SmartDashboard.getNumber(dash_shooter_target,default_val);
        }
        public static final void setHoodPosition(boolean up) {
            SmartDashboard.putString(dash_hood_pos,(up)?"up":"down");
        }
    }
    public static final class Gyro {
        public static final void setGyroAngle(double angle) {
            SmartDashboard.putNumber(dash_gyro_angle,angle);
        }
    }
}