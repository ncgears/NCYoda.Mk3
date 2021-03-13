//OI = Operator Interface
package frc.team1918.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * This class reads and writes values to/from the SmartDashboard
 */
public class Dashboard {
    public static final class Shooter {
        public static final void setCurrentSpeed(double speed) {
            SmartDashboard.putNumber("ShootSpeed",speed);
        }
        
        public static final void setTargetSpeed(double speed) {
            SmartDashboard.putNumber("Shooter Target Speed",speed);
        }

        public static final double getTargetSpeed(double default_val) {
            return (double) SmartDashboard.getNumber("Shooter Target Speed",default_val);
        }

        public static final void setHoodPosition(boolean up) {
            SmartDashboard.putString("HoodPosition",(up)?"up":"down");
        }
    }
    public static final class Gyro {
        public static final void setGyroAngle(double angle) {
            SmartDashboard.putNumber("GyroAngle",angle);
        }
    }
}