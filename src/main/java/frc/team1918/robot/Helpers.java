//OI = Operator Interface
package frc.team1918.robot;

import java.text.DecimalFormat;

import edu.wpi.first.wpilibj.Joystick;

public class Helpers {
    public static final class Debug {
        private static boolean debugEnabled = Constants.Global.DEBUG_ENABLED_DEFAULT;
        /**
         * This function takes a string and outputs it to the console when the debugging is enabled
         * @param message String to print to console
         */
        public final static void debug(String message) {
            if (debugEnabled) {
                System.out.println(message);
            }
        }
        public final static int debug(String message, int ticks) {
            if (debugEnabled) {
                if(ticks % Constants.Global.DEBUG_RECURRING_TICKS == 0) System.out.println(message);
            }
            return ticks++;
        }

        /**
         * This function toggles the debugging output to console. In a future version, each press will increase the debug level
         * through a set list of severity levels.
         */
        public final static void toggleDebug() {
            debugEnabled = !debugEnabled;
            System.out.println("Debugging Output=" + debugEnabled);
        }
    }
    public static final class General {
        /**
        * This function takes a value in inches and returns in meters
        * @param inches double precision value in inches
        * @return value in meters
        */
        public final static double inToMeters(double inches) {
           return inches * 0.0254;
        }

        public final static double roundDouble(double val, int decimals) {
            return Math.round(val * (10 ^ decimals)) / (10 ^ decimals);
            // final DecimalFormat df = new DecimalFormat(format);
            // return df.format(val);
        }

        /**
         * This function takes encoder ticks and returns radians
         * @param ticks integer value in encoder ticks
         * @return value in radians
         */
        public final static double ticksToRadians(int ticks) {
            return (ticks * Math.PI / (Constants.DriveTrain.DT_TURN_ENCODER_FULL_ROTATION / 2));
        }

        /**
         * This function takes radians and returns encoder ticks
         * @param rads double precision value in radians
         * @return integer value in encoder ticks
         */
        public final static int radiansToTicks(double rads) {
			return (int) (rads / Math.PI * (Constants.DriveTrain.DT_TURN_ENCODER_FULL_ROTATION / 2));
        }

        /**
         * This function takes a value and modifies it by a gear reduction (or multiplication) for a single reduction
         * @param value double precision value of input
         * @param gearOne integer value of the number of teeth on the input side of the gear set (motor side)
         * @param gearTwo integer value of the number of teeth on the output side of the gear set (wheel side)
         * @return double precision value after gear reduction (or multiplication)
         */
        public final static double gearCalcSingle(double value, int gearOne, int gearTwo) {
            return value * (gearOne / gearTwo);
        }
        /**
         * This function takes a value and modifies it by a gear reduction (or multiplication) for a single reduction
         * @param value double precision value of input
         * @param firstGearOne integer value of the number of teeth on the input side of the first gear set (motor side)
         * @param firstGearTwo integer value of the number of teeth on the output side of the first gear set (wheel side)
         * @param secondGearOne integer value of the number of teeth on the output side of the second gear set (wheel side)
         * @param secondGearTwo integer value of the number of teeth on the output side of the second gear set (wheel side)
         * @return double precision value after gear reduction (or multiplication)
         */
        public final static double gearCalcDouble(double value, int firstGearOne, int firstGearTwo, int secondGearOne, int secondGearTwo) {
            return value * (firstGearOne / firstGearTwo) * (secondGearOne / secondGearTwo);
        }

        /**
         * This function takes a value in RPMs and converts it to meters Per Second.
         * @param rpm double precision value of RPMs from the motor controller
         * @return double precision value in meters per second based on wheel size and gear sets
         */
        public final static double rpmToMetersPerSecond(double rpm, double wheelDiamMM) {
            return ((rpm / 60) * (wheelDiamMM * Math.PI)) / 1000;
        }

    }
    public static final class OI {
        private static Joystick dj = new Joystick(Constants.OI.OI_JOY_DRIVE);
        private static Joystick oj = new Joystick(Constants.OI.OI_JOY_OPER);
        //DRIVER CONTROLS
        /**
         * @param useDeadband Boolean value indicating whether to apply deadband to output
         * @return double precision -1 to 1 of the strafe axis
         */
        public final static double getAxisStrafeValue(boolean useDeadband) {
            return (useDeadband) ? applyDeadband(dj.getRawAxis(Constants.OI.DRIVE_AXIS_STRAFE)) : dj.getRawAxis(Constants.OI.DRIVE_AXIS_STRAFE);
        }

        /**
         * @param useDeadband Boolean value indicating whether to apply deadband to output
         * @return double precision -1 to 1 of the fwd axis
         */
        public final static double getAxisFwdValue(boolean useDeadband) {
            return (useDeadband) ? applyDeadband(dj.getRawAxis(Constants.OI.DRIVE_AXIS_FWD)*-1) : dj.getRawAxis(Constants.OI.DRIVE_AXIS_FWD)*-1;
        }

        /**
         * @param useDeadband Boolean value indicating whether to apply deadband to output
         * @return double precision -1 to 1 of the turn axis
         */
        public final static double getAxisTurnValue(boolean useDeadband) {
            return (useDeadband) ? applyDeadband(dj.getRawAxis(Constants.OI.DRIVE_AXIS_TURN)) : dj.getRawAxis(Constants.OI.DRIVE_AXIS_TURN);
        }
        // /**
        //  * @return integer value of Drive DPAD
        //  */
        // public int getPOVDrive() {
        //     return dj.getPOV(0);
        // }
        // /**
        //  * This function determines if we are pressing any of the top half of the driver DPAD.
        //  * @return boolean indicating top half of Driver DPAD is pressed
        //  */
        // public boolean isDriveDpadUp() {
        //     switch (getPOVDrive()) {
        //         case Constants.OI.DRIVE_DPAD_UPLEFT:
        //         case Constants.OI.DRIVE_DPAD_UP:
        //         case Constants.OI.DRIVE_DPAD_UPRIGHT: return true;
        //     }
        //     return false;
        // }
        // /**
        //  * This function determines if we are pressing any of the bottom half of the driver DPAD.
        //  * @return boolean indicating bottom half of Drive DPAD is pressed
        //  */
        // public boolean isDriveDpadDown() {
        //     switch (getPOVDrive()) {
        //         case Constants.OI.DRIVE_DPAD_DNLEFT:
        //         case Constants.OI.DRIVE_DPAD_DN:
        //         case Constants.OI.DRIVE_DPAD_DNRIGHT: return true;
        //     }
        //     return false;
        // }
        // /**
        //  * This function returns the direction of the Drive DPAD
        //  * @return enum OI.driveDpadDirection containing one of UP, DOWN, IDLE
        //  */
        // public Constants.OI.driveDpadDirection getDriveDpadDirection() {
        //     switch (getPOVDrive()) {
        //         //Top half of Drive DPAD
        //         case Constants.OI.DRIVE_DPAD_UPLEFT:
        //         case Constants.OI.DRIVE_DPAD_UP:
        //         case Constants.OI.DRIVE_DPAD_UPRIGHT: return Constants.OI.driveDpadDirection.UP;
        //         //Bottom half of Drive DPAD
        //         case Constants.OI.DRIVE_DPAD_DNLEFT:
        //         case Constants.OI.DRIVE_DPAD_DN:
        //         case Constants.OI.DRIVE_DPAD_DNRIGHT: return Constants.OI.driveDpadDirection.DOWN;
        //     }
        //     return Constants.OI.driveDpadDirection.IDLE;
        // }

        //OPERATOR CONTROLS
        /**
         * @param useDeadband Boolean value indicating whether to apply deadband to output
         * @return double precision -1 to 1 of the fwd axis
         */
        public double getClimbAxisValue(boolean useDeadband) {
            return (useDeadband) ? applyDeadband(oj.getRawAxis(Constants.OI.OPER_AXIS_CLIMB)) : oj.getRawAxis(Constants.OI.OPER_AXIS_CLIMB);
        }
        // /**
        //  * @return integer value of Operator DPAD
        //  */
        // public int getOperPOV() {
        //     return oj.getPOV(0);
        // }

        //HELPERS
        /**
         * @param inVal double precision input value to apply deadband
         * @return double precision -1 to 1 after applying deadband calculation
         */
        public static final double applyDeadband(double inVal) {
            return ( Math.abs(inVal) < Constants.OI.OI_JOY_DEADBAND ) ? 0.0 : inVal;
        }

        /**
         * This function zeros the joystick below the deadband. After the deadband, the value is normalized
         * from zero to 1
         * @param inVal double precision input value to apply deadband
         * @return double precision -1 to 1 after applying deadband calculation
         */
        public static final double applyRampingDeadband(double inVal) {
            double ramped = inVal;
            //TODO: Figure out math for transformation
            return ( Math.abs(inVal) < Constants.OI.OI_JOY_DEADBAND ) ? 0.0 : ramped;
        }
        // public static enum DPAD {
        //     UP(0), UP_RIGHT(45), RIGHT(90), DOWN_RIGHT(135), DOWN(180), DOWN_LEFT(225), LEFT(270), UP_LEFT(315);
        //     private int value;
        //     /**
        //      * Constructor
        //      * @param value
        //      */
        //     private DPAD(int value) {
        //         this.value = value;
        //     }

        //     /**
        //      * Convert integers to DPAD values
        //      * @param angle
        //      * @return DPAD with matching angle
        //      */
        //     public static DPAD getEnum(int angle) {
        //         angle = Math.abs(angle);
        //         angle %= 360;
        //         angle = Math.round(angle / 45) * 45; //May have rounding errors, due to rounding errors...
        //         for (DPAD dpad : values()) {
        //             if (dpad.value == angle) return dpad;
        //         }
        //         return null;
        //     }
        // }
        // public static class DirectionalPad extends Button {
        //     private final Joystick parent;
        //     public final Button UP;
        //     public final Button UPRIGHT;
        //     public final Button RIGHT;
        //     public final Button DOWNRIGHT;
        //     public final Button DOWN;
        //     public final Button DOWNLEFT;
        //     public final Button LEFT;
        //     public final Button UPLEFT;

        //     /**
        //      * Constructor
        //      * @param parent - This is the joystick object that owns this dpad
        //      */
        //     DirectionalPad(final Joystick parent) {
        //         this.parent = parent;
        //         this.UP = new DPadButton(this, DPAD.UP);
        //         this.UPRIGHT = new DPadButton(this, DPAD.UP_RIGHT);
        //         this.RIGHT = new DPadButton(this, DPAD.RIGHT);
        //         this.DOWNRIGHT = new DPadButton(this, DPAD.DOWN_RIGHT);
        //         this.DOWN = new DPadButton(this, DPAD.DOWN);
        //         this.DOWNLEFT = new DPadButton(this, DPAD.DOWN_LEFT);
        //         this.LEFT = new DPadButton(this, DPAD.LEFT);
        //         this.UPLEFT = new DPadButton(this, DPAD.UP_LEFT);
        //     }

        //     /**
        //      * This sub-class is used to represent each of the 8 values as a button
        //      */
        //     public static class DPadButton extends Button {
        //         private final DPAD direction;
        //         private final DirectionalPad parent;

        //         /**
        //          * Constructor
        //          * @param parent - This is the DirectionalPad that owns this button
        //          * @param DPAD
        //          */
        //         DPadButton(final DirectionalPad parent, final DPAD dPadDirection) {
        //             this.direction = dPadDirection;
        //             this.parent = parent;
        //         }

        //         @Override
        //         public boolean get() {
        //             return parent.getAngle() == direction.value;
        //         }
        //     }

        //     private int angle() {
        //         return parent.getPOV();
        //     }

        //     @Override
        //     public boolean get() {
        //         return angle() != -1;
        //     }

        //     /**
        //      * UP           0;
        //      * UP_RIGHT     45;
        //      * RIGHT        90;
        //      * DOWN_RIGHT   135;
        //      * DOWN         180;
        //      * DOWN_LEFT    225;
        //      * LEFT         270;
        //      * UP_LEFT      315;
        //      * @return A number between 0 and 315 indicating dpad direction
        //      */
        //     public int getAngle() {
        //         return angle();
        //     }

        //     /**
        //      * Just like getAngle, but returns a direction instead of an angle
        //      * @return A DPAD direction
        //      */
        //     public DPAD getDirection() {
        //         return DPAD.getEnum(angle());
        //     }
        // }
    }
}