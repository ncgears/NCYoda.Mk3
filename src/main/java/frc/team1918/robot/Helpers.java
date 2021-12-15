package frc.team1918.robot;

import edu.wpi.first.wpilibj.Joystick;

public class Helpers {
    //Helpers for Debugging
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
                if(debugThrottleMet(ticks)) System.out.println(message);
            }
            return ticks++;
        }
        public final static boolean debugThrottleMet(int ticks) {
            return (ticks % Constants.Global.DEBUG_RECURRING_TICKS == 0);
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
    //General Helpers
    public static final class General {
        /**
         * This function takes values a and b. It determines the minimum Change between the two based on the wrap value.
         * @param a 
         * @param b
         * @param wrap
         * @return
         */
        public final static double minChange(double a, double b, double wrap) {
            return Math.IEEEremainder(a - b, wrap);
        }
        public final static int minChange(int a, int b, int wrap) {
            return (int) Math.IEEEremainder(a - b, wrap);
        }

        public final static double roundDouble(double val, int decimals) {
            return Math.round(val * Math.pow(10,decimals)) / Math.pow(10,decimals);
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
         * This function takes radians and returns encoder ticks (based on a 0 offset)
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

        public final static double metersPerSecondToRPM(double mps, double wheelDiamMM) {
            return ((mps * 1000) / (wheelDiamMM * Math.PI)) * 60;
        }

    }
    //Helpers for the Operator Interface
    public static final class OI {
        private static Joystick dj = new Joystick(Constants.OI.OI_JOY_DRIVER);
        private static Joystick oj = new Joystick(Constants.OI.OI_JOY_OPER);
        //DRIVER CONTROLS
        /**
         * @param useDeadband Boolean value indicating whether to apply deadband to output
         * @return double precision -1 to 1 of the strafe axis
         */
        public final static double getAxisStrafeValue(boolean useDeadband) {
            return (useDeadband) ? applyDeadband(dj.getRawAxis(Constants.OI.Driver.AXIS_STRAFE)) : dj.getRawAxis(Constants.OI.Driver.AXIS_STRAFE);
        }

        /**
         * @param useDeadband Boolean value indicating whether to apply deadband to output
         * @return double precision -1 to 1 of the fwd axis
         */
        public final static double getAxisFwdValue(boolean useDeadband) {
            return (useDeadband) ? applyDeadband(dj.getRawAxis(Constants.OI.Driver.AXIS_FWD)*-1) : dj.getRawAxis(Constants.OI.Driver.AXIS_FWD)*-1;
        }

        /**
         * @param useDeadband Boolean value indicating whether to apply deadband to output
         * @return double precision -1 to 1 of the turn axis
         */
        public final static double getAxisTurnValue(boolean useDeadband) {
            return (useDeadband) ? applyDeadband(dj.getRawAxis(Constants.OI.Driver.AXIS_TURN)) : dj.getRawAxis(Constants.OI.Driver.AXIS_TURN);
        }

        //OPERATOR CONTROLS
        /**
         * @param useDeadband Boolean value indicating whether to apply deadband to output
         * @return double precision -1 to 1 of the fwd axis
         */
        public double getClimbAxisValue(boolean useDeadband) {
            return (useDeadband) ? applyDeadband(oj.getRawAxis(Constants.OI.Operator.AXIS_CLIMB)) : oj.getRawAxis(Constants.OI.Operator.AXIS_CLIMB);
        }

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
            return ( Math.abs(inVal) < Constants.OI.OI_JOY_DEADBAND ) ? 0.0 : ramped;
            //TODO: Figure out math for transformation
            /**
             * The following math is from 2767 Stryke Force ExpoScale Utility
             * https://github.com/strykeforce/thirdcoast/blob/master/src/main/java/org/strykeforce/thirdcoast/util/ExpoScale.java
             */
            // var scale = 1;
            // var deadband = 0.15;
            // var offset = 1.0 / (scale * Math.pow(1 - deadband, 3) + (1 - scale) * (1 - deadband));
            // return (scale * Math.pow(inVal, 3) + (1 - scale) * inVal) * offset;
        }
    }
}