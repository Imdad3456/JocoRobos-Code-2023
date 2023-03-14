package frc.robot;

import edu.wpi.first.wpilibj.I2C;

public class MPU6050 {
    static private final int xOutAddrH = 67;
    static private final int yOutAddrH = 69;
    static private final int zOutAddrH = 71;

    /*
     * Using the onboard I2C port has been known to cause system lockups, Driver
     * Station will warn about this. Proceed with caution.
     */
    static private final I2C i2c = new I2C(I2C.Port.kOnboard, 1);

    /*
     * I'm not sure exactly what these "H" and "L" things are but I know they have
     * something to do with ratios (15:8 for H and 7:0 for L).
     * static private final int xOutAddrL = 68;
     * static private final int yOutAddrL = 70;
     * static private final int zOutAddrL = 72;
     */

    /**
     * Reads and returns position values from the gyroscope.
     * 
     * @param valChoice - Which axis to return gyroscope values for. Options are
     *                  "X","Y", or "Z", non-case sensitive.
     * @return A byte array with the gyroscope values.
     */
    static public byte[] readGyro(String valChoice) {
        valChoice = valChoice.toLowerCase();
        byte[] valOut = new byte[1];
        switch (valChoice) {
            case "x":
                i2c.read(xOutAddrH, 1, valOut);
            case "y":
                i2c.read(yOutAddrH, 1, valOut);
            case "z":
                i2c.read(zOutAddrH, 1, valOut);
                break;
        }
        return valOut;
    }
}
