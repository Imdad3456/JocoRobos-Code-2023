package frc.robot;

import edu.wpi.first.wpilibj.I2C;

public class MPU6050 {
    static private final int xOutAddrH = 67;
    static private final int xOutAddrL = 68;
    static private final int yOutAddrH = 69;
    static private final int yOutAddrL = 70;
    static private final int zOutAddrH = 71;
    static private final int zOutAddrL = 72;

    /**
     * Using the onboard I2C port has been known to cause system lockups, Driver
     * Station will warn about this. Proceed with caution.
     */
    static private final I2C i2c = new I2C(I2C.Port.kOnboard, 1);

    /**
     * Reads and returns position values from the gyroscope.
     * 
     * @param valChoice - Which axis to return gyroscope values for. Options are
     *                  "X","Y", or "Z", non-case sensitive.
     * @return A byte array with the gyroscope values.
     */
    static public byte[] readGyro(String valChoice) {
        valChoice = valChoice.toLowerCase();
        byte[] valOut = new byte[2];
        byte[] temp0 = new byte[1], temp1 = new byte[1];

        switch (valChoice) {
            case "x":
                i2c.read(xOutAddrH, 1, temp0);
                valOut[0] = reorderBitUtil(temp0);
                i2c.read(xOutAddrL, 1, temp1);
                valOut[1] = reorderBitUtil(temp1);
                break;
            case "y":
                i2c.read(yOutAddrH, 1, temp0);
                valOut[0] = reorderBitUtil(temp0);
                i2c.read(yOutAddrL, 1, temp1);
                valOut[1] = reorderBitUtil(temp1);
                break;
            case "z":
                i2c.read(zOutAddrH, 1, temp0);
                valOut[0] = reorderBitUtil(temp0);
                i2c.read(zOutAddrL, 1, temp1);
                valOut[1] = reorderBitUtil(temp1);
                break;
        }
        return valOut;
    }

    /**
     * 
     * @param a - Byte array to rearrange the bits of.
     * @return The rearranged byte.
     */
    static public byte reorderBitUtil(byte[] a) {
        return (byte) (((a[0] & 0xFF) << 8) | ((a[0] & 0xFF00) >>> 8));
    }
}