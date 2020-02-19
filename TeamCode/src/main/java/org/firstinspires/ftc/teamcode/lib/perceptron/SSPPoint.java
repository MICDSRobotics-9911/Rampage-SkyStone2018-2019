package org.firstinspires.ftc.teamcode.lib.perceptron;

/**
 * Basic data point for the skystone perceptron (SSP)
 * @since 2/1/20
 */
public class SSPPoint {
    /**
     * The data point from the color sensor
     */
    private float colorSensorPoint;

    /**
     * The data point from the imu (acceleration)
     */
    private float IMUPoint;

    public SSPPoint (float cs, float imu) {
        this.colorSensorPoint = cs;
        this.IMUPoint = imu;
    }

    public float getIMUPoint() {
        return IMUPoint;
    }

    public float getColorSensorPoint() {
        return colorSensorPoint;
    }

    public void setColorSensorPoint(int colorSensorPoint) {
        this.colorSensorPoint = colorSensorPoint;
    }

    public String toString() {
        return this.colorSensorPoint + "," + this.IMUPoint;
    }

    public void setIMUPoint(float IMUPoint) {
        this.IMUPoint = IMUPoint;
    }
}
