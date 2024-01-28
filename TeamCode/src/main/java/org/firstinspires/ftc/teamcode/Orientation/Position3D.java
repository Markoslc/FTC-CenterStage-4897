package org.firstinspires.ftc.teamcode.Orientation;

import androidx.annotation.NonNull;

import static org.firstinspires.ftc.teamcode.RobotParameters.*;

import java.util.Locale;

public class Position3D {

    public double x;
    public double y;
    public double z;
    public double yaw;
    public double pitch;
    public double roll;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param x specifies the x component of the pose.
     * @param y specifies the y component of the pose.
     * @param z specifies the z component of the pose.
     * @param yaw specifies the yaw angle.
     * @param pitch specifies the pitch angle.
     * @param roll specifies the roll angle.
     */
    public Position3D(double x, double y, double z, double yaw, double pitch, double roll)
    {
        this.x = x;
        this.y = y;
        this.z = z;
        this.yaw = yaw;
        this.pitch = pitch;
        this.roll = roll;
    }   //RobotPos

    @NonNull
    @Override
    public String toString()
    {
        return String.format(
                Locale.US, "(x=%.1f,y=%.1f,z=%.1f,yaw=%.1f,pitch=%.1f,roll=%.1f)", x, y, z, yaw, pitch, roll);
    }   //toString
    /**
     * This method sets this pose to be the same as the given pose.
     *
     * @param pose specifies the pose to make this pose equal to.
     */
    public void setAs(Position3D pose)
    {
        this.x = pose.x;
        this.y = pose.y;
        this.z = pose.z;
        this.yaw = pose.yaw;
        this.pitch = pose.pitch;
        this.roll = pose.roll;
    }   //setAs

    /**
     * This method compares this pose with the specified pose for equality.
     *
     * @return true if equal, false otherwise.
     */
    @Override
    public boolean equals(Object o) {
        if (this == o) {
            return true;
        } else if (o == null || getClass() != o.getClass()) {
            return false;
        } else {
            Position3D pose = (Position3D) o;
            return Math.abs(pose.x - x) == 0.0 &&
                    Math.abs(pose.y - y) == 0.0 &&
                    Math.abs(pose.z - z) == 0.0 &&
                    Math.abs(pose.yaw - yaw) == 0.0 &&
                    Math.abs(pose.pitch - pitch) == 0.0 &&
                    Math.abs(pose.roll - roll) == 0.0;
        }
    }   //equals

    /**
     * This method compares this pose with the specified pose if they are similar to each other true if the difference is withing the tolerance..
     *
     * @return true if similar, false otherwise.
     */
    public boolean similar(Object o, double tolerance) {
        if (this == o) {
            return true;
        } else if (o == null || getClass() != o.getClass()) {
            return false;
        } else {
            Position3D pose = (Position3D) o;
            return Math.abs(pose.x - x) <= tolerance &&
                    Math.abs(pose.y - y) <= tolerance &&
                    Math.abs(pose.z - z) <= tolerance &&
                    Math.abs(pose.yaw - yaw) <= tolerance &&
                    Math.abs(pose.pitch - pitch) <= tolerance &&
                    Math.abs(pose.roll - roll) <= tolerance;
        }
    }   //similar

}
