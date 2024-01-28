package org.firstinspires.ftc.teamcode.Orientation;

import androidx.annotation.NonNull;

import java.util.Locale;

public class Position2D {
    public double x;
    public double y;
    public double angle;


    /**
     * Constructor: Create an instance of the object.
     *
     * @param x specifies the x component of the pose (in cm).
     * @param y specifies the y component of the pose (in cm).
     * @param angle specifies the angle in degrees.
     */
    public Position2D(double x, double y, double angle)
    {
        this.x = x;
        this.y = y;
        this.angle = angle;
    }   //Position2D

    /**
     * @return A Position2D string
     */
    @NonNull
    @Override
    public String toString()
    {
        return String.format(
                Locale.US, "(x=%.1f,y=%.1f,angle=%.1f)", x, y, angle);
    }   //toString

    /**
     * Compares this pose with the specified pose for equality.
     * @param obj the Position2D object that should be compared.
     * @return true if equal, false otherwise.
     */
    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        } else if (obj == null || getClass() != obj.getClass()) {
            return false;
        } else {
            Position2D pose = (Position2D) obj;
            return Math.abs(pose.x - x) == 0.0 &&
                    Math.abs(pose.y - y) == 0.0 &&
                    Math.abs(pose.angle - angle) == 0.0;
        }
    }   //equals

    /**
     * Compares the pose with the specified pose.
     * @param obj The Position2D object that should be compared.
     * @param tolerance The tolerance for the similarity of the poses (in cm).
     * @return True if similar, false otherwise.
     */
    public boolean similar(Object obj, double tolerance) {
        if (this == obj) {
            return true;
        } else if (obj == null || getClass() != obj.getClass()) {
            return false;
        } else {
            Position2D pose = (Position2D) obj;
            return Math.abs(pose.x - x) <= tolerance &&
                    Math.abs(pose.y - y) <= tolerance &&
                    Math.abs(pose.angle - angle) <= tolerance;
        }
    }   //similar


}
