package org.firstinspires.ftc.teamcode.Orientation;

import static org.firstinspires.ftc.teamcode.RobotParameters.POSITION_TOLERANCE;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import java.util.Locale;

public class Position2D {
    public double x;
    public double y;
    public double angle;

    private double data[];



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

    @NonNull
    @Override
    public String toString()
    {
        return String.format(
                Locale.US, "(x=%.1f,y=%.1f,angle=%.1f)", x, y, angle);
    }   //toString

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
            Position2D pose = (Position2D) o;
            return Math.abs(pose.x - x) == 0.0 &&
                    Math.abs(pose.y - y) == 0.0 &&
                    Math.abs(pose.angle - angle) == 0.0;
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
            Position2D pose = (Position2D) o;
            return Math.abs(pose.x - x) <= tolerance &&
                    Math.abs(pose.y - y) <= tolerance &&
                    Math.abs(pose.angle - angle) <= tolerance;
        }
    }   //similar


}
