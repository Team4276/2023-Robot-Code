package frc.utilities;

//methods for vector math

public class Vector3 {

    public double x;
    public double y;
    public double z;

    public Vector3() {
        x = 0.0;
        x = 0.0;
        y = 0.0;
    }

    public Vector3(Vector3 other) {
        copy(other);
    }

    public Vector3 copy(Vector3 other) {
        x = other.x;
        y = other.y;
        z = other.z;     
        return this;
    }
    
    public void set(double new_x, double new_y, double new_z) {
        x = new_x;
        y = new_y;
        z = new_z;
    }

    public Vector3(double val_x, double val_y, double val_z) {
        x = val_x;
        y = val_y;
        z = val_z;
    }

    public double magnitude() {
        return Math.sqrt((x * x) + (y * y) + (z * z));
    }

    public Vector3 normalize() {
        double mag = magnitude();
        x /= mag;
        y /= mag;
        z /= mag;
        return this;
    }

    public Vector3 add(Vector3 other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return this;
    }

    public Vector3 subtract(Vector3 other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return this;
    }

    public double dot(Vector3 other) {
        return (x * other.x) + (y * other.y) + (z * other.z);
    }

    public double angle(Vector3 other) {
        double norm = magnitude() * other.magnitude();
        double angleprior = dot(other) / norm;
        double angleradians = Math.acos(angleprior);
        double angle = Math.toDegrees(angleradians);
        return angle;
    }

    public double distance(Vector3 other) {
        Vector3 temp = new Vector3(this);
        return temp.subtract(other).magnitude();
    }

    public boolean isEqual(Vector3 other) {
        return ( (x==other.x) && (y==other.y) && (z==other.z) );
    }
}