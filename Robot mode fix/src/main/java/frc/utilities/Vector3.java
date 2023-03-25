package frc.utilities;

//methods for vector math

public class Vector3 {

    public double x;
    public double y;
    public double z;

    public Vector3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public void copy(Vector3 other) {
        x = other.x;
        y = other.y;
        z = other.z;
    }

    public void set(double new_x, double new_y, double new_z) {
        x = new_x;
        y = new_y;
        z = new_z;
    }

    public double[] getDoubleArray() {
        double[] retVal = {x, y, z};
        return retVal;
    }

    public Vector3 subtract(Vector3 other) {
        double x = this.x - other.x;
        double y = this.y - other.y;
        double z = this.z - other.z;
        return new Vector3(x, y, z);
    }

    public Vector3 add(Vector3 other) {
        double x = this.x + other.x;
        double y = this.y + other.y;
        double z = this.z + other.z;
        return new Vector3(x, y, z);
    }

    public double magnitude() {
        return Math.sqrt(x * x + y * y + z * z);
    }

    public Vector3 normalize() {
        double mag = magnitude();
        double x = this.x / mag;
        double y = this.y / mag;
        double z = this.z / mag;
        return new Vector3(x, y, z);
    }

    public double dot(Vector3 other) {
        return this.x * other.x + this.y * other.y + this.z * other.z;
    }

    public double angle(Vector3 other) {
        Vector3 a = this.normalize();
        Vector3 b = other.normalize();
        double dot = a.dot(b);
        return Math.acos(dot);
    }

    public boolean isEqual(Vector3 other) {
        return ( (x==other.x) && (y==other.y) && (z==other.z) );
    }

}
