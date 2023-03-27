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
    
    public Vector3 subtract(Vector3 other) {
        double x = this.x - other.x;
        double y = this.y - other.y;
        double z = this.z - other.z;
        return new Vector3(x, y, z);
    }

    public Vector3 add(Vector3 other){
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
        double norm = this.magnitude()*other.magnitude();
        double angleprior = this.dot(other)/norm;
        double angleradians = Math.acos(angleprior);
        double angle = Math.toDegrees(angleradians);
        return angle;
    }

    public double distance(Vector3 other)
    {
        double dx = this.x- other.x;
        double dy = this.y- other.y;
        double dz = this.z- other.z;
        return Math.sqrt(dx*dx + dy*dy + dz*dz);
    
    }
}
