package frc.robot.Framework;

public class RectangularZone {
    private double x, y;
    private double width, height;

    public RectangularZone(double x, double y, double width, double height) {
        this.x = x;
        this.y = y;
        this.width = width;
        this.height = height;
    }

    public boolean inZone(double x, double y) {
        return (
            x > this.x &&
            x < this.x+width &&
            y > this.y &&
            y < this.y+height
        );
    }
}
