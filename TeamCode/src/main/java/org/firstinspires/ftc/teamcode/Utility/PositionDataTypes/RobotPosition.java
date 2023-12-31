package org.firstinspires.ftc.teamcode.Utility.PositionDataTypes;

public class RobotPosition extends FieldPosition {
    public double rotation;

    public RobotPosition(double x, double y, double rotation) {
        super(x, y);
        this.rotation = rotation;
    }

    /**
     * This constructor clones the values of a vector over to this vector.
     *
     * @param otherVector The vector who's values you want to clone.
     */
    public RobotPosition(RobotPosition otherVector) {
        super(otherVector.x, otherVector.y);
        this.rotation = otherVector.rotation;
    }

    /**
     * This method combines the values of another RobotPosition with this RobotPosition instance.
     * Adds the X, Y, and Rotation values from the specified Vector 3 to this RobotPosition.
     *
     * @param otherVector The RobotPosition instance whose values will be added to this vector.
     */
    public void addValues(RobotPosition otherVector) {
        this.x += otherVector.x;
        this.y += otherVector.y;
        this.rotation += otherVector.rotation;
    }

    /**
     * This method subtracts the values of a RobotPosition from this vector.
     * Subtracts the X, Y, and Rotation values from the specified RobotPosition from this vector.
     *
     * @param otherVector The RobotPosition instance who's values will be subtracted from this vector.
     */
    public void subtractValues(RobotPosition otherVector) {
        this.x -= otherVector.x;
        this.y -= otherVector.y;
        this.rotation -= otherVector.rotation;
    }

    /**
     * This method divides this vector's values by those from the specified RobotPosition.
     * Divides this vector's X, Y, and Rotation values by those from the specified RobotPosition.
     *
     * @param otherVector The RobotPosition instance containing the values that this vector's values will
     *                    be divided by.
     */
    public void divideValues(RobotPosition otherVector) {
        this.x /= otherVector.x;
        this.y /= otherVector.y;
        this.rotation /= otherVector.rotation;
    }

    /**
     * This method multiplies this vector's values by those from the specified RobotPosition.
     * Multiplies this vector's X, Y, and Rotation values by those from the specified RobotPosition.
     *
     * @param otherVector The RobotPosition instance containing the values that this vector's values will
     *                    be multiplied by.
     */
    public void multiplyValues(RobotPosition otherVector) {
        this.x *= otherVector.x;
        this.y *= otherVector.y;
        this.rotation *= otherVector.rotation;
    }
}