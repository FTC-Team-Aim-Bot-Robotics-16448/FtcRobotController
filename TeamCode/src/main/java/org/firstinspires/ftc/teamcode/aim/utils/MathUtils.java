package org.firstinspires.ftc.teamcode.aim.utils;

public class MathUtils {
    /**
     * Calculate robot target position to reach a ball
     * @param currentX Current robot X position on field (mm)
     * @param currentY Current robot Y position on field (mm)
     * @param currentHeading Current robot heading (degrees, 0 = facing forward)
     * @param relativeX Ball X relative to robot (mm, + = right)
     * @param relativeY Ball Y relative to robot (mm, + = front)
     * @param collectOffset Distance to stop before ball (mm, for intake reach)
     * @return target robot position [x, y, heading]
     */
    public static double[] calculateTargetPosition(
            double currentX, double currentY, double currentHeading,
            double relativeX, double relativeY, double collectOffset) {

        // Convert robot heading to radians
        double headingRad = Math.toRadians(currentHeading);

        // Convert relative ball position to field coordinates
        double ballFieldX = currentX + relativeX * Math.cos(headingRad) - relativeY *
                Math.sin(headingRad);
        double ballFieldY = currentY + relativeX * Math.sin(headingRad) + relativeY *
                Math.cos(headingRad);

        // Calculate distance from robot to ball
        double distanceToBall = Math.sqrt(relativeX * relativeX + relativeY * relativeY);

        // Calculate target position (stop collectOffset mm before ball)
        double targetDistance = distanceToBall - collectOffset;

        // Calculate target robot position
        double targetX = currentX + (relativeX / distanceToBall) * targetDistance *
                Math.cos(headingRad)
                - (relativeY / distanceToBall) * targetDistance *
                Math.sin(headingRad);
        double targetY = currentY + (relativeX / distanceToBall) * targetDistance *
                Math.sin(headingRad)
                + (relativeY / distanceToBall) * targetDistance *
                Math.cos(headingRad);

        // Calculate target heading (face toward ball)
        double targetHeading = Math.toDegrees(Math.atan2(relativeY, relativeX)) +
                currentHeading;

        // Normalize heading to 0-360 degrees
        targetHeading = ((targetHeading % 360) + 360) % 360;

        return new double[]{targetX, targetY, targetHeading};
    }
}
