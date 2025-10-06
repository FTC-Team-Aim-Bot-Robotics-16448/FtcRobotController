package org.firstinspires.ftc.teamcode.aim;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;

import edu.wpi.first.math.MathUtil;

import org.json.JSONObject;
import org.json.JSONArray;

public class Vision {
    private Limelight3A camera;
    private Telemetry telemetry;
    private boolean isDataOld = false;
    private LLResult result;
    double cameraHeight;
    double cameraAngle;
    double targetHeight;
    private boolean isStarted = false;

    public static class ObjectDetectionResult {
        public double forwardOffset;
        public double strafeOffset;
        public double ta; // target area percentage
        public double tx; // horizontal offset
        public double ty; // vertical offset
    }

    public void init(final HardwareMap hardwareMap, Telemetry telemetry, String cameraName) {

        camera = hardwareMap.get(Limelight3A.class, cameraName);
        this.telemetry = telemetry;
        //camera.setPollRateHz(50);
    }

    public void startObjectDetection(int pipeline, double cameraHeight,
                                     double cameraAngle, double targetHeight) {
        if (this.isStarted) {
            this.stop();
        }
        this.cameraHeight = cameraHeight;
        this.cameraAngle = cameraAngle;
        this.targetHeight = targetHeight;
        camera.start();
        camera.pipelineSwitch(pipeline);

        this.isStarted = true;
    }

    public void stop() {
        if (this.isStarted) {
            camera.stop();
            this.isStarted = false;
        }
    }

    public double getTx(double defaultValue) {
        if (result == null) {
            return defaultValue;
        }
        return result.getTx();
    }

    public double getTy(double defaultValue) {
        if (result == null) {
            return defaultValue;
        }
        return result.getTy();
    }

    public String getRawJSON() {
        if (result == null) {
            return "";
        }
        return result.toString();  // Returns raw JSON string with 'pts' data
    }

    public double calculateOrientation() {
        String json = getRawJSON();
        if (json.isEmpty()) {
            return 0.0;
        }

        try {
            JSONObject jsonObj = new JSONObject(json);

            if (!jsonObj.has("pts")) {
                return 0.0; // No pts data found
            }

            JSONArray ptsArray = jsonObj.getJSONArray("pts");
            if (ptsArray.length() < 4) {
                return 0.0; // Need at least 4 values (2 points)
            }

            // Extract first two points: [x0,y0,x1,y1,...]
            double x0 = ptsArray.getDouble(0);
            double y0 = ptsArray.getDouble(1);
            double x1 = ptsArray.getDouble(2);
            double y1 = ptsArray.getDouble(3);

            // Calculate angle of first edge
            double angleRadians = Math.atan2(y1 - y0, x1 - x0);
            double angleDegrees = Math.toDegrees(angleRadians);

            // Normalize to -90 to +90 degrees
            while (angleDegrees > 90) angleDegrees -= 180;
            while (angleDegrees < -90) angleDegrees += 180;

            return angleDegrees;

        } catch (Exception e) {
            return 0.0; // JSON parsing error
        }
    }

    public boolean isTargetVisible() {
        if (result == null) {
            return false;
        }
        return !MathUtil.isNear(0, result.getTa(), 0.0001);
    }

    private double[] calTargetPosition(double tx, double ty) {
        // Adjust ty for camera mount angle
        double tyAdjusted = ty + this.cameraAngle;
        // Height difference (camera is ABOVE target)
        double heightDiff = this.cameraHeight - this.targetHeight;  // This will be positive

        // Calculate distance - use absolute value of tyAdjusted since target is below
        double distance;
        if (Math.abs(tyAdjusted) > 0.1) {  // Avoid division by zero
            distance = heightDiff / Math.tan(Math.toRadians(Math.abs(tyAdjusted)));
        } else {
            // Target is at same level as camera - use area method or set large distance
            distance = 100.0;  // Default distance
        }

        // Calculate relative position
        double relativeX = distance * Math.sin(Math.toRadians(tx)); // positive = right of robot
        double relativeY = distance * Math.cos(Math.toRadians(tx)); // positive = in front of robot

        return new double[]{relativeX, relativeY};
    }

    public double[] getTargetPosition() {
        return calTargetPosition(this.getTx(0), this.getTy(0));
    }

    public double getTargetForwardOffset() {
        double[] pos = getTargetPosition();
        return pos[1];
    }

    public double getTargetStrafeOffset() {
        double[] pos = getTargetPosition();
        return pos[0];
    }

    public Vision.ObjectDetectionResult getObjectDetectionResult() {
        if (result == null) {
            return null;
        }
        Vision.ObjectDetectionResult r = new Vision.ObjectDetectionResult();
        double pos[] = getTargetPosition();
        r.forwardOffset = pos[1];
        r.strafeOffset = pos[0];
        r.ta = result.getTa();
        r.tx = result.getTx();
        r.ty = result.getTy();
        result = null;
        return r;
    }

    public void update() {
        if (!this.isStarted) {
            return;
        }
        LLResult tmpResult = camera.getLatestResult();
        if (tmpResult != null && tmpResult.isValid()) {
            result = tmpResult;
            long staleness = result.getStaleness();
            // Less than 100 milliseconds old
            // TODO: could check this before assigning tmpResult to result
            isDataOld = staleness >= 100;
        }
    }
}