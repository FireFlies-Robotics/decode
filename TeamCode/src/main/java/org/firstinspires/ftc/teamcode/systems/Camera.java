package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Camera {
    OpMode opMode;
    public Camera(OpMode opMode){
        this.opMode = opMode;
    }
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"));
        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
    }   // end method initAprilTag()
    public double returnBearing() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        opMode.telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 20){
                opMode.telemetry.addLine("tag found");
                return detection.ftcPose.bearing;

            }
        }   // end for() loop
         return -999;

    }   // end method telemetryAprilTag()


}
