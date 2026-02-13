package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.List;

public class Camera {

    private final Limelight3A limelight;
    private final OpMode opMode;  // needed for telemetry

    public Camera(OpMode opMode) {
        this.opMode = opMode;
        limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0); // choose the correct pipeline
        limelight.start();           // REQUIRED for getLatestResult() to work
    }

    /**
     * Returns the bearing (horizontal angle) to tag 20.
     * Returns -999 if no valid tag 20 is detected.
     */
    public double getBearingToTag20() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return -999;
        }

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

        for (LLResultTypes.FiducialResult tag : tags) {
            if (tag.getFiducialId() == 20) {
                double bearing = tag.getTargetXDegrees();
                opMode.telemetry.addData("Tag 20 Bearing", bearing);
                return bearing;
            }
        }

        opMode.telemetry.addData("Tag 20 Bearing", "Not Detected");
        return -999;
    }

    public void stop() {
        limelight.stop();
    }
}