    package org.firstinspires.ftc.teamcode.systems;

    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.hardware.AnalogInput;
    import com.qualcomm.robotcore.hardware.CRServo;
    import com.qualcomm.robotcore.hardware.IMU;
    ;

    import com.acmerobotics.dashboard.FtcDashboard;
    import com.acmerobotics.dashboard.config.Config;
    import com.qualcomm.robotcore.hardware.Servo;
    import com.qualcomm.robotcore.util.Range;

    import org.firstinspires.ftc.teamcode.Utils.PID;
@Config
    public class TurretPosition {
        private double turretAngle = 60;
    private double lastDetectionTime = 0;

        public static double kp = 0.036;

        private LinearOpMode opMode;
        public Servo rightTurret;
        public Servo leftTurret;
        Camera camera;



        public TurretPosition(LinearOpMode opMode, Camera camera) {
            this.camera = camera;
            this.opMode = opMode;

            rightTurret = opMode.hardwareMap.get(Servo.class, "rightTurret");
            leftTurret = opMode.hardwareMap.get(Servo.class, "leftTurret");
        }
        public void setTurretPosition(double pos){
            rightTurret.setPosition(pos);
            leftTurret.setPosition(pos);
        }
        public void calculateTurretPosition(double offset) {
            double bearing = camera.getBearingToTag20();
            double adjustBearing = bearing -offset;
            double now = opMode.getRuntime();


            opMode.telemetry.addData("bearing", bearing);

            if (bearing != -999) {
                if (adjustBearing > 1.3 || adjustBearing < -1.3) {
                    turretAngle += (adjustBearing) * kp// * loopTime)
                     ;

                    double turretAngleAfterClip = Range.clip(turretAngle, 0, 120);

                    double turretPosition = turretAngleAfterClip / 120.0;

                    setTurretPosition(turretPosition);
                    opMode.telemetry.addData("turretAngle", turretAngle);
                    opMode.telemetry.addData("angle after clip", turretAngleAfterClip);
                    opMode.telemetry.addData("turret final position", turretPosition);
                }

            } else { // 400ms בלי זיהוי
                {
                    double middleAngle = 60; // middle of 0-120
                    double returnKp = 0.05; // adjust for speed of return
                    turretAngle += (middleAngle - turretAngle) * returnKp;
                    turretAngle = Range.clip(turretAngle, 0, 120);
                    setTurretPosition(turretAngle / 120.0);                }

            }
        }



    public void calculateTurretPositionRed(double offset){
        double bearing = camera.getBearingToTag24();
        double adjustBearing = bearing -offset;
        double now = opMode.getRuntime();


        opMode.telemetry.addData("bearing", bearing);

        if (bearing != -999) {
            if (adjustBearing > 1.3 || adjustBearing < -1.3) {
                turretAngle += (adjustBearing) * kp// * loopTime)
                ;

                double turretAngleAfterClip = Range.clip(turretAngle, 0, 120);

                double turretPosition = turretAngleAfterClip / 120.0;

                setTurretPosition(turretPosition);
                opMode.telemetry.addData("turretAngle", turretAngle);
                opMode.telemetry.addData("angle after clip", turretAngleAfterClip);
                opMode.telemetry.addData("turret final position", turretPosition);
            }

        } else { // 400ms בלי זיהוי
            {
                double middleAngle = 60; // middle of 0-120
                double returnKp = 0.05; // adjust for speed of return
                turretAngle += (middleAngle - turretAngle) * returnKp;
                turretAngle = Range.clip(turretAngle, 0, 120);
                setTurretPosition(turretAngle / 120.0);                }
        }
    }



    }
