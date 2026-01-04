package BlueClose;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import MeepMeep.coordinates.BlueCloseCoordinatesMeepMeep;

public class BlueClose {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(670);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(BlueCloseCoordinatesMeepMeep.getStart())
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(BlueCloseCoordinatesMeepMeep.getFirstIntakeStart(), BlueCloseCoordinatesMeepMeep.getFirstIntakeStart().heading)
                .splineToSplineHeading(BlueCloseCoordinatesMeepMeep.getFirstIntakeEnd(), BlueCloseCoordinatesMeepMeep.getFirstIntakeEnd().heading)
                        .splineToLinearHeading(BlueCloseCoordinatesMeepMeep.getShooting(), BlueCloseCoordinatesMeepMeep.getShooting().heading)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}