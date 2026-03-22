package BluePath;

import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import MeepMeep.coordinates.BlueCoordinates.BlueCloseCoordinatesMeepMeep;

public class BlueClose {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(670);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(15.5, 17)
                .setConstraints(100, 70, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(
                myBot.getDrive().actionBuilder(BlueCloseCoordinatesMeepMeep.getStart())
                        // Go to first shooting position
                        .strafeTo(BlueCloseCoordinatesMeepMeep.getShooting().position)

                        // Go to first intake
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(
                                BlueCloseCoordinatesMeepMeep.getFirstIntakeStart(),
                                BlueCloseCoordinatesMeepMeep.getFirstIntakeStart().heading
                        )
                        .splineToLinearHeading(
                                BlueCloseCoordinatesMeepMeep.getFirstIntakeEnd(),
                                BlueCloseCoordinatesMeepMeep.getFirstIntakeEnd().heading
                        )

                        // Go back to first shooting
                        .strafeToLinearHeading(
                                BlueCloseCoordinatesMeepMeep.getShooting().position,
                                BlueCloseCoordinatesMeepMeep.getShooting().heading
                        )
                        .waitSeconds(2)

                        // Go to second intake
                        .splineToLinearHeading(
                                BlueCloseCoordinatesMeepMeep.getSecondIntakeStart(),
                                BlueCloseCoordinatesMeepMeep.getSecondIntakeStart().heading
                        )
                        .strafeToLinearHeading(
                                BlueCloseCoordinatesMeepMeep.getSecondIntakeEnd().position,
                                BlueCloseCoordinatesMeepMeep.getSecondIntakeEnd().heading
                        )

                        // Go back to last shooting
                        .setTangent(Math.toRadians(70))
                        .strafeToLinearHeading(
                                BlueCloseCoordinatesMeepMeep.getShooting().position,
                                BlueCloseCoordinatesMeepMeep.getShooting().heading
                        )
                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}