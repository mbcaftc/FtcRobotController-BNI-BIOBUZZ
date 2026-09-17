package org.firstinspires.ftc.teamcode.BioBuzz.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c ->
    {
        c.frontLeftName.set("FL");
        c.frontRightName.set("FR");
        c.backLeftName.set("BL");
        c.backRightName.set("BR");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.manualBrakeMode.set(true);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("ODO");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-3.7311490877406808);
        c.yPodOffset.set(1.6752501360074743);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.21522104892997287);
                Controller secondaryTranslationalForward = Controller.proportional(0.07951842676728337);
                Controller primaryTranslationalLateral = Controller.proportional(0.29823142435092936);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1101885424072446);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01715644453894683));
                c.brake.set(Controller.proportionalFeedforward(0.014582977858104806));

                c.headingFeedback.set(Controller.proportional(2.3921819029344715));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04503874597506732, 0.008128964359370312));

                c.linearBrakeCoefficients.set(Matrix.diag(0.05964506202498545, 0.06914223837710153));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.002367276385766134, 0.001992325022849771));

                c.maxAchievableForwardVelocity.set(60.46414609878146);
                c.maxAchievableStrafeVelocity.set(50.79217439224484);
                c.naturalForwardDeceleration.set(33.60575372570946);
                c.naturalStrafeDeceleration.set(51.00641392938486);
            }
    );



    public static Follower create(HardwareMap h) {
        return new Follower(new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig), new Foresight(foresightConfig));
    }
}
