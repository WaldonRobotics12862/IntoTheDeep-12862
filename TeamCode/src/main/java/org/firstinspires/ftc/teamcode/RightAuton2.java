package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.Arrays;

@Autonomous(name="RightAuton2", preselectTeleOp = "WaldonTeleOp")
public final class RightAuton2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Define all the locations that we need
        Pose2d beginPose = new Pose2d(15, -63, Math.toRadians(90));

        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, beginPose);
        DiveActions.Lift Lift = new DiveActions.Lift(hardwareMap);
        DiveActions.SpecimenDelivery SpecimenDelivery = new DiveActions.SpecimenDelivery(hardwareMap);
        DiveActions.SampleDelivery SampleDelivery = new DiveActions.SampleDelivery(hardwareMap);
        DiveActions.Ascend Ascend = new DiveActions.Ascend(hardwareMap);
        DiveActions.Intake Intake = new DiveActions.Intake(hardwareMap);

        DcMotorEx liftLeft = hardwareMap.get(DcMotorEx.class, "leftLift");
        liftLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        liftLeft.setDirection(DcMotorEx.Direction.FORWARD);
        liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotorEx liftRight = hardwareMap.get(DcMotorEx.class, "rightLift");
        liftRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        liftRight.setDirection(DcMotorEx.Direction.REVERSE);
        liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        RevBlinkinLedDriver LED = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        RevBlinkinLedDriver.BlinkinPattern pattern;
        pattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE;
        LED.setPattern(pattern);

        //RIGHT AUTON 2 PATH
        //This delivers one, then goes and pushes the specimen over into the observations zone and immeadiatly picks up a second
        //specimen that is already with the human player.
        //The human player builds a 3rd specimen while the second is being delivered
        //This path is only used if our partner does NOT load a specimen but this is our preferred path

        //new TranslationalVelConstraint(20.0)

        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(50.0),
                new AngularVelConstraint(Math.PI / 2)
        ));

        Action Deliver1 = drive.actionBuilder(new Pose2d(15, -63, Math.toRadians(90)))

                .lineToY(-32)
                .build();

        Action backup2 = drive.actionBuilder(new Pose2d(15, -32, Math.toRadians(90)))
                .lineToY(-40)
                .build();

        Action Pickup2 = drive.actionBuilder(new Pose2d(15, -40, Math.toRadians(90)))
                //.lineToY(-40)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(18,-40, Math.toRadians(90)), 0) // this should strafe right
                .splineTo(new Vector2d(33,-12), Math.toRadians(90))
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(60,-55, Math.toRadians(180)),Math.toRadians(45)) // push the sample over to the observation zone
                .setTangent(0)
                .lineToX(55)
                .splineToLinearHeading(new Pose2d(40, -45, Math.toRadians(-90)), 0)
                .build();

        Action Pickup22 = drive.actionBuilder(new Pose2d(40, -45, Math.toRadians(270)))
                .lineToY(-68)
                .build();

        Action Deliver2 = drive.actionBuilder(new Pose2d(40, -63, Math.toRadians(270)))
                .lineToY(-58)
                .turn(Math.toRadians(-90))
                .splineTo(new Vector2d(6, -28),Math.toRadians(90))
                .build();

        Action Pickup3 = drive.actionBuilder(new Pose2d(6, -33, Math.toRadians(90)))
                .lineToY(-38)
                .turn(Math.toRadians(-90))
                .splineTo(new Vector2d(40, -70), Math.toRadians(270)) // was 48, -65
                .build();

        Action Deliver3 = drive.actionBuilder(new Pose2d(40, -63, Math.toRadians(270)))
                .lineToY(-58)
                .turn(Math.toRadians(-90))
                .splineTo(new Vector2d(0, -26), Math.toRadians(90)) // was -5, -32
                .build();

        Action Park = drive.actionBuilder(new Pose2d(0, -33, Math.toRadians(90)))
                .lineToY(-38)
                .turn(Math.toRadians(-120)) // was 90
                .lineToX(48)
//                .splineTo(new Vector2d(40, -68), Math.toRadians(270))
                .build();


        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                Deliver1,
                                DiveActions.SpecimenDelivery.open()
                        ),
                        Deliver1,
                        DiveActions.Lift.liftToHeight(-950),
//                        new SleepAction(0.1),
                        backup2,
                        DiveActions.Lift.autonDown(),
                        Pickup2,
//                        new SleepAction(.1),// this is the pause to let someone build the specimen
                        Pickup22,
                        DiveActions.SpecimenDelivery.close(),
                        DiveActions.Lift.liftToHighChamber(),
                        new ParallelAction(
                                DiveActions.Lift.liftToHighChamber(),
                                Deliver2
                        ),
                        DiveActions.Lift.liftToHighChamber(),
                        DiveActions.SpecimenDelivery.open(),
                        new SleepAction(0.05 ),
                        DiveActions.Lift.autonDown(),
                        Pickup3,
                        DiveActions.SpecimenDelivery.close(),
                        DiveActions.Lift.liftToHighChamber(),
                        new ParallelAction(
                                DiveActions.Lift.liftToHighChamber(),
                                Deliver3
                        ),
                        DiveActions.Lift.liftToHighChamber(),
                        DiveActions.SpecimenDelivery.open(),
                        new SleepAction(0.05 ),
                        DiveActions.Lift.autonDown(),
                        Park
                )
       );
    }
}