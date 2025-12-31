package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="Pedro Auto Template", group="Auto")
public class PedroAutoTemplate extends OpMode {

    private Follower follower;

    // 1) Set your start pose (x,y,heading). Heading usually radians in Pedro.
    public static Pose startPose = new Pose(72, 130, Math.toRadians(90));

    // Example target poses
    public static Pose scorePose = new Pose(72, 72, Math.toRadians(90));
    public static Pose parkPose  = new Pose(120, 72, Math.toRadians(0));

    private PathChain mainChain;

    private enum AutoState { RUN_CHAIN, DONE }
    private AutoState state = AutoState.RUN_CHAIN;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // 2) Build paths
        Path toScore = new Path(new BezierLine(startPose, scorePose));
        toScore.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        Path toPark  = new Path(new BezierLine(scorePose, parkPose));
        toPark.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());

        // 3) Put them into a chain
        mainChain = new PathChain(toScore, toPark);

        telemetry.addLine("Ready");
    }

    @Override
    public void start() {
        // 4) Start following
        follower.followPath(mainChain, true); // true = hold end heading (common)
    }

    @Override
    public void loop() {
        // 5) MUST update every loop
        follower.update();

        switch (state) {
            case RUN_CHAIN:
                // 6) Decide when to do stuff
                // Option A: wait for all paths done
                if (!follower.isBusy()) {
                    state = AutoState.DONE;
                }
                break;

            case DONE:
                // stop motors / idle
                break;
        }

        Pose p = follower.getPose();
        telemetry.addData("x", p.getX());
        telemetry.addData("y", p.getY());
        telemetry.addData("h", p.getHeading());
        telemetry.addData("busy", follower.isBusy());
    }
}
