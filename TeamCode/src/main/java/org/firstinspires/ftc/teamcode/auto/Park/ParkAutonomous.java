package org.firstinspires.ftc.teamcode.auto.Park;

import org.firstinspires.ftc.teamcode.auto.AutoConstants;
import org.firstinspires.ftc.teamcode.auto.AutoConstants.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.api.Paths;

import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import org.firstinspires.ftc.teamcode.pedro.Constants;

// ----------------WHAT THIS PROGRAM IS FOR------------------
// Start near the LOADING ZONE. Move into the LOADING ZONE to park.
public class ParkAutonomous extends LinearOpMode {

    // =========================================================================================
    // #region CONFIG NAMES
    // =========================================================================================

    protected double START_HEADING = 90;  // Default for red alliance.

    // =========================================================================================
    // #region DERIVED CONSTANTS -- DO NOT ADD VALUES
    // =========================================================================================


    // =========================================================================================
    // #region ACTUAL CODE
    // =========================================================================================

    protected String alliance;

    // The Follower that drives the robot. Set up in runOpMode(), so it's null until then.
    protected Follower follower;

    // Factory for making Poses. degrees() means headings are in degrees (0-360).
    // The alternative would be radians.
    protected final PoseFactory poseFactory = PoseFactory.degrees();

    // Ending position

    protected Pose START_POSE; // Assuming an 18x18 robot. units are all inches. USA USA USA.
    protected Pose END_POSE;

    public ParkAutonomous(String alliance) {
        this.alliance = alliance;
        if (alliance.equals("BLUE")) {
            // Super cool Pedro feature: all future poses created by poseFactory will be mirrored to Blue.
            poseFactory.mirrorAroundPoint(Field.HALF_WIDTH, Field.HALF_HEIGHT);
        } else if (!alliance.equals("RED")) {
            throw new IllegalArgumentException("Invalid alliance: " + alliance);
        }

        START_POSE = poseFactory.of(LoadingZone.X, LoadingZone.Y - LoadingZone.LENGTH / 2 - Robot.START_LENGTH / 2, START_HEADING);
        END_POSE = poseFactory.of(LoadingZone.X + Robot.START_WIDTH/2, LoadingZone.Y, START_HEADING);
    }

    public Command autoRoutine() {
        return sequential(
                follow(follower, parkAutonomous())  // Drive along the path returned by parkAutonomous()
        );
    }

    @Override
    public void runOpMode() {
        // Clear any leftover commands from a previous run.
        Scheduler.reset();

        // Build the Follower from your Constants file, which wires up motors and odometry using hardwareMap.
        follower = Constants.create(hardwareMap);

        // Tell the follower where the robot is right now. Without this it assumes (0, 0, 0).
        follower.setPose(START_POSE);

        // Run once so the follower updates its internal state before the match starts.
        follower.update();

        waitForStart();

        // Hand the routine to the Scheduler. It won't run yet. The loop below drives it.
        schedule(autoRoutine());

        while (opModeIsActive()) {
            // Refresh the robot's position estimate from odometry. Do this every loop.
            follower.update();

            // Advance the scheduled commands by one step. This is what actually makes the robot move.
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    // Builds the Path that drives from start to end.
    // Paths.line(...) makes a straight line between two Poses.
    // .constant(...) is a chained option that holds the robot's heading steady for this path.
    public Path parkAutonomous() {
        return Paths.line(START_POSE, END_POSE).constant(END_POSE);
    }
}