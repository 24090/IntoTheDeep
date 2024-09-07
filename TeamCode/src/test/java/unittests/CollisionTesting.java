package unittests;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;

import testutils.ProgramMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.pathfinding.Collision;
import org.firstinspires.ftc.teamcode.subsystems.pathfinding.RobotCollision;
import org.firstinspires.ftc.teamcode.subsystems.pathfinding.SquareCollision;
import org.junit.Assert;
import org.junit.Test;

import java.util.List;

public class CollisionTesting {
    private ProgramMecanumDrive drive = new ProgramMecanumDrive(new Pose2d(-100.0, 0.0, 0.0));
    private TrajectoryBuilder get_builder() {
        return new TrajectoryBuilder(
            drive.actionBuilder(new Pose2d(-100, 0,0)).getTrajectoryBuilderParams(),
            new Pose2d(-100, 0.0, 0.0), 0,
            drive.defaultVelConstraint,
            drive.defaultAccelConstraint
        );
    }
    private Vector2d[] robot_shape = new Vector2d[]{
        new Vector2d(-10,-10),
        new Vector2d(10,-10),
        new Vector2d(10,10),
        new Vector2d(-10,10),
    };
    @Test
    public void StraightLineTest(){
        List<Trajectory> trajectory = get_builder().lineToX(100).build();
        RobotCollision robot = new RobotCollision(robot_shape, trajectory.get(0));
        double end_time = new TimeTrajectory(trajectory.get(0)).duration;
        Assert.assertTrue(Collision.TestCollision(new SquareCollision(), robot, 0, end_time, 0.01));
    }
    @Test
    public void CurveTest(){
        List<Trajectory> trajectory = get_builder()
            .splineTo(new Vector2d(0,50), 0)
            .splineTo(new Vector2d(100,0),0)
            .build();
        RobotCollision robot = new RobotCollision(robot_shape, trajectory.get(0));
        double end_time = new TimeTrajectory(trajectory.get(0)).duration;
        Assert.assertFalse(Collision.TestCollision(new SquareCollision(), robot, 0, end_time, 0.01));
    }
}
