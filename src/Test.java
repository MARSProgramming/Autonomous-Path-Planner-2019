import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.Waypoint;

public class Test {
	
	public static void main(String[] args) {
	
//		Waypoint[] points = new Waypoint[] {
//			    new Waypoint(-4, -1, Pathfinder.d2r(-45)),      // Waypoint @ x=-4, y=-1, exit angle=-45 degrees
//			    new Waypoint(-2, -2, 0),                        // Waypoint @ x=-2, y=-2, exit angle=0 radians
//			    new Waypoint(0, 0, 0)                           // Waypoint @ x=0, y=0,   exit angle=0 radians
//			};
	
		Waypoint[] points = new Waypoint[] {
				new Waypoint(0, 0, 0),
				new Waypoint(1.524, 0, 0)
		};
		
			Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_FAST, 0.04, 1.5, 2.0, 60.0);
			Trajectory trajectory = Pathfinder.generate(points, config);
			
			for(int i = 0; i < trajectory.length(); i++) {
			
				Segment seg = trajectory.segments[i];
				
//			    System.out.printf("%f,%f,%f,%f,%f,%f,%f,%f\n", 
//		        seg.dt, seg.x, seg.y, seg.position, seg.velocity, 
//		            seg.acceleration, seg.jerk, seg.heading);
	
				System.out.println(i + "\t" + seg.velocity);
				
			}
	}
		
}
