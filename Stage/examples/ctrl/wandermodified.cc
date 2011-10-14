
#include "stage.hh"
#define PI 3.1415926
using namespace Stg;


const double avoidspeed = 1.6; 
const double minfrontdistance = 1.3;//1.3 good
const double coef_forgetting = 0.98;
const double scan_angle = PI/4; //scan from "scan_angle" to -"scan_angle"
const double fix_scan_angle = 2*PI/3;
const double maximal_wheel_speed = 2.6;// rot_wheel_speed * wheel_radius in [m/s]
const double robot_radius = 0.20;//[m]
const double sensor_precision = 0.005;

const double cruisespeed = 0.4; 
const double avoidturn = 0.1;
const bool verbose = false;
const bool with_noise = true;
const double stopdist = 0.3;
const int avoidduration = 10;
int stepcount = 0;

typedef struct
{
  double x;
  double y;
} vector_t;

typedef struct
{
  ModelPosition* pos;
  ModelRanger* laser;
  int avoidcount, randcount;
  double goal_orientation;
} robot_t;

typedef struct
{
  double orientation[800];
  double range[800];
  int length;
  double min[2]; //position and value
  double max[2]; //position and value
  double orientation_start;
  double orientation_end;
  vector_t past_pose; 
} scan_t;

bool Static_scan(double angle_start, double scan_fov, double rot_speed, scan_t* scan, robot_t* robot, Pose pose, double distance);
int LaserUpdate( Model* mod, robot_t* robot );
int PositionUpdate( Model* mod, robot_t* robot );
double Absolute(double a);// return absolute value of "a" = ||a||                 _________
double Norm_vector(double x, double y); // return the norm of the vector [x,y] = V x^2+y^2 
int Motor_control(robot_t* robot, double speed, double rot_speed, double orientation);// speed : in the direction of the goal, orientation: global orientation of the robot
double Simple_normal_deviate( double mean, double stddev );

// Stage calls this when the model starts up
extern "C" int Init( Model* mod, CtrlArgs* args )
{
  // local arguments
	/*  printf( "\nWander controller initialised with:\n"
			"\tworldfile string \"%s\"\n" 
			"\tcmdline string \"%s\"",
			args->worldfile.c_str(),
			args->cmdline.c_str() );
	*/

  robot_t* robot = new robot_t;
  
 
  robot->avoidcount = 0;
  robot->randcount = 0;
  
  robot->pos = (ModelPosition*)mod;

  if( verbose )
    robot->pos->AddCallback( Model::CB_UPDATE, (model_callback_t)PositionUpdate, robot );

  robot->pos->Subscribe(); // starts the position updates

  robot->laser = (ModelRanger*)mod->GetChild( "ranger:1" );
  robot->laser->AddCallback( Model::CB_UPDATE, (model_callback_t)LaserUpdate, robot );
  robot->laser->Subscribe(); // starts the ranger updates
   
  return 0; //ok
}


// inspect the ranger data and decide what to do
int LaserUpdate( Model* mod, robot_t* robot )
{
    // get the data
  const std::vector<meters_t>& new_scan = robot->laser->GetRanges();
  uint32_t sample_count = new_scan.size();
  if( sample_count < 1 )
    return 0;
  
  double distance;
  if (with_noise==true)
    distance=Simple_normal_deviate(0, sensor_precision)+new_scan[1];
  else
    distance=new_scan[1];
  
  //bool obstruction = false;
  //bool stop = false;
  // find the closest distance to the left and right and check if
  // there's anything in front
  //double minleft = 1e6;
  //double minright = 1e6;
  double distance_traveled=0;
  //double speedX,speedY,orientation;
  //double tmp_orientation;

  //static bool rotate_clockwise = true;
  static bool scan_finished = false;
  //static double pos_start;
  //static int i=0;
  static scan_t scan;
  //static vector_t vector_avoidance, speed_avoidance;
  Pose pose = robot->pos->GetPose();
 // static Pose past_pose;

  
  if(scan_finished==false)
    scan_finished = Static_scan(-PI/2.0+robot->goal_orientation, PI, 2, & scan, robot, pose, distance);
  else
  {
    Motor_control(robot, 1,0,pose.a);
    
    /*if((stepcount%2) == 1)
    {
      
      if( normalize( pose.a - robot->goal_orientation ) > scan_angle)
	rotate_clockwise=true;
      else if (normalize( pose.a-robot->goal_orientation ) < -scan_angle)
	rotate_clockwise=false;
      if ( rotate_clockwise )
      {
	//printf("clockwise\n"); 
	robot->pos->SetSpeed( 0,0,-8 );
      }
      else
      {
	//printf("counterclockwise\n");
	robot->pos->SetSpeed( 0,0,8 );
      }
      //printf("Rotate, goal orientation %.2f robotX %.2f, robotY %.2f robot orientation %.2f\n", robot->goal_orientation, pose.x, pose.y, pose.a);
      
    }
    else
    {
      robot->pos->SetTurnSpeed(0);
      double coef_avoidance=0;
      tmp_orientation = normalize(robot->goal_orientation - pose.a);
      vector_avoidance.x*=coef_forgetting;
      vector_avoidance.y*=coef_forgetting;
      if(distance<minfrontdistance)//distance current distances
      {
	printf("avoid %.2lf\n",distance);
	coef_avoidance=0.4/(1.0*distance);
	if (coef_avoidance>avoidspeed)
	  coef_avoidance=avoidspeed;
	vector_avoidance.x+=coef_avoidance*cos(pose.a+PI); //vector_avoidance global repulsive vector
	vector_avoidance.y+= coef_avoidance*sin(pose.a+PI);
	double norm;
	norm=Norm_vector(vector_avoidance.x,vector_avoidance.y);
	if(norm>avoidspeed)
	{
	  vector_avoidance.x/=norm/avoidspeed;
	  vector_avoidance.y/=norm/avoidspeed;
	}
      }
      speed_avoidance.x = vector_avoidance.x*cos(pose.a) - vector_avoidance.y*sin(pose.a); // vector in the robot frame
      speed_avoidance.y = vector_avoidance.x*sin(pose.a) + vector_avoidance.y*cos(pose.a); 	
      speedX= cos(tmp_orientation)+speed_avoidance.x;
      speedY= sin(tmp_orientation)+speed_avoidance.y; 
      robot->pos->SetSpeed(speedX,speedY,0 );
      //printf("robot orientation %.2f goal orientation %.2f robotX %.2f, robotY %.2f, tmp_orientation %.2f,  SpeedX: %.2f, SpeedY: %.2f\n", pose.a, robot->goal_orientation, pose.x, pose.y,tmp_orientation,speedX, speedY);	
      printf("Coef: %.3f Vector[%.3f %.3f] Speed_avoidance [%.3f %.3f] Speed [%.2f %.2f] \n",coef_avoidance,vector_avoidance.x, vector_avoidance.y,speed_avoidance.x, speed_avoidance.y, speedX,speedY);
    }
    stepcount++*/;
    
    distance_traveled=Norm_vector(pose.x-scan.past_pose.x, pose.y-scan.past_pose.y);
    printf("Go straight until %.1f m. Distance travelled: %.2f m\n",scan.max[1], distance_traveled);
 
    if(scan.max[1] > 6)
      scan.max[1]=6;
    if ( distance_traveled> scan.max[1]-1 )
    {
      scan_finished=false;
      robot->pos->SetSpeed( 0,0,0 );
    }
  }
  
  /*
  for (uint32_t i = 0; i < sample_count; i++)
  {
    
    if( verbose ) printf( "%.3f ", scan[i] );
    
    if( (i > (sample_count/3)) 
      && (i < (sample_count - (sample_count/3))) 
      && scan[i] < minfrontdistance)
      {
	if( verbose ) puts( "  obstruction!" );
	obstruction = true;
}

if( scan[i] < stopdist )
{
  if( verbose ) puts( "  stopping!" );
  stop = true;
}

if( i > sample_count/2 )
  minleft = std::min( minleft, scan[i] );
else      
  minright = std::min( minright, scan[i] );
}

if( verbose ) 
{
  puts( "" );
  printf( "minleft %.3f \n", minleft );
  printf( "minright %.3f\n ", minright );
}

if( obstruction || stop || (robot->avoidcount>0) )
{
  if( verbose ) printf( "Avoid %d\n", robot->avoidcount );
  
  robot->pos->SetXSpeed( stop ? 0.0 : avoidspeed );      
  
  // once we start avoiding, select a turn direction and stick
  //with it for a few iterations 
  if( robot->avoidcount < 1 )
  {
    if( verbose ) puts( "Avoid START" );
    robot->avoidcount = random() % avoidduration + avoidduration;
    
    if( minleft < minright  )
    {
      robot->pos->SetTurnSpeed( -avoidturn );
      if( verbose ) printf( "turning right %.2f\n", -avoidturn );
}
else
{
  robot->pos->SetTurnSpeed( +avoidturn );
  if( verbose ) printf( "turning left %2f\n", +avoidturn );
}
}

robot->avoidcount--;
}
else
{
  if( verbose ) puts( "Cruise" );
  
  robot->avoidcount = 0;
  robot->pos->SetXSpeed( cruisespeed );	  
  robot->pos->SetTurnSpeed(  0 );
}

//  if( robot->pos->Stalled() )
// 	 {
  // 		robot->pos->SetSpeed( 0,0,0 );
  // 		robot->pos->SetTurnSpeed( 0 );
  // }
  */		
  
  return 0; // run again
}

bool Static_scan(double angle_start, double scan_fov, double rot_speed, scan_t* scan, robot_t* robot, Pose pose, double distance)
{
  static bool scan_init = true, scan_pending=false, scan_finished = false;
  static int ii;
  static double rotation_angle=0;
  double diff_orientation, speedW;
  
   if (scan_init==true)
  {
     scan_finished=false;
    diff_orientation=normalize(pose.a-angle_start);
    if( Absolute(diff_orientation) >0.01) // go to angle_start
    {
      printf("Rotate to the start angle %.3f\n", diff_orientation);
      if(diff_orientation*10>maximal_wheel_speed/robot_radius)
	speedW=-maximal_wheel_speed;
      else if(diff_orientation*10<-maximal_wheel_speed/robot_radius)
	speedW=maximal_wheel_speed;
      else
	speedW=-diff_orientation*10;
      robot->pos->SetSpeed( 0,0,speedW );	//P controller
    }
    else
    {
      ii=0;
      rotation_angle=0;
      scan_init=false;
      scan_pending=true;
      scan->past_pose.x=pose.x;
      scan->past_pose.y=pose.y;
      scan->orientation_start=angle_start;
      if( rot_speed>=0 )
	scan->orientation_end=normalize(angle_start+scan_fov);
      else
	scan->orientation_end=normalize(angle_start-scan_fov);
      scan->orientation[ii]=pose.a;
      scan->range[ii]=distance;
      scan->min[1]=distance;
      scan->min[0]=ii;
      scan->max[1]=distance;
      scan->max[0]=ii;
      printf( "Scan  start\n" );
      robot->pos->SetTurnSpeed(rot_speed);
      ii++;
    }
  }
  else if(scan_pending == true) // until scan is finished
  {
    if( rotation_angle < scan_fov ) // to symetrise, doesn't work when clockwise rotation
    {
      scan->orientation[ii]=pose.a;
      scan->range[ii]=distance;
      if(scan->min[1]>distance)
      {
	scan->min[1]=distance;
	scan->min[0]=ii;
      }
      if(scan->max[1]<distance)
      {
	scan->max[1]=distance;
	scan->max[0]=ii;
      }
      rotation_angle+=Absolute(normalize(pose.a-normalize(rotation_angle+scan->orientation_start)));
      printf("\rScanning[%d%]",(ii*100)/410);
      ii++;
    }
    else// one turn --> stop
    {
      robot->pos->SetTurnSpeed(0);
      scan->length=ii-1;
      scan_pending=false;// finish rotation
      scan_finished=true;
      //improve choice of direction
      if(true)
      {
	double delta=(scan_fov)/(1.0*scan->length);
	printf("delta:%.2lf\n",delta);
	int k=21; //width of the window : k*delta
	double *optimal_r = (double *)malloc((scan->length+1)*sizeof(double));
	double *average_r = (double *)malloc((scan->length+1)*sizeof(double));
	for(int i=0; i<(scan->length-k); i++)
	{
	  double small_r=9999;
	  average_r[i]=0.0;
	  for(int j=i; j<i+k; j++ )
	  {
	    if( (scan->range[j]-1) < small_r)
	      small_r = scan->range[j]-1;
	    average_r[i] = average_r[i]+scan->range[j]-1;
	  }
	  optimal_r[i] = small_r;
	  average_r[i] = average_r[i]/(1.0*k);
	}
	double large_r = -1;
	int index_r=0;
	for(int i=0; i<(scan->length-k); i++)
	{
	  double tmp;
	  tmp = 2.0* optimal_r[i] * sin(delta*(k-1)/2.0); 
	  printf("[%d], tmp:%.2lf, optimal_r:%.2lf, large_r:%.2lf\n",i,tmp,average_r[i],large_r);
	  if(tmp > (2.0*robot_radius*1.5) && average_r[i] > large_r)
	  {
	    large_r = average_r[i];
	    index_r = i;
	  }
	}
	scan->max[1] = scan->range[index_r+(int)(k/2)];
	scan->max[0] = scan->orientation[index_r+(int)(k/2)];
	robot->goal_orientation = scan->orientation[index_r+(int)(k/2)];
	printf("o:%.2lf,r:%.2lf,large_r:%.2lf\n",robot->goal_orientation,scan->max[1],large_r);
      }
      else
	robot->goal_orientation=scan->orientation[(int)(scan->max[0])];//go farthest
      
      printf("\nEnd scan: #measurements %d, global goal: [%.2f %.2f], orientation robot: %.2f, index %d\n",scan->length,robot->goal_orientation,scan->max[1], pose.a,0);		
    }
  }
  else
    printf("Error scan\n");
  
  if (scan_finished==true)
  {
    scan_init=true;
    printf("Scan finished\n");
    return true ;
  }
  else
    return false;
}

int PositionUpdate( Model* mod, robot_t* robot )
{
  Pose pose = robot->pos->GetPose();

  printf( "Pose: [%.2f %.2f %.2f %.2f]\n",pose.x, pose.y, pose.z, pose.a );

  return 0; // run again
}

double Absolute(double a)
{
  if(a>=0)
    return(a);
  else
    return(-a);
}

double Norm_vector(double x, double y)
{
  return( sqrt( (x)*(x)+(y)*(y) ) );
}

int Motor_control(robot_t* robot, double speed, double rot_speed, double orientation)// speed : in the direction of the goal, orientation: global orientation of the robot
{
  double speedX,speedY,speedW, rot_speed_radius, tmp_orientation;
  double v1, v2, v3, v4, coef, factor, max;
  
  tmp_orientation = normalize(robot->goal_orientation - orientation);
  
  speedX= speed*cos(tmp_orientation);
  speedY= speed*sin(tmp_orientation); 
  speedW= rot_speed;
  
  // check if max wheel speed is overpassed
  coef=sqrt(2)/2;
  rot_speed_radius=speedW*robot_radius;  // rot_speed_radius is rot. speed * radius of the robot in [m/s]
  v1=-coef*speedX + coef*speedY + rot_speed_radius;
  v2=-coef*speedX - coef*speedY + rot_speed_radius;
  v3= coef*speedX - coef*speedY + rot_speed_radius;
  v4= coef*speedX + coef*speedY + rot_speed_radius;
  
  max =Absolute(v1);
  if(v2>max)
    max=Absolute(v2);
  if(v3>max)
    max=Absolute(v3);
  if(v4>max)
    max=Absolute(v4);
   
  if (max > maximal_wheel_speed)
  {
    factor = maximal_wheel_speed/max;
    speedX*=factor;
    speedY*=factor;
    speedW*=factor;
    printf("Factor of max speed and max : %.2f %.2f\n", factor, max);
  }
  
  robot->pos->SetSpeed(speedX,speedY,speedW );
  
  return 0;
}

double Simple_normal_deviate( double mean, double stddev )
{
  double x = 0.0;
  
  for( int i=0; i<12; i++ )
    x += rand();
  x/=(RAND_MAX+1.0);
  
  return ( stddev * (x - 6.0) + mean );  
}
