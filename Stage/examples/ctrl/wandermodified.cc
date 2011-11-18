
#include "stage.hh"
#define PI 3.1415926
using namespace Stg;


const double avoidspeed = 1.6; 
const double minfrontdistance = 1.3;
const double coef_forgetting = 0.98;
const double scan_angle = PI; //scan from "scan_angle" to -"scan_angle"
const double scan_speed = 2; // rotational speed during the scan [rad/s]
const int scan_analyse = 3; // choose the algorithm you want{ 0:FirstMax, 1:FixedMovingWindow, 2:FocusOnMax 3:FollowWall}

//robot
const double maximal_wheel_speed = 2.6; // rot_wheel_speed * wheel_radius in [m/s]
const double robot_radius = 0.20; //[m]
const double dist_safe = 0.5; // [m] 
const double odometry_precision = 0.001;//[m] and [radian]

//Sensor
const double sensor_max_range = 10; //max range of the sensor [m]
const double sensor_min_range = 0.2; //min range of the sensor [m]
const double sensor_precision = 0.005;
const bool with_noise = false; // for the laser and the position

const double cruisespeed = 0.4; 
const double avoidturn = 0.1;
const bool verbose = false;

const double stopdist = 0.3;
const int avoidduration = 10;
int stepcount = 0;

bool FollowWall_activated=false;

typedef struct
{
  double x;
  double y;
  Pose pose;
} vector_t;

typedef struct
{
  ModelPosition* pos;
  ModelRanger* laser;
  int avoidcount, randcount;
  double goal_orientation;
  double goal_distance;
} robot_t;

typedef struct
{
  double orientation[800];
  double range[800];
  int length;
  double delta; // angle between two measurements
  double min[2]; //position and value
  double max[2]; //position and value
  double orientation_start;
  double orientation_end;
  vector_t past_pose; 
} scan_t;

bool FollowWall(scan_t* scan, robot_t* robot, Pose pose, double distance );
bool LeastSquares( vector_t data[], int data_length, vector_t* parameters);
bool Static_scan(double angle_start, double scan_fov, double rot_speed, scan_t* scan, robot_t* robot, Pose pose, double distance);
void FocusOnMax(scan_t* scan, robot_t* robot);
void FixedMovingWindow( scan_t* scan, robot_t* robot);
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
  
  robot->laser = (ModelRanger*)mod->GetChild( "ranger:0" );
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
  
  double distance=new_scan[0];
  if (with_noise==true)
    distance+=Simple_normal_deviate(0, sensor_precision);
  
  if(distance > sensor_max_range)
    distance = sensor_max_range;
  if(distance < sensor_min_range)
    distance = sensor_min_range;
  
  //bool obstruction = false;
  //bool stop = false;
  // find the closest distance to the left and right and check if
  // there's anything in front
  //double minleft = 1e6;
  //double minright = 1e6;
  static bool init=true, scan_finished = false;
  
  double distance_traveled=0;
  
  static double angle_start=scan_angle, rot_speed=-scan_speed;
  //double speedX,speedY,orientation;
  //double tmp_orientation;
  
  //static bool rotate_clockwise = true;
  
  //static double pos_start;
  //static int i=0;
  static scan_t scan;
  //static vector_t vector_avoidance, speed_avoidance;
  
  Pose pose = robot->pos->GetPose();; 

  // static Pose past_pose;
  
  if(scan_finished==false)
  {
    if(init==true)
    {
      rot_speed=-rot_speed;
      angle_start=-angle_start;
      init=false;
    }
    else
    {
      scan_finished = Static_scan(angle_start+robot->goal_orientation, 2*scan_angle, rot_speed, & scan, robot, pose, distance);  
    }
  }
  else if(FollowWall_activated==true)
  {
    FollowWall(& scan, robot, pose, distance);
  }
  else
  {
    if(robot->goal_distance<=0)
      Motor_control(robot, -maximal_wheel_speed,0,pose.a);
    else
      Motor_control(robot, maximal_wheel_speed,0,pose.a);
    
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
  printf("Go straight until %.1f m. Distance travelled: %.2f m\n",robot->goal_distance, distance_traveled);
  
  if ( distance_traveled>= Absolute(robot->goal_distance) )
  {
    scan_finished=false;
    init=true;
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

bool FollowWall(scan_t* scan, robot_t* robot, Pose pose, double distance )
{
  double angle_offset=PI/4,diff_orientation, threshold_dist=2*dist_safe, diff_dist, threshold_change_curvature=0.2;
  static double curvature=0, perpendicular_orientation=99;
  static bool follow_init=true,follow_start=false;
  double rot_speed=0, lin_speed=maximal_wheel_speed;
  
  static vector_t data[2], parameters; 
  int data_length=2; 
  static int current_new_data=0,jj=0;
  double tX,tY;
  
  if(follow_init==true)
  {
    if(Absolute(perpendicular_orientation-99)<0.001)//if perpendicular_orientation == 99, the perpendicular is the min of the scan
    {
      diff_orientation=normalize(pose.a-scan->orientation[(int)scan->min[0]]-angle_offset);
      printf("Perpendicular is min : %.2f \n",scan->orientation[(int)scan->min[0]]);
    }
    else
    {
      diff_orientation=normalize(pose.a-perpendicular_orientation-angle_offset);
      printf("Perpendicular is %.2f\n", perpendicular_orientation);
    }
      if(Absolute(diff_orientation) > 0.01)
    {
      //printf("\nRotate to the start angle %.3f ", diff_orientation);
      Motor_control(robot, 0, -diff_orientation*10, 0); //P controller
    }
    else
    {
      follow_init=false;
      follow_start=true;
    }
  }
  else if(follow_start==true)
  {
    diff_dist=distance-threshold_dist;
    if(diff_dist>3*threshold_change_curvature)//corner
    {
      curvature=-0.5;
      rot_speed=-1;
      lin_speed=maximal_wheel_speed/4;
      printf("Outside Corner\n");
      current_new_data=0;//reset the counter
    }
    else if(diff_dist<-threshold_change_curvature)
    {
      if(diff_dist<-2*threshold_change_curvature)
      {
	rot_speed=0.5;
	lin_speed=distance*1;
	curvature= -3*PI/2+angle_offset; //go back
	printf("Inside Corner\n");
      }
      else
      {
	curvature=0.3;
	printf("Start a curve\n");
      }
      current_new_data=0;//reset the counter
    }
    else // do the curve and estimate the position of the wall
    {
      curvature-=0.02;
      printf("Do the curve\n");
     
      
      if(jj++==5)// estimate the position of the wall
      {
	jj=0;
	data[current_new_data].x = distance*cos(pose.a); //store the current data
	data[current_new_data].y = distance*sin(pose.a);
	data[current_new_data].pose.x=pose.x;
	data[current_new_data].pose.y=pose.y;
	current_new_data++;
	if(current_new_data>data_length-1)
	{
	  for(int i=0;i<current_new_data-1;i++) //adapt the data in the current frame of the robot
	  {
	    tX=pose.x-data[i].pose.x; //translation in the global frame,centered on the robot
	    tY=pose.y-data[i].pose.y;
	    printf("tx %.2f\tty %.2f\n",tX,tY);
	    printf("1 Data%d, x %.2f\ty %.2f\tpose [%.2f, %.2f] \n",i, data[i].x,data[i].y,data[i].pose.x,data[i].pose.y);
	    data[i].y= -tY+data[i].y;
	    data[i].x= -tX+data[i].x;
	    data[i].pose.x=pose.x;
	    data[i].pose.y=pose.y;
	    printf("2 Data%d, x %.2f\ty %.2f\tpose [%.2f, %.2f] \n",i, data[i].x,data[i].y,data[i].pose.x,data[i].pose.y);
	  }
	  
	  LeastSquares(data,data_length,&parameters);//estimate the parameters
	  printf("Parameters : %.2f \t %.2f", parameters.x,parameters.y );
	  
	  perpendicular_orientation=atan(-1/parameters.x); // compute the perpendicular direction to the wall
	  if(normalize(perpendicular_orientation-pose.a)>PI/2) //the perpendicular direction cannot be outside of [-PI/2, PI/2] relatively to the robot
	  {
	    perpendicular_orientation=normalize(perpendicular_orientation-PI);
	  }
	  else if(normalize(perpendicular_orientation-pose.a)<-PI/2)
	  {
	    perpendicular_orientation=normalize(perpendicular_orientation+PI);
	  }
	  printf("Least Squares, perpendicular : %.2f\n",perpendicular_orientation);
	  
	  if( Absolute(normalize(pose.a-(perpendicular_orientation+angle_offset))) > 0.1 ) // correct the bearing of the robot if the robot has not the correct offset 
	  {
	    follow_init=true;
	    follow_start=false;
	    printf("Offset has to be corrected\n");
	  }
	  else
	  {
	    perpendicular_orientation=99;
	  }
	  current_new_data=0;//reset the counter
	}
      }
    }
    robot->goal_orientation=pose.a+PI/2-angle_offset+curvature;
    printf("Follow wall goal orientation %.2f\n\n",robot->goal_orientation);
    Motor_control(robot, lin_speed,rot_speed,pose.a);
  }
  else
    printf("Error Follow_wall\n");
  return 0;
}

bool LeastSquares( vector_t data[], int data_length, vector_t* parameters)
{
  // least squares with y=a*x+b, with are looking for a and b which will be stored in parameters ; x and y are known and store in data
 
   printf("LS: Data0x %.2f\tData0y %.2f\tData1x %.2f\tData1y %.2f\t", data[0].x, data[0].y,data[1].x, data[1].y );
  parameters->x = (data[1].y-data[0].y)/(data[1].x-data[0].x); // not least squares : a = (y1-y0)/(x1-x0) and b= y1-a*x1
  parameters->y = data[1].y-parameters->x*data[1].x;
   printf("Parameters : %.2f \t %.2f\n", parameters->x,parameters->y );
  
  return 0;
}

double PID_controller(double measured_value, double desired_value, double K, double Ti, double Td)
{
  
  return 0;
}

bool Static_scan(double angle_start, double scan_fov, double rot_speed, scan_t* scan, robot_t* robot, Pose pose, double distance)
{
  static bool scan_init = true, scan_pending=false, scan_finished = false;
  static int ii;
  double diff_orientation;
  
  if (scan_init==true)
  {
    scan_finished=false;
    diff_orientation=normalize(pose.a-angle_start);
    if( Absolute(diff_orientation) >0.01) // go to angle_start
    {
      //printf("\nRotate to the start angle %.3f ", diff_orientation);
      Motor_control(robot, 0, -diff_orientation*10, 0); //P controller
    }
    else
    {
      ii=0;
      scan_init=false;
      scan_pending=true;
      scan->past_pose.x=pose.x;
      scan->past_pose.y=pose.y;
      scan->orientation_start=pose.a;
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
    if(ii==5)
    {
      scan->delta=Absolute(normalize(pose.a-scan->orientation_start)/4); // the 2 first samples have the same orientation
      printf("Delta = %.3f",scan->delta);
    }
    if( ii*scan->delta < scan_fov )
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
      
      //       rotation_angle+=Absolute(normalize(pose.a-normalize(rotation_angle+scan->orientation_start)));
      printf("%.2f, %.2f\n",pose.a, distance);
      //       printf("\rScanning[%d%]",(ii*100)/410);
      ii++;
    }
    else// one turn --> stop
    {
      robot->pos->SetTurnSpeed(0);
      scan->length=ii-1;
      scan_pending=false;// finish rotation
      scan_finished=true;
      
      switch(scan_analyse)
      {
	case 0 : robot->goal_orientation=scan->orientation[(int)(scan->max[0])];break; // FirstMax go farthest
	case 1 : FixedMovingWindow(scan,robot);break;// FixedMovingWindow
	case 2 : FocusOnMax(scan,robot);break;
	case 3 : FollowWall_activated=true;break;
	default : printf("\nWrong scan analyse !\n");break;
      }
      //printf("\nEnd scan: #measurements %d, global goal: [%.2f %.2f], orientation robot: %.2f\n",scan->length,robot->goal_orientation,robot->goal_distance, pose.a);		
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

void FocusOnMax(scan_t* scan, robot_t* robot)
{
  int i,j=0,k,l, j_max, window_size, optimal_i, nb_consecutive_free_windows=0;
  bool no_window_free=true;
  double threshold_slope=0.5, threshold_diff_slope=0.9, slope0, slope1, diff_slope, alpha=0,optimal_r=0,dist_to_travel=0;
  int *possible_i = (int *)malloc((scan->length)*sizeof(int));
  double *possible_r = (double *)malloc((scan->length)*sizeof(double));
  
  /* 
  There are i samples in the scan
  There are j discontinuities
  There are k windows
  There are l samples per window
  */
  
  for(i=1;i<scan->length-2;i++) // find extremums in the scan
  {
    slope0 = scan->range[i]-scan->range[i-1];
    slope1 = scan->range[i+1]-scan->range[i];
    if(slope0 < -threshold_slope || slope1 > threshold_slope) //if slope is steep
    {
      diff_slope=Absolute( slope1-slope0 );
      printf("slope : %.3f, %.3f\t acc: %.3f\n",slope0, slope1, diff_slope );
      if( diff_slope > threshold_diff_slope ) //if slope_i is different from slope_i+1 
      {
	possible_i[j]=i;
	possible_r[j]=scan->range[i];
	printf("Extremum %d : orientation %.2f, range %.2f\n",j,scan->orientation[possible_i[j]], possible_r[j]);
	j++;
      }
    }
  }
  
  j_max=j;
  for(j=0;j<j_max;j++) // check if extremums are good direction
  {
    alpha=asin( (robot_radius+dist_safe/2)/possible_r[j] );
    window_size= (int)(alpha/scan->delta)+1;
    printf("orientation %.2f, window %d, %.2f\n",scan->orientation[possible_i[j]],window_size, alpha);
    bool *is_window_ok = (bool *)malloc((3*window_size)*sizeof(bool));
    
    for(k=0;k<3*window_size;k++) // moving the window
    {
      if(k>0.8*window_size && k<2.2*window_size) // do not check these windows because the extremum is in these windows
      {
	is_window_ok[k]=false;
      }
      else
      {
	is_window_ok[k]=true;
	//printf(" window from %.2f to %.2f\n",scan->orientation[possible_i[j]+k-2*window_size], scan->orientation[possible_i[j]+k-window_size]);
	for(l=0;l<window_size;l++) // check each sample in the window
	{
	  if( possible_i[j]-window_size+k-l<0 || possible_i[j]-window_size+k-l>scan->length ) //avoid going out of the table
	  {
	    is_window_ok[k]=false;
	    // printf("out of table\n");
	  }
	  else
	  {
	    if( scan->range[ possible_i[j]-window_size+k-l ] < possible_r[j] )
	    {
	      is_window_ok[k]=false;
	      //printf("Obstacle -> false: %.3f, %.3f\n",scan->range[ possible_i[j]-window_size+k-l ], possible_r[j]);
	    }
	    if( l == window_size-1 && is_window_ok[k]==true)// if the window is free
	    {
	      no_window_free=false;
	      if(k>=1 && is_window_ok[k-1]==true)//sum the number of consecutive free windows
	      {
		nb_consecutive_free_windows++;
	      }
	      else
	      {
		nb_consecutive_free_windows=0;
	      }
	      if( possible_r[j] >= optimal_r ) // if this window has the longest range, go in this direction
	      {
		optimal_i=possible_i[j]+k-(int)(nb_consecutive_free_windows/2) -window_size-(int)(window_size/2);
		optimal_r=possible_r[j];
	      }
	      printf( "The window [%.3f %.3f] is free. CenterOfWindow: %.3f  RangeOfDiscontinuity: %.3f\n",scan->orientation[possible_i[j]+k-2*window_size], scan->orientation[possible_i[j]+k-window_size], scan->orientation[possible_i[j]+k - window_size-(int)(window_size/2)],  optimal_r);
	    }
	  }
	}
      }
    }
  }
  
  if(no_window_free==false)//go in the direction of the best window
  {
    dist_to_travel = optimal_r-dist_safe;
    if(dist_to_travel<0)
    {
      dist_to_travel=0;
      robot->goal_orientation-=scan_angle/4;
    }
    else
    {
      robot->goal_orientation=scan->orientation[optimal_i];
    }
    robot->goal_distance=dist_to_travel;
  }
  else // rotate a little bit and rescan
  {
    printf("No window free\n");
    robot->goal_orientation-=scan_angle/4;
    robot->goal_distance=0;
  }
}

void FixedMovingWindow( scan_t* scan, robot_t* robot)
{
  int k=10; //width of the window : k*delta
  double *optimal_r = (double *)malloc((scan->length+1)*sizeof(double));
  double *average_r = (double *)malloc((scan->length+1)*sizeof(double));
  
  for(int i=0; i<(scan->length-k); i++)
  {
    double small_r=sensor_max_range;
    average_r[i]=0.0;
    for(int j=i; j<i+k; j++ ) // search the min and the average for each window
    {
      if( (scan->range[j]) < small_r)
      {  
	small_r = scan->range[j];
      }
      average_r[i] = average_r[i]+scan->range[j];
    }
    optimal_r[i] = small_r;
    average_r[i] = average_r[i]/(1.0*k);
  }
  
  double large_r = -1;
  int index_r=0;
  for(int i=0; i<(scan->length-k); i++)
  {
    double tmp;
    tmp = 2.0* (optimal_r[i]-0.5) * sin(scan->delta*(k-1)/2.0); 
    if(tmp > (2.0*robot_radius*1.5) && average_r[i] > large_r)
    {
      large_r = average_r[i];
      index_r = i;
    }
    printf("[%d], tmp:%.2lf, optimal_r:%.2lf, large_r:%.2lf\n",i,tmp,optimal_r[i],large_r);
  }
  if(optimal_r[index_r]>1 && large_r>1)
  {
    //scan->max[0] = scan->orientation[index_r+(int)(k/2)];
    robot->goal_distance= optimal_r[index_r]-1;
    robot->goal_orientation = scan->orientation[index_r+(int)(k/2)];
  }
  else
  {
    robot->goal_distance=0;
    robot->goal_orientation = normalize(robot->goal_orientation - scan_angle*0.5);
    printf("so small : large_r: %.2lf, goal_orientation %.2lf\n",large_r,robot->goal_orientation);
  }
  printf("\nChoice parameters:Width %.2f*, Goal %.2lf*, Dist to travel %.2lfm, Average %.2lfm\n",k*scan->delta*180.0/PI,robot->goal_orientation*180.0/PI,robot->goal_distance,large_r);
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
  rot_speed_radius=speedW*robot_radius;  // rot_speed_radius is rot. speed * radius of the robot in [m/s], tangential speed
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
    printf("Wheel speed too high ! factor [%.2f] max [%.2f]\n", factor, max);
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
