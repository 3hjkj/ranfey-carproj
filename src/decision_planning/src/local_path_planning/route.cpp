#include"route.h"

Local_route::Local_route( )
{
}
Local_route::~Local_route()
{
}
void Local_route::init_pub( ros::NodeHandle n)
{
	pub_global_center= n.advertise<visualization_msgs::Marker>("/global_path_center", 10);
	pub_global_left= n.advertise<visualization_msgs::Marker>("/global_path_left", 10);
	pub_global_right= n.advertise<visualization_msgs::Marker>("/global_path_right", 10);
	pub_local_center= n.advertise<visualization_msgs::Marker>("/loacl_path_center", 10);
	pub_local_left= n.advertise<visualization_msgs::Marker>("/loacl_path_left", 10);
	pub_local_right= n.advertise<visualization_msgs::Marker>("/loacl_path_right", 10);
}
void Local_route:: ReadTxt(string trace_path,vector<vector<double>>&paths)
{
	vector<double> path;
    // 只适用于逗号分分隔
    string line;
    std::ifstream input;
    input.open(trace_path);
	int num_lines=0;
    while (getline(input, line))
    {   num_lines++;
		split(line, ",",path);
		paths.push_back(path);
		path.clear();
    }
    input.close();
}
void Local_route::split(string str, string pattern,vector<double>& result)
{
    string::size_type pos;
    str += pattern;//扩展字符串以方便操作
    int size = str.size();
    for (int i = 0; i < size; i++)
    {
        pos = str.find(pattern, i);
        if (pos < size )
        {
			result.push_back(std::atof(str.substr(i, pos - i).c_str()));
            i = pos + pattern.size() - 1;
        }
    }
}
void Local_route::visiual_global_trace(vector<vector<double>>paths)
{
	vector<visualization_msgs::Marker> sloacl_path;
	for(int ii=0;ii<paths.size()/3;ii++)
	{
	visualization_msgs::Marker sloacl_path_;
	geometry_msgs::Point wp;
	sloacl_path_.header.frame_id = "/world";
	sloacl_path_.header.stamp = ros::Time();
	sloacl_path_.ns = "";
	sloacl_path_.action = visualization_msgs::Marker::ADD;
	sloacl_path_.scale.x = 0.1;
	sloacl_path_.frame_locked = false;
	for(int i=0;i<paths[ii].size();i++)
	{
		wp.x=paths[ii*3][i];
		wp.y=paths[ii*3+1][i];
		wp.z=0;
		sloacl_path_.points.push_back(wp);
	}
	sloacl_path_.type = visualization_msgs::Marker::LINE_STRIP;
	sloacl_path_.color.b = 1;
	sloacl_path_.color.g = 1;
	sloacl_path_.color.r = 1;
	sloacl_path_.color.a = 1;
	sloacl_path.push_back(sloacl_path_);
	}
	pub_global_center.publish(sloacl_path[1]);
	pub_global_left.publish(sloacl_path[0]);
	//pub_global_right.publish(sloacl_path[2]);
}
void Local_route::calculate_parameter()
{
	double point_distance=0.3;
	int chang_lane_point=10;
	double lane_distance=2;
	double chang_lane_distance;
	double weight_data=0.45;
	double weight_smooth=0.4;
	double tolerance=0.05;
}
void Local_route::route_plan(vector<vector<double>>paths,int trace_id,int local_point_id)
{
	// vector<visualization_msgs::Marker> sloacl_path;
	// visualization_msgs::Marker sloacl_path_;
    // geometry_msgs::Point wp;
	vector<vector<double>> local_path_1;
	// for(int i=0;i<paths.size()/3;i++)
	// {
	// 	sloacl_path_.header.frame_id = "/world";
	// 	sloacl_path_.header.stamp = ros::Time();
	// 	sloacl_path_.ns = "";
	// 	sloacl_path_.action = visualization_msgs::Marker::ADD;
	// 	sloacl_path_.frame_locked = false;
	// 	sloacl_path_.scale.x = 0.3;
	// 	sloacl_path_.frame_locked = false;
	// 	sloacl_path.push_back(sloacl_path_);
	// }
	//ros::Rate loop_rate(20);
	// while(ros::ok())
	// {
		if(local_path_size+local_point_id<paths[0].size())
		{
		vector<double> local_x;
		vector<double> local_y;
		vector<double> local_yaw;
		for( int idd=0;idd<paths.size()/3;idd++)
		{
			if(idd==trace_id)
			{
				if(local_point_id+local_path_size<paths[trace_id*3].size())
				{
					local_x.assign(paths[trace_id*3].begin()+local_point_id,paths[trace_id*3].begin()+local_point_id+local_path_size);
					local_y.assign(paths[trace_id*3+1].begin()+local_point_id,paths[trace_id*3+1].begin()+local_point_id+local_path_size);
					local_yaw.assign(paths[2].begin()+local_point_id,paths[2].begin()+local_point_id+local_path_size);
					local_path_1.push_back(local_x);
					local_path_1.push_back(local_y);
					local_path_1.push_back(local_yaw);
					local_x.clear();
					local_y.clear();
					local_yaw.clear();
				}
			}
			else if(abs(idd-trace_id)==1)
			{       
				if(generator_local_trace(paths[trace_id*3],paths[trace_id*3+1],paths[2],
				paths[idd*3],paths[idd*3+1],local_x,local_y,local_yaw, local_point_id)){
					local_path_1.push_back(local_x);
					local_path_1.push_back(local_y);
					local_path_1.push_back(local_yaw);
					local_x.clear();
					local_y.clear();
					local_yaw.clear();
				}
			}
		}
		vector<vector<double>> local_path_;
		for(int ix=0;ix<int(local_path_1.size()/3);ix++)
		{
			local_path_.push_back(local_path_1[ix*3]);
			local_path_.push_back(local_path_1[ix*3+1]);
			smoothPath(local_path_,weight_data,weight_smooth,tolerance);
			// for(int i=0;i<local_path_[0].size();i++)
			// {
			// 	wp.x=local_path_[0][i];
			// 	wp.y=local_path_[1][i];
			// 	wp.z=0;
			// 	sloacl_path[ix].points.push_back(wp);
			// }
			local_path_1[ix*3]=local_path_[0];
			local_path_1[ix*3+1]=local_path_[1];
			local_path_.clear();
		}
		local_path.clear();
		local_path=local_path_1;
		local_path_1.clear();
		// for(int ii=0;ii<3;ii++)
		// {
		// 	if(ii==trace_id)
		// 	{
		// 		sloacl_path[ii].type = visualization_msgs::Marker::LINE_STRIP;
		// 		sloacl_path[ii].color.b = 0;
		// 		sloacl_path[ii].color.g = 0;
		// 		sloacl_path[ii].color.r = 1;
		// 		sloacl_path[ii].color.a = 1;
		// 	}
		// 	else 
		// 	{
		// 		sloacl_path[ii].type = visualization_msgs::Marker::LINE_STRIP;
		// 		sloacl_path[ii].color.b = 0;
		// 		sloacl_path[ii].color.g = 1;
		// 		sloacl_path[ii].color.r = 0;
		// 		sloacl_path[ii].color.a = 1;
		// 	}
		// }
		// cout<<__LINE__<<":"<<sloacl_path[1].points.size()<<endl;
		// pub_local_center.publish(sloacl_path[1]);
		// pub_local_left.publish(sloacl_path[0]);
		//pub_local_right.publish(sloacl_path[2]);
		// sloacl_path[0].points.clear();
		// sloacl_path[1].points.clear();
		//sloacl_path[2].points.clear();
		}
		//loop_rate.sleep();
}
double  Local_route::azimuthAngle( double  x1,double  y1,double  x2,double  y2)
{
    double angle = 0.0;
    double dx = x2 - x1;
    double dy = y2 - y1;
    if(x2 == x1)
    {
        angle = M_PI_2 ;
        if( y2 == y1 )
            angle = 0.0;
        else// (y2 < y1 )
            angle = 3.0 *M_PI_2 ;
    }
    else if (x2 > x1 && y2 > y1)  //一象限
        angle = atan(dy / dx);
    else if (x2 > x1 && y2 < y1)  //四象限
        angle = 2*M_PI + atan(dy / dx);
    else if (x2 < x1 && y2 < y1 )  //三象限
        angle = M_PI+ atan(dy / dx);
    else if(x2 < x1 && y2 > y1 )  //二象限
        angle = M_PI + atan(dy / dx);
    return angle;
}
void Local_route::smoothPath(std::vector<vector<double>>& paths_dect, double weight_data,double weight_smooth, double tolerance)
{
	for(int ix=0;ix<int(paths_dect.size()/2);ix++)
	{
		std::vector<vector<double>> path_in;
		path_in.assign(paths_dect.begin()+(ix*2),paths_dect.begin()+(ix*2)+2);
		std::vector<vector<double>> smoothPath_out = path_in;
		double change = tolerance;
		double xtemp, ytemp;
		int nIterations = 0;
		int size = paths_dect[ix].size();
		while (change >= tolerance) {
			change = 0.0;
			for (int i = 1; i < size - 1; i++) {
				xtemp = smoothPath_out[0][i];
				ytemp = smoothPath_out[1][i];
				smoothPath_out[0][i] += weight_data * (path_in[0][i] - smoothPath_out[0][i]);
				smoothPath_out[1][i] += weight_data * (path_in[1][i]  - smoothPath_out[1][i]);
				smoothPath_out[0][i] += weight_smooth * (smoothPath_out[0][i - 1] + smoothPath_out[0][i + 1] - (2.0 * smoothPath_out[0][i]));
				smoothPath_out[1][i] += weight_smooth * (smoothPath_out[1][i - 1] + smoothPath_out[1][i + 1] - (2.0 * smoothPath_out[1][i]));
				change += fabs(xtemp - smoothPath_out[0][i]);
				change += fabs(ytemp - smoothPath_out[1][i]);
			}
			nIterations++;
		}
		paths_dect[ix] = smoothPath_out[0];
		paths_dect[ix+1] = smoothPath_out[0+1];
	}
}
bool Local_route::generator_local_trace(vector<double>x_orignal,vector<double>y_orignal,vector<double>yaw_orignal,vector<double>x_target,vector<double>y_target,
vector<double>&local_x,vector<double>&local_y,vector<double> &local_yaw,int local_point_id)
{
    //double yaw_change=azimuthAngle(x_orignal[local_point_id+keep_point],y_orignal[local_point_id+keep_point],
	//x_target[local_point_id+keep_point+chang_lane_point],y_target[local_point_id+keep_point+chang_lane_point]);
    double dx=x_target[local_point_id+keep_point+chang_lane_point]-x_orignal[local_point_id+keep_point];
	double dy=y_target[local_point_id+keep_point+chang_lane_point]-y_orignal[local_point_id+keep_point];
	Eigen::Matrix3d axis_rotation;
	axis_rotation = Eigen::AngleAxisd(yaw_orignal[local_point_id+keep_point], Eigen::Vector3d::UnitZ()) *
	Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) * 
	Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
	Eigen::Vector3d local_axis_loc(dx, dy, 0);
    auto local_axis_point=axis_rotation.inverse()*local_axis_loc;
    double local_angle=abs(atan((local_axis_point[0])/(local_axis_point[1])));
	double xxx=local_axis_point[0];
	double yyy=local_axis_point[1];
    double x_average=local_axis_point[0]/(chang_lane_point+1);
	if (local_angle>M_PI_2)
        local_angle=M_PI-local_angle;
    else if (local_angle<-M_PI_2)
        local_angle=M_PI+local_angle;
    double change_rate_angle=(M_PI_2-local_angle)/(chang_lane_point+1)*2;

    if(x_target[local_point_id+keep_point+chang_lane_point]-x_orignal[local_point_id+keep_point]==0)
	{
		return false;
	}

	if(local_point_id+keep_point+chang_lane_point<x_orignal.size())
    {
		local_x.assign(x_orignal.begin()+local_point_id,x_orignal.begin()+local_point_id+keep_point+1);
		local_y.assign(y_orignal.begin()+local_point_id,y_orignal.begin()+local_point_id+keep_point+1);
		local_yaw.assign(yaw_orignal.begin()+local_point_id,yaw_orignal.begin()+local_point_id+keep_point);

		for (unsigned int i = 0; i < int(chang_lane_point/2); i++)
		{
			double local_xx=(i+1)*abs(x_average);
			double local_yy=(local_axis_point[1]/abs(local_axis_point[1]))*(i+1)*abs(x_average) * tan(change_rate_angle*(i+1));
			local_x.push_back(local_xx*cos(yaw_orignal[local_point_id+keep_point])-local_yy*sin(yaw_orignal[local_point_id+keep_point])+x_orignal[local_point_id+keep_point]);
			local_y.push_back(local_xx*sin(yaw_orignal[local_point_id+keep_point])+local_yy*cos(yaw_orignal[local_point_id+keep_point])+y_orignal[local_point_id+keep_point]);
			local_yaw.push_back(change_rate_angle*(i+1));
		}
		local_x.push_back(local_axis_point[0]/2*cos(yaw_orignal[local_point_id+keep_point])-local_axis_point[1]/2*sin(yaw_orignal[local_point_id+keep_point])+x_orignal[local_point_id+keep_point] );
		local_y.push_back(local_axis_point[0]/2*sin(yaw_orignal[local_point_id+keep_point])+local_axis_point[1]/2*cos(yaw_orignal[local_point_id+keep_point])+y_orignal[local_point_id+keep_point] );
        local_yaw.push_back(change_rate_angle*12);
		for (unsigned int i = 0; i < int(chang_lane_point/2); i++) {
			double local_xx_2=local_axis_point[0]-(int(chang_lane_point/2)-i)*abs(x_average);
			double local_yy_2=local_axis_point[1]-(local_axis_point[1]/abs(local_axis_point[1]))*(int(chang_lane_point/2)-i)*abs(x_average) * tan((change_rate_angle*(int(chang_lane_point/2)-i)));
			local_x.push_back(local_xx_2*cos(yaw_orignal[local_point_id+keep_point] )-local_yy_2*sin(yaw_orignal[local_point_id+keep_point] )+x_orignal[local_point_id+keep_point]);
			local_y.push_back(local_xx_2*sin(yaw_orignal[local_point_id+keep_point] )+local_yy_2*cos(yaw_orignal[local_point_id+keep_point] )+y_orignal[local_point_id+keep_point]);
			local_yaw.push_back(change_rate_angle*(int(chang_lane_point/2)-i));
		}
		if(keep_point+chang_lane_point<local_path_size)
		{
			local_x.insert(local_x.end(),x_target.begin()+local_point_id+keep_point+chang_lane_point,x_target.begin()+(local_path_size+local_point_id-1));
			local_y.insert(local_y.end(),y_target.begin()+local_point_id+keep_point+chang_lane_point,y_target.begin()+(local_path_size+local_point_id-1));
			local_yaw.insert(local_yaw.end(),yaw_orignal.begin()+local_point_id+keep_point+chang_lane_point,yaw_orignal.begin()+(local_path_size+local_point_id-1));
		}
		else
		{
			cout<<"keep_point + chang_lane_point 大于"<<local_path_size<<endl;
		}
		//cout<<"未平滑。。。。。。。。。。。。。。。。。。。。"<<endl;
		// for(int i=0;i<local_x.size();i++)
		// {
		// 	cout<<local_x[i]<<endl;
		// }
		// for(int i=0;i<local_x.size();i++)
		// {
		// 	cout<<local_y[i]<<endl;
		// }
		return 1;
    }
	else 
	{
		return 0;
	}
}
void Local_route::debug_show_road_cells(vector<can_control_msgs::msg::Autocontrol>& vec_at,lidar_msgs::msg::Cells cells_,
int V_RefPoint,double yaw_vel,double x_vel,double y_vel)
{
	// double yaw_vel;
	// tf::Quaternion quat;
	// yaw_vel=tf::getYaw(vhicle_pose.pose.orientation);
	//GPS航向角是与Y(正北)轴夹角，方向是顺时针Y-x，和车辆坐标系相反
	bool is_vaild_front;
	bool is_vaild_back;
	can_control_msgs::msg::Autocontrol at;
    double minx_cell_x=20;
	double minx_cell_x_=-20;
	cout<<"local_path :"<<local_path[0].size()<<endl;
		cout<<"local_path :"<<local_path[1].size()<<endl;
	// local_path_mtx.lock();
	for(int ix=0;ix<int(local_path.size()/3);ix++)
	{
		// if (trace_num_detection  > local_path[ix*3].size())
		// {
		// 	trace_num_detection = local_path[ix*3].size();
		// }
		is_vaild_front = false;
		is_vaild_back=false;
		for (int k = 0; k <  local_path[ix*3].size() ; k++)
		{
			geometry_msgs::Point wp;
			if(yaw_vel<0)
			{
				yaw_vel=yaw_vel+M_PI*2;
			}
			//float l2 = sqrt(pow(local_path[ix*3][k] - vhicle_pose.pose.position.x, 2) + pow(local_path[ix*3+1][k] - vhicle_pose.pose.position.y, 2));
			for (auto cell : cells_.cells)
			{
				double cell_angle_vhicle_axis=azimuthAngle(0,0,cell.x,cell.y);
				 Eigen::Matrix3d   own_utm_rotation;
				 own_utm_rotation = Eigen::AngleAxisd(yaw_vel, Eigen::Vector3d::UnitZ()) *
				 Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) * 
				Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
				//  Eigen::Matrix3d   cell_angle_vhicle_rotation_ = Eigen::AngleAxisd(cell_angle_vhicle_axis, Eigen::Vector3d::UnitZ()) *
				//  Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) * 
				//  Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
				Eigen::Vector3d cell_pos(cell.x, cell.y, 0);
				Eigen::Vector3d vhicle_pos(x_vel, y_vel, 0);
                auto cell_utm=own_utm_rotation*cell_pos+vhicle_pos;		
				double circle_r=sqrt(pow(cell_utm[0]-local_path[ix*3][k],2)+pow(cell_utm[1]-local_path[ix*3+1][k],2));
				// double l2_vhicle_axis=sqrt(cell.x*cell.x+cell.y*cell.y);
				// double  angle_vhicle2utm=cell_angle_vhicle_axis-yaw_vel;
				// double  cell_x_utm=l2_vhicle_axis*cos(angle_vhicle2utm)+vhicle_pose.pose.position.x;
				// double  cell_y_utm=l2_vhicle_axis*cos(angle_vhicle2utm)+vhicle_pose.pose.position.y;
				//double circle_r=sqrt((cell_x_utm-local_path[ix*3][k])*(cell_x_utm-local_path[ix*3][k])+(cell_y_utm-local_path[ix*3+1][k])*(cell_y_utm-local_path[ix*3+1][k]));
				if (circle_r>threshold_y)
				{
					continue;
				}
				else
				{
						//cout << "有障碍物 ~~~~~~~~~~~~ V_RefPoint: " << V_RefPoint << endl;
						// cout<<"ix:"<<ix<<endl;
						// cout<<"k:"<<k<<endl;
						// cout<<"V_RefPoint:"<<V_RefPoint<<endl;
						// cout<<"x_vel:"<<x_vel<<" y_vel:"<<y_vel<<endl;
						// cout<<"........."<<endl;
						// cout<<"cell.x"<<cell.x<<"      cell.y"<< cell.y<<endl;
						// cout<<"x:"<< local_path[ix*3][k]  <<"   y:"<< local_path[ix*3+1][k]  <<endl;
						// cout<<"circle_r"<<circle_r  << "   cell_x_utm:" <<cell_utm[0]  << "    cell_y_utm:" << cell_utm[1] << endl;
						// cout<< " yaw_vel heading:" << yaw_vel   << "    heading k:" << local_path[ix*3+2][k]   << std::endl;
						// //is_st.data = true;
						//cell.x 排列从小到大  车后------> 车前
						// if( cell.x<0){
						// 	is_vaild_back = true;
						// 	at.target_back_obj_distance = cell.x;
						// }
						// if(cell.x>0){
							is_vaild_front = true;
							at.RT1_L_LatObj = 0;
							at.RT1_L_LongObj = cell.x;
							at.front_l_lat_obj = 0;
							at.front_l_long_obj = cell.x;
							break;
						// }
				}
				// if(is_vaild==true)
				// 	break;
			}
		}
		if (is_vaild_front ==false)
		{
			at.RT1_L_LatObj = 0;
			at.RT1_L_LongObj = 0;
			at.front_l_lat_obj = 0;
			at.front_l_long_obj = 0;
		}
		// if (is_vaild_back ==false)
		// 	at.target_back_obj_distance=0;
		vec_at.push_back(at);
	}
	// local_path_mtx.unlock();
    cout << " cell.size1:" << cells_.cells.size() << endl;
    cout << " cell.size2:" << cells_.cells.size() << endl;
}