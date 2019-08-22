#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <iomanip> //////////////////////////////////////////////////////////////////////////////
// Created by zhouzzz on 2018/6/18.
// Description : 服务器类
//////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include<WINSOCK2.H>
#include "Ws2tcpip.h"
#include "Winsock2.h"
#include"zss_cmd.pb.h"
#include"vision_detection.pb.h"
#include "zss_debug.pb.h"
#include <random>
#include <ctime>
#include <vector>
#include <cmath>
#include <algorithm> 
#include <cstdio> 
using std::cin;
using std::cout;
using std::cerr;
using std::endl;
double ROBO[30][2];
int NumPoint = 0;
double cost_path[500001];//500*500的地图共250000个点，留有冗余
double rrt_point[10000][2];
//int costs[500001];
#define ME 3
struct point
{
	int parent;
	double pos_x, pos_y;
};
int getIndex(double x, double y)
{
	return int(int(x - 1) * 600 + int(y));
}

char* getIP()
{
	BYTE minorVer = 2;
	BYTE majorVer = 2;
	WSADATA wsaData;
	WORD sockVersion = MAKEWORD(minorVer, majorVer);
	WSAStartup(sockVersion, &wsaData);
	int i;
	char szHost[256];
	gethostname(szHost, 256);
	hostent *pHost = gethostbyname(szHost);
	in_addr addr;

	char *p = pHost->h_addr_list[0];
	memcpy(&addr.S_un.S_addr, p, pHost->h_length);
	char *szIp = inet_ntoa(addr);
	return (szIp);
}

double measure_distance(point x, point y)
{
	return sqrt(pow(fabs(x.pos_x - y.pos_x), 2) + pow(fabs(x.pos_y - y.pos_y), 2));
}
int WALL(double x, double y)
{
	for (int i = 0; i <= 15; ++i)
		if((ROBO[i][0] != -1 || ROBO[i][1] != -1) && i!=ME)
			if (sqrt(pow(fabs(x - ROBO[i][0]), 2) + pow(fabs(y - ROBO[i][1]), 2)) < 36.0)
				return 1;
	return 0;
}
int argmin(std::vector<point>& seeds, point seed)
{
	int result = 0;
	double minimun = 1.0e10;
	for (int i = 0; i < seeds.size(); i++)
	{
		double dis = measure_distance(seeds[i], seed);
		if (dis < minimun)
		{
			result = i;
			minimun = dis;
		}
	}
	return result;
}
int detect(point a, point b)
{
	double deltx = (b.pos_x - a.pos_x) / 100;
	double delty = (b.pos_y - a.pos_y) / 100;
	for (double i = 1; i <= 100; i += 1.0) {
		if (WALL(a.pos_x + i * deltx, a.pos_y + i * delty) > 0) return 1;
	}
	return 0;
}
#pragma comment(lib, "ws2_32.lib")

int SOCKADDR_IN_SIZE = sizeof(SOCKADDR_IN);
const u_short DEFAULT_PORT = 23333;
const size_t MSG_BUF_SIZE = 2048;
const size_t IP_BUF_SIZE = 256;
//const int Port = 50001;

int RRTstar(double start_x, double start_y, double end_x, double end_y)
{
	/*for (int i = 1; i <= 500000; ++i) costs[i] = 0;
	for (int i = 100; i <= 150; ++i)
		for (int j = 0; j <= 300; ++j)
			costs[getIndex(i, j)] = 253;
	for (int i = 250; i <= 300; ++i)
		for (int j = 250; j <= 500; ++j)
			costs[getIndex(i, j)] = 253;*/
			//double start_x = ROBO[0][0], start_y = ROBO[0][1], end_x = 0, end_y = 550;
	int xs_ = 450, ys_ = 600;
	int ns_ = xs_ * ys_;
	int lethal_cost_ = 253; //barrier height 
	double step = 4.0; //调节每步最小步长 
	std::vector<point> seeds; //随机生成点的集合 
	point end_point; //point struct记录其xy坐标与父结点在seeds中的下标 
	end_point.pos_x = end_x;
	end_point.pos_y = end_y;
	point current;
	current.pos_x = start_x;
	current.pos_y = start_y;
	current.parent = -1; //出发点的父结点为源点-1 
	//cout << "SXSY EXEY ";
	//cout << current.pos_x << " " << current.pos_y << " " << end_point.pos_x << " " << end_point.pos_y;    // set x and set y  

	int Xmin;
	seeds.push_back(current); //加入vector，进行广搜  
	int cycles = 0;
	//for (int i = 0; i < 500000; ++i) cost_path[i] = 5000;
	cost_path[getIndex(current.pos_x, current.pos_y)] = 0;//除起点外距离均为max，max为防止累加导致溢出不宜过大，如0x3f3f3f3，本地图5000足够大。 
	//std::default_random_engine generator(time(NULL));//依系统时间为种子生成随机数 

	clock_t st; st = clock();
	int Able = 0;
	while (cycles < 5000)
	{
		cycles++;
		//std::uniform_int_distribution<int> random_gen_y(0, 600);
		//std::uniform_int_distribution<int> random_gen_x(0, 450);
		int ran_x = rand() % 450;
		int ran_y = rand() % 600;

		current.pos_x = ran_x;
		current.pos_y = ran_y; //X.current<-X.rand
		int nearest = argmin(seeds, current); //get X.nearest
		double minimum = measure_distance(seeds[nearest], current);
		if (measure_distance(seeds[nearest], current) < step)
		{
			cycles--; //离X.nearest距离不足最小步长，则重新随机取点
			continue;
		}
		else {
			double a = 1.0;//前往随机采样点方向的增长系数
			double b = 2.0;//前往终点方向的增长系数
			double distance_x_end = end_point.pos_x - seeds[nearest].pos_x;
			double distance_y_end = end_point.pos_y - seeds[nearest].pos_y;
			double delta_x = current.pos_x - seeds[nearest].pos_x;
			double delta_y = current.pos_y - seeds[nearest].pos_y;

			double rate = sqrt(pow(step, 2) / (pow(fabs(delta_x), 2) + pow(fabs(delta_y), 2)));
			double rate_end = sqrt(pow(step, 2) / (pow(fabs(distance_x_end), 2) + pow(fabs(distance_y_end), 2)));
			//以线性的步长/直线距离为扩展率，离得越近“拉力”越大
			double forward_x = seeds[nearest].pos_x + delta_x * rate * a + distance_x_end * rate_end * b;
			double forward_y = seeds[nearest].pos_y + delta_y * rate * a + distance_y_end * rate_end * b;
			//依系数加权合成X.new位置
			if (WALL(forward_x, forward_y) > 0)
			{
				//发现目标点撞墙
				//cout << "shit" << forward_x << " " << forward_y << endl;
				double forward_x_again = seeds[nearest].pos_x + delta_x * rate * a;
				double forward_y_again = seeds[nearest].pos_y + delta_y * rate * a; //去掉目标点dx、dy对选点的影响
				if (WALL(forward_x_again, forward_y_again) > 0) //还是撞墙，则“弃疗”，否则保留
					continue;
				else
				{
					forward_x = forward_x_again;
					forward_y = forward_y_again;
				}
			}

			//point Now;
			//Now.pos_x = forward_x;
			//Now.pos_y = forward_y;
			//Now.parent = nearest;
			//cout << "NOW :   " << Now.pos_x << "   " << Now.pos_y <<" ? "<< getIndex(Now.pos_x, Now.pos_y)<<" "<<WALL(Now.pos_x,Now.pos_y)<<endl;

			//int nearest = argmin(seeds, Now);
			//double minimum = measure_distance(seeds[nearest], Now);
			//cost_path[getIndex(forward_x, forward_y)] = cost_path[getIndex(seeds[nearest].pos_x, seeds[nearest].pos_y)] + measure_distance(seeds[nearest], Now);//Cost[Nearest]+Now与Nearest间的距离，即为到搜索树根结点距离
			//Xmin = nearest;//初始时暂认为X.nearest引过来的路径最短

						 
			//Now.parent = Xmin; //将X.Now以r内的最短路连到搜索树上
			//seeds.push_back(Now);


			current.pos_x = forward_x;
			current.pos_y = forward_y;
			current.parent = nearest; //更新X.current为X.Now（已在vector的最后一位）
			seeds.push_back(current);
			if (measure_distance(end_point, current) < step)
			{
				Able = 1;
				break;//到达终点附近，结束
			}
		}
	}
	end_point.parent = seeds.size() - 1;//vector的最后一位即current的下标
	seeds.push_back(end_point);

	cout << "Time: " << (double)clock() - st << endl;
	//std::pair<float, float> now;
	NumPoint = 0;
	rrt_point[++NumPoint][0] = end_point.pos_x;
	rrt_point[NumPoint][1] = end_point.pos_y;
	cout << end_point.pos_x << " " << end_point.pos_y << endl;
	while (current.parent != -1)
	{
		//now.first = current.pos_x;
		//now.second = current.pos_y;
		int lzmax = 20;
		if (seeds[current.parent].parent != -1)
			while (detect(seeds[seeds[current.parent].parent], current) == 0)
			{
				lzmax--;
				if (lzmax == 0) break;
				//cout << "lazy" << endl;
				current.parent = seeds[current.parent].parent;
				if (seeds[current.parent].parent == -1) break;
			}
		rrt_point[++NumPoint][0] = current.pos_x;
		rrt_point[NumPoint][1] = current.pos_y;
		cout << current.pos_x << " " << current.pos_y << endl;
		//path.push_back(now); //path记录路径上的结点并输出
		current = seeds[current.parent]; //往父亲结点迭代
	}
	rrt_point[++NumPoint][0] = start_x;
	rrt_point[NumPoint][1] = start_y;
	return Able;
}
Debug_Msgs draw_msgs;
char msg[16384];
char bufferf[1024];
bool has_arrive(double x, double y, double x1, double y1, double dis_lev) {

	if (sqrt(pow(x - x1, 2) + pow(y - y1, 2)) < dis_lev)
		return 1;
	else
		return 0;
}
double compute_heading(double x, double y, double head_x, double head_y, double theta) {
	double bias = 0.0;
	if ((head_x - x) < 0.0) bias = 3.14159265;
	//printf(" jiajiao: %.4lf\n", atan((head_y - y) / (head_x - x))+bias);
	return atan((head_y - y) / (head_x - x)) + bias - theta;
}
double compute_velocity(double distance, double X, double Y, int id_num, int flag) {
	double VELOCITY;
	if (distance < 100) {
		double dx = 76.0;
		double dy = 50.0;
		double k = dy / dx;
		VELOCITY = 150 + (distance - 100) * k;
	}//减速程序
	else
	{
		//double dx = 76.0;
		//double dy = 200.0;
		double k = 6;
		distance = sqrt(pow(rrt_point[id_num - (flag * 2 - 1)][1] - 300 - X, 2) + pow(rrt_point[id_num - (flag * 2 - 1)][0] - 225 - Y, 2));
		VELOCITY = min(30 + distance * k, 150);

	}
	return VELOCITY;
}

void SendDebug()
{
	Debug_Msg* draw_msg;
	for (int i = 1; i < NumPoint; ++i)
	{
		draw_msg = draw_msgs.add_msgs();
		draw_msg->set_type(Debug_Msg_Debug_Type::Debug_Msg_Debug_Type_LINE);
		draw_msg->set_color(Debug_Msg_Color::Debug_Msg_Color_WHITE);
		Debug_Line* line = draw_msg->mutable_line();
		Point* start = line->mutable_start();
		Point* end = line->mutable_end();
		//start = new Point();
		//end = new Point();
		//line = new Debug_Line();
		start->set_x(rrt_point[i][1] - 300);
		start->set_y(-rrt_point[i][0] + 225);
		end->set_x(rrt_point[i + 1][1] - 300);
		end->set_y(-rrt_point[i + 1][0] + 225);
		//cout << rrt_point[i][1]-300 << " " << rrt_point[i][0]-225 << endl;
		//cout << rrt_point[i+1][1] - 300 << " " << rrt_point[i+1][0] - 225 << endl;
		/*line->set_allocated_start(start);
		line->set_allocated_end(end);*/
		line->set_forward(1);
		line->set_back(1);
		//draw_msg->set_allocated_line(line);
	}
	draw_msgs.SerializeToArray(msg, 16384);
	draw_msgs.Clear();
}
void SendCmd(double v_x, double v_y)
{
	Robots_Command rcs;
	Robot_Command* rc;
	rc = rcs.add_command();
	int ret_val = 0;
	rc->set_robot_id(ME);
	rc->set_velocity_r(0);
	rc->set_velocity_x(v_x);
	rc->set_velocity_y(v_y);
	rc->set_kick(false);
	rc->set_power(100);
	rc->set_dribbler_spin(1);
	//	int sizef = rcs.command_size();
	int buffer_sizef = 1024;
	rcs.SerializeToArray(bufferf, buffer_sizef);
	//rcs.cl
	rcs.Clear();
}
int main() {
	srand(time(NULL));
	//SendDebug();
	WSADATA wsa_data;
	SOCKET sock_serv = INVALID_SOCKET;
	SOCKADDR_IN addr_serv, addr_clt;
	char ip_buf[IP_BUF_SIZE];
	char msg_buf[MSG_BUF_SIZE];


	WSADATA wsaData;//初始化
	sockaddr_in RecvAddrf1;//服务器地址
	sockaddr_in RecvAddrf2;
	int Port1 = 50001;//服务器监听地址
	int Port2 = 20001;
	char SendBuff[1024 * 10];//发送数据的缓冲区
	int BufLenf = 1024 * 10;//缓冲区大小

	double barrier_x[16];
	double barrier_y[16];
	double barrier_angle[16];
	double distance[16];
	double barrier_V[16];

	int ret_val = WSAStartup(MAKEWORD(2, 2), &wsa_data);
	if (ret_val != 0) {
		cerr << "WSAStartup() function failed with error: " << WSAGetLastError() << "\n";
		return 1;
		system("pause");
	}
	SOCKET SendSocketf;
	SendSocketf = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	//设置服务器地址
	RecvAddrf2.sin_family = AF_INET;
	RecvAddrf2.sin_port = htons(Port2);
	char *ipnow = getIP();
	cout << ipnow << endl;
	RecvAddrf2.sin_addr.s_addr = inet_addr(ipnow);
	RecvAddrf1.sin_family = AF_INET;
	RecvAddrf1.sin_port = htons(Port1);
	RecvAddrf1.sin_addr.s_addr = inet_addr(ipnow);
	//向服务器发送数据报
	printf("Sending a datagram to the receiver...\n");


	//cout << sendto(SendSocketf, msg, BufLenf, 0, (SOCKADDR*)& RecvAddrf2, sizeof(RecvAddrf2)) << endl;
	//
	//SecureZeroMemory(&addr_serv, SOCKADDR_IN_SIZE);
	addr_serv.sin_family = AF_INET;
	addr_serv.sin_port = htons(DEFAULT_PORT);
	addr_serv.sin_addr.S_un.S_addr = ADDR_ANY;
	//
	sock_serv = socket(addr_serv.sin_family, SOCK_DGRAM, IPPROTO_UDP);
	if (sock_serv == INVALID_SOCKET) {
		cerr << "socket() function failed with error: " << WSAGetLastError() << "\n";
		WSACleanup();
		system("pause");
		return 1;
	}
	//
	ret_val = bind(sock_serv, (SOCKADDR*)& addr_serv, SOCKADDR_IN_SIZE);
	if (ret_val != 0) {
		cerr << "bind() function failed with error: " << WSAGetLastError() << "\n";
		system("pause");
		return 1;
	}
	cout << "A UDP server has started successfully..." << endl;
	//
	int ME1 = ME;
	while (true) {
		SecureZeroMemory(msg_buf, MSG_BUF_SIZE);
		ret_val = recvfrom(sock_serv, msg_buf, MSG_BUF_SIZE, 0, (SOCKADDR*)& addr_clt, &SOCKADDR_IN_SIZE);
		if (ret_val > 0)
		{
			inet_ntop(addr_clt.sin_family, &addr_clt, ip_buf, IP_BUF_SIZE);
			cout << "message from client " << ip_buf << ": " << msg_buf << endl;
			Vision_DetectionFrame frame;
			Vision_DetectionBall  balls;

			Vision_DetectionRobot robots_yellow;
			frame.ParseFromArray(msg_buf, MSG_BUF_SIZE);// 获取required数据
			for (int i = 0; i <= 15; ++i) ROBO[i][0] = ROBO[i][1] = -1;
			for (int ii = 0; ii < frame.robots_blue_size(); ++ii)
			{
				int i = frame.robots_blue(ii).robot_id();
				cout << "RBid=   "<<i<<endl;
				if (i == ME) {
					ME1 = ii; cout << "ME1 = " << ME1 << endl;
				}
				Vision_DetectionRobot robots_blue = frame.robots_blue(ii);
				ROBO[i][1] = robots_blue.x() / 10.0 + 300;
				ROBO[i][0] = 450 - (robots_blue.y() / 10.0 + 225);
				cout << ROBO[i][0] << " " << ROBO[i][1] << endl;
				//cout << /*robots_blue.raw_vel_x() << robots_blue.raw_vel_y() << */robots_blue.orientation() << endl;
			}
			for (int ii = 0; ii < frame.robots_yellow_size(); ++ii)
			{
				int i = frame.robots_yellow(ii).robot_id();
				//cout << "RBid=   " << i << endl;
				Vision_DetectionRobot robots_yellow = frame.robots_yellow(ii);
				ROBO[i + 8][1] = robots_yellow.x() / 10.0 + 300;
				ROBO[i + 8][0] = 450 - (robots_yellow.y() / 10.0 + 225);
				//cout << ROBO[i][0] << " " << ROBO[i][1] << endl;
			}
			double ROBO00 = ROBO[ME][0], ROBO01 = ROBO[ME][1], des_x = 40, des_y = 500;
			int flagRRT = RRTstar(ROBO00, ROBO01, des_x, des_y);
			cout << flagRRT << " is RRT working status" << endl;
			SendDebug();
			cout << sendto(SendSocketf, msg, BufLenf, 0, (SOCKADDR*)& RecvAddrf2, sizeof(RecvAddrf2)) << endl;

			int id_num = NumPoint - 1, flag = 0;
			double VELOCITY = 200;
			//bool is_accerate = 0;
			int cnt = 0;
			while (1) {
				//VELOCITY = 300;
				SecureZeroMemory(msg_buf, MSG_BUF_SIZE);
				ret_val = recvfrom(sock_serv, msg_buf, MSG_BUF_SIZE, 0, (SOCKADDR*)& addr_clt, &SOCKADDR_IN_SIZE);
				frame.ParseFromArray(msg_buf, MSG_BUF_SIZE);// 获取required数据
				Vision_DetectionRobot my_robot = frame.robots_blue(ME1);
				//double X = my_robot.x() / 10;
				//double Y = -my_robot.y() / 10;
				double theta = - my_robot.orientation();
				double now_v_x = my_robot.vel_x();
				double now_v_y = my_robot.vel_y();
				for (int i = 0; i <= 15; ++i) barrier_x[i] = barrier_y[i] = -1;
				
				for (int ii = 0; ii < frame.robots_blue_size(); ++ii)
				{
					int i = frame.robots_blue(ii).robot_id();
					//cout << "RBid=   " << i << endl;
					barrier_x[i] = frame.robots_blue(ii).x() / 10;
					barrier_y[i] = -frame.robots_blue(ii).y() / 10;
					//cout << barrier_x[i] << "  " << barrier_y[i] << endl;
				}
				double X = barrier_x[ME], Y = barrier_y[ME];
				//cout << "SIZE: " << frame.robots_yellow_size() << endl;
				for (int ii = 0; ii < frame.robots_yellow_size(); ++ii)
				{
					int i = frame.robots_yellow(ii).robot_id();
					//cout << "RBid=   " << i << endl;
					barrier_x[i + 8] = frame.robots_yellow(ii).x() / 10;
					barrier_y[i + 8] = -frame.robots_yellow(ii).y() / 10;
					//cout << barrier_x[i+8] << "  " << barrier_y[i+8] << endl;
				}

				double sum_v_x = 0;
				double sum_v_y = 0;
				for (int i = 0; i <= 15; i++) 
				{
					if ((barrier_x[i]!=-1 || barrier_y[i] != -1) && i != ME)
					{
						barrier_angle[i] = compute_heading(X, Y, barrier_x[i], barrier_y[i], theta);
						distance[i] = sqrt(pow(X - barrier_x[i], 2) + pow(Y - barrier_y[i], 2));
						barrier_V[i] = -5000 / pow(distance[i] - 22, 2) - 5000 / pow(distance[i] - 21, 2) - 5000 / pow(distance[i] - 20, 2) - 5000 / pow(distance[i] - 19, 2);
						sum_v_x += barrier_V[i] * cos(barrier_angle[i]);
						sum_v_y += barrier_V[i] * sin(barrier_angle[i]);
					}
				}


				//cout << rrt_point[id_num][1] - 300 << " " << rrt_point[id_num][0] - 225 << " " << X << " " << Y << endl;
				//cout << "ORt  " << theta << endl;
				double dis_lev = 0;
				if (id_num == 1 || id_num >= NumPoint) dis_lev = 5.0; else dis_lev = 15.0;
				if (has_arrive(rrt_point[id_num][1] - 300, rrt_point[id_num][0] - 225, X, Y, dis_lev))
				{

					if (!flag)
						id_num--;
					else id_num++;
					if (id_num == 0 && !flag)
					{
						id_num = 1;
						flag = 1;
					}
					if (id_num > NumPoint && flag)
					{
						id_num--;
						flag = 0;
					}
				}
				while (1) 
				{
					int Flag = 0;
					if (sqrt(pow(rrt_point[id_num][1] - 300 - X, 2) + pow(rrt_point[id_num][0] - 225 - Y, 2)) < 60) {
						for (int i = 0; i <= 15; i++) 
						if((barrier_x[i] != -1 || barrier_y[i] != -1) && i!=ME)
						{
							if (sqrt(pow(rrt_point[id_num][1] - 300 - barrier_x[i], 2) + pow(rrt_point[id_num][0] - 225 - barrier_y[i], 2)) < 30)
								Flag = 1;
						}
					}
					//if ((id_num == 1 || id_num == NumPoint) && Flag == 1)id_num = id_num - flag * 2 + 1;
					if (Flag == 1) id_num = id_num + flag * 2 - 1;
					if (Flag == 0) break;
				}

				//double now_velocity = sqrt(pow(now_v_x, 2)+ pow(now_v_y, 2));



				double distance = sqrt(pow(rrt_point[id_num][1] - 300 - X, 2) + pow(rrt_point[id_num][0] - 225 - Y, 2));
				//double distance1= sqrt(pow(rrt_point[id_num][1] - 300 - X, 2) + pow(rrt_point[id_num][0] - 225 - Y, 2));

				VELOCITY = compute_velocity(distance, X, Y, id_num, flag);
				double heading = compute_heading(X, Y, rrt_point[id_num][1] - 300, rrt_point[id_num][0] - 225, theta);
				double v_x = VELOCITY * cos(heading) + sum_v_x;
				double v_y = VELOCITY * sin(heading) + sum_v_y;
				if (v_x <= 10 && v_y <= 10) {
					v_x = v_x + rand() % 60 - 30;
					v_y = v_y + rand() % 60 - 30;
				}
				SendCmd(v_x, v_y);
				//cout << v_x << " " << v_y << " " << heading << endl;
				sendto(SendSocketf, bufferf, BufLenf, 0, (SOCKADDR*)& RecvAddrf1, sizeof(RecvAddrf1));
				//Sleep(3);
				/*++cnt;
				if (sqrt(v_x * v_x + v_y * v_y) < 10.0 && cnt>300)
				{
					cnt = 0;
					int flagRRT=0;
					while (!flagRRT)
					{
						if (flag) flagRRT = RRTstar(barrier_y[0] + 225, barrier_x[0] + 300, ROBO00, ROBO01);
						else flagRRT = RRTstar(barrier_y[0] + 225, barrier_x[0] + 300, des_x, des_y);
						printf("%d\n is RRT woring status", flagRRT);
					}
					SendDebug();
					cout << sendto(SendSocketf, msg, BufLenf, 0, (SOCKADDR*)& RecvAddrf2, sizeof(RecvAddrf2)) << endl;
				}*/
			}

			Sleep(5000000);
		}
		else if (ret_val == 0) {
			cout << "connection is closed..." << endl;
		}
		else {
			cerr << "recvfrom() function failed with error: " << WSAGetLastError() << "\n";
			closesocket(sock_serv);
			WSACleanup();
			system("pause");
			return 1;
		}
	}
	ret_val = shutdown(sock_serv, SD_BOTH);
	if (ret_val == SOCKET_ERROR) {
		cerr << "shutdown() function failed with error: " << WSAGetLastError() << "\n";
		closesocket(sock_serv);
		WSACleanup();
		system("pause");
		return 1;
	}
	closesocket(sock_serv);
	WSACleanup();
	cout << "server shutdown..." << endl;
	system("pause");
	return 0;
}

