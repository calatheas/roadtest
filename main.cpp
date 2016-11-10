#include <stdio.h>
#include <conio.h>
#include <stdint.h>
#include <math.h>
#include <string>
#include <Windows.h>
#include <vector>

using namespace std;

#define SHARED_MOMORY_NAME1 "TORCS_SHARED1"
#define SHARED_MOMORY_NAME2 "TORCS_SHARED2"

#define CURVE_TYPE_RIGHT		1
#define CURVE_TYPE_LEFT			2
#define CURVE_TYPE_STRAIGHT	3

#define GEAR_FORWARD			0   // 전진 (D)
#define GEAR_BACKWARD	   -1	// 후진 (R)

#define INPUT_AICAR_SIZE			20
#define INPUT_FORWARD_TRACK_SIZE	20

#define MIN(x,y) ((x) < (y) ? (x) : (y))
#define MAX(x,y) ((x) > (y) ? (x) : (y))

#define MAX_CENTER_STEER 1.8
#define MIN_CENTER_STEER 1.1
#define CENTER_RANGE_1 0.18
#define CENTER_RANGE_2 0.25

// opponent state
#define OPP_IGNORE		0
#define OPP_FRONT		(1<<0)
#define OPP_BACK			(1<<1)
#define OPP_SIDE			(1<<2)
#define OPP_COLL			(1<<3)
#define OPP_LETPASS		(1<<4)
#define OPP_FRONT_FAST	(1<<5)

#define RCM_MAX_DT_ROBOTS	0.02

#define PI 3.14160
const double G = 9.80665;
const double MAX_UNSTUCK_ANGLE = 15.0 / 180.0 * PI;	// [radians] If the angle of the car on the track is smaller, we assume we are not stuck.
const double UNSTUCK_TIME_LIMIT = 0.5;				// [s] We try to get unstuck after this time.
const double MAX_UNSTUCK_SPEED = 5.0;				// [m/s] Below this speed we consider being stuck.
const double MIN_UNSTUCK_DIST = 0.0;				// [m] If we are closer to the middle we assume to be not stuck.
const float FULL_ACCEL_MARGIN = 1.0f;				// [m/s] Margin reduce oscillation of brake/acceleration.
const int MAX_UNSTUCK_COUNT = int(UNSTUCK_TIME_LIMIT / RCM_MAX_DT_ROBOTS);



struct shared_use_st
{
	// System Value
	BOOL connected;
	int written;

	// Driving Parameters
	double toMiddle;
	double angle;
	double speed;

	// Track Parameters
	double toStart;
	double dist_track;
	double track_width;
	double track_dist_straight;
	int    track_curve_type;
	double track_forward_angles[INPUT_FORWARD_TRACK_SIZE];
	double track_forward_dists[INPUT_FORWARD_TRACK_SIZE];
	double track_current_angle;

	// Other Cars Parameters
	double dist_cars[INPUT_AICAR_SIZE];

	// Racing Info. Parameters
	double damage;
	double damage_max;
	int    total_car_num;
	int    my_rank;
	int    opponent_rank;

	// Output Values
	double steerCmd;
	double accelCmd;
	double brakeCmd;
	int    backwardCmd;
};

class RoadTest
{
public:
	BOOL DEBUG_MODE = FALSE;

	shared_use_st *shared;
	unsigned int frameNum = 0;

	LARGE_INTEGER startTime, endTime, baseTime, frequency;
	double clientTime;

	int forward_opponents_n = 0;
	int backward_opponents_n = 0;


	int stuck = 0;

	BOOL startFlag = false; //스타팅 하는 순간 1프레임만 true;

	/******************
	추가정보 생성
	******************/
	double track_forward_dist2[INPUT_FORWARD_TRACK_SIZE];
	double track_forward_angle2[INPUT_FORWARD_TRACK_SIZE];
	double track_forward_radius[INPUT_FORWARD_TRACK_SIZE];
	double track_forward_point[INPUT_FORWARD_TRACK_SIZE][2];
	int track_forward_type[INPUT_FORWARD_TRACK_SIZE];
	double prevTrack_forward_dist[INPUT_FORWARD_TRACK_SIZE];
	double prevTrack_forward_angle[INPUT_FORWARD_TRACK_SIZE];
	double track_current_dist = 0.0;
	double current_point[2];
	int track_current_curve_type = CURVE_TYPE_STRAIGHT;
	double dist_to_hairpin_curve;
	double dist_to_not_hairpin_curve;
	bool in_hairpin_curve;
	double hairpin_angle = PI / 4;

	double nearestCar = 0.0;

	double steerLookaheadCoef1 = 15.0;
	double steerLookaheadCoef2 = 0.33;
	double steerOutlineCoef = 0.1;

	double trackUnitLength = 10.0;

	const double CAR_LENGTH = 4.39;
	const double CAR_WIDTH = 1.94;

	double FRICTION = 1.15;
	double TIRE_MU = 1.6;
	double MU_FACTOR = 0.69;
	double STEER_LOCK = 0.367;
	double MASS = 1150.0;
	double MAX_SPEED = 100.0;
	double MAX_SPEED_LIMIT = 100.0;
	double CA = 2.663; // BT에서 car7 / 일반도로 기준

	const double FRONTCOLLDIST = 70.0;				// [m] distance on the track to check other cars.
	const double BACKCOLLDIST = 50.0;				// [m] distance on the track to check other cars.
	const double LENGTH_MARGIN = 3.0;				// [m] savety margin.
	const double SIDE_MARGIN = 1.0;					// [m] savety margin.
	const double EXACT_DIST = 12.0;					// [m] if the estimated distance is smaller, compute it more accurate
	const double LAP_BACK_TIME_PENALTY = -30.0;	// [s]
	const double OVERLAP_WAIT_TIME = 5.0;			// [s] overlaptimer must reach this time before we let the opponent pass.
	const double SPEED_PASS_MARGIN = 5.0;			// [m/s] avoid overlapping opponents to stuck behind me.
	const double MAX_INC_FACTOR = 5.0;				// [m] Increment faster if speed is slow [1.0..10.0].
	const double WIDTHDIV = 3.0;						// [-] Defines the percentage of the track to use (2/WIDTHDIV).
	const double BORDER_OVERTAKE_MARGIN = 0.5;		// [m]
	const double OVERTAKE_OFFSET_SPEED = 5.0;				// [m/s] Offset change speed.
	const double CATCH_FACTOR = 10.0;						// [-] select MIN(catchdist, dist*CATCH_FACTOR) to overtake.
	const double CENTERDIV = 0.1;								// [-] (factor) [0.01..0.6].
	const double DISTCUTOFF = 100.0;							// [m] How far to look, terminate while loops.
	const double LOOKAHEAD_CONST = 17.0f;					// [m]
	const double LOOKAHEAD_FACTOR = 0.33f;				// [-]

	int stuck_count = 0;
	bool stuck_yn = false;
	double old_timer = 0;
	bool tenthTimer = false;

	double pre_forward_dist = 0;
	double track_backward_dist = 0;


	bool itstucked = false;
	int release_stuck_count = 0;

	vector<int> opponent_num;

	int opponent_state[10];

	string logStr;

	typedef struct Opponent
	{
		int state[10];
		double catch_distance[10];
		double distance[10];
		double speed[10];
		double side_dist[10];
		double width[10];
		double pre_dist_cars[20];

	} tOpponent;

	tOpponent opponent_inform;

	double myoffset;			// Offset to the track middle.
	double OVERTAKE_OFFSET_INC;		// [m/timestep]

	double old_lookahead;		// Lookahead for steering in the previous step.

	/******************
	생성자
	******************/
	RoadTest(){

		memset(opponent_inform.state, 0, sizeof(opponent_inform.state));
		memset(opponent_inform.catch_distance, 0, sizeof(opponent_inform.catch_distance));
		memset(opponent_inform.side_dist, 0, sizeof(opponent_inform.side_dist));
		memset(opponent_inform.pre_dist_cars, 0, sizeof(opponent_inform.pre_dist_cars));

		myoffset = 0.0;

		OVERTAKE_OFFSET_INC = OVERTAKE_OFFSET_SPEED * (double)RCM_MAX_DT_ROBOTS;

		old_lookahead = 0.0;
	}

	RoadTest(shared_use_st *shared){
		this->shared = shared;
	}

	~RoadTest(){

	}


	/******************
	메소드
	******************/
	double diffTrackDist(double longDist, double ShortDist);
	double diffTrackAngle(double largeAngle, double smallAngle);
	void setAdditionalParam();
	void truncAngle();
	//void getAllAttr(char *tmpStr);

	void updateTimer();

	// Check if I'm stuck.
	bool isStuck();

	// Init the friction coefficient of the the tires.
	double initTireMu();

	double getAllowedSpeed(int trackIdx);


	double getFilteredAccel();
	double getReleaseStuckAccel();
	double getAccel();


	// 이전 트랙의 끝 거리 설정
	void setBackwardDist();

	// 현재거리부터 이전 앞 트랙
	double getDistToSegStart();

	// 현재거리부터 첫번째 앞 트랙
	double getDistToSegEnd();

	// Compute the needed distance to brake.
	double brakedist(double allowedspeed, double mu);

	// Compute initial brake value.
	double getBrake();

	double filterBColl(double brake);

	double filterBrakeSpeed(double brake);

	double filterException(double brake);

	void updateOpponentState();

	double getOffset();

	double getTargetPoint();

	double getSteer();

	double rad_norm(double rad){
		if (rad >= 0 && rad <= PI / 2){
			return rad;
		}
		else if (rad > PI / 2 && rad <= PI){
			return PI - rad;
		}
		else if (rad >= PI * -1 && rad < PI / 2 * -1){
			return PI + rad;
		}
		else{
			return rad *-1;
		}
	}

	double filterSColl(double steer);
	void initTrack();



#define NORM_PI_PI(x) do{ while ((x) > PI) { (x) -= 2 * PI; } while ((x) < -PI) { (x) += 2 * PI; } } while (0)

#define SIGN(x)     ((x) < 0 ? -1.0 : 1.0)
};



using namespace std;

RoadTest rt;

int controlDriving(shared_use_st *shared){
	if (shared == NULL) return -1;
	if (shared->my_rank == 0){
		rt.release_stuck_count = 180;
		shared->accelCmd = 1.0;	// 100% accelerator pedal.
		return 0;
	}

	//current time update
	rt.frameNum++;

	if (rt.frameNum == 1){
		QueryPerformanceCounter(&rt.baseTime); //첫 request 시작시각
		QueryPerformanceFrequency(&rt.frequency);
		QueryPerformanceCounter(&rt.startTime); //request 시작시각
	}
	else{
		QueryPerformanceCounter(&rt.startTime); //request 시작시각
	}

	rt.clientTime = (rt.startTime.QuadPart - rt.baseTime.QuadPart) / (double)rt.frequency.QuadPart;

	rt.shared = shared;
	rt.setAdditionalParam();
	rt.updateOpponentState();


	/*************************************************
	알고리즘 구현부(Output : 4개의 Output Cmd 값을 도출하세요.)
	1. shared->steerCmd
	2. shared->brakeCmd
	3. shared->accelCmd
	4. shared->backwardCmd
	*************************************************/
	rt.setBackwardDist();
	rt.updateTimer();

	if (rt.isStuck()) {

		shared->steerCmd = -shared->angle / rt.STEER_LOCK;
		shared->backwardCmd = GEAR_BACKWARD;		// Reverse gear.
		shared->accelCmd = 1.0;	// 100% accelerator pedal.
		shared->brakeCmd = 0.0;	// No brakes.

	}
	else {

		shared->steerCmd = rt.filterSColl(rt.getSteer());
		shared->steerCmd = shared->steerCmd > 1 ? 1 : shared->steerCmd;
		shared->steerCmd = shared->steerCmd < -1 ? -1 : shared->steerCmd;
		if (rt.frameNum > 80)
			shared->brakeCmd = rt.filterException(rt.filterBrakeSpeed(rt.filterBColl(rt.getBrake())));
		else
			shared->brakeCmd = 0.0f;

		if (shared->brakeCmd == 0.0){
			//shared->accelCmd = rt.filterTCL(rt.filterTrk(rt.getAccel())) * 0.81;
			if (shared->speed < rt.MAX_SPEED_LIMIT)
				shared->accelCmd = rt.getFilteredAccel();
			else
				shared->accelCmd = 0.0;

			if (shared->accelCmd < 0)
				shared->accelCmd = 0.0f;
			shared->accelCmd = shared->accelCmd > 1 ? 1 : shared->accelCmd;

		}
		else{
			shared->accelCmd = 0.0f;
		}

		shared->backwardCmd = GEAR_FORWARD;
	}

	return 0;
}

void endShare(struct shared_use_st *&shared, HANDLE &hMap){
	// shared memory initialize
	if (shared != NULL)	{
		UnmapViewOfFile(shared);
		shared = NULL;
	}
	if (hMap != NULL) {
		CloseHandle(hMap);
		hMap = NULL;
	}
}

int main(int argc, char **argv){
	////////////////////// set up memory sharing
	struct shared_use_st *shared = NULL;

	// try to connect to shared memory 1
	HANDLE hMap = OpenFileMappingA(FILE_MAP_ALL_ACCESS, false, SHARED_MOMORY_NAME1);
	if (hMap == NULL){
		fprintf(stderr, "Shared Memory Map open failed.\n");
		exit(EXIT_FAILURE);
	}

	shared = (struct shared_use_st*) MapViewOfFile(hMap, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(struct shared_use_st));
	if (shared == NULL){
		fprintf(stderr, "Shared Memory Map open failed.\n");
		exit(EXIT_FAILURE);
	}

	// shared memory 1 is already occupied.
	if (shared->connected == true) {
		endShare(shared, hMap);
		hMap = OpenFileMappingA(FILE_MAP_ALL_ACCESS, false, SHARED_MOMORY_NAME2);
		if (hMap == NULL){
			fprintf(stderr, "Shared Memory Map open failed.\n");
			exit(EXIT_FAILURE);
		}
		shared = (struct shared_use_st*) MapViewOfFile(hMap, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(struct shared_use_st));
		if (shared == NULL){
			fprintf(stderr, "Shared Memory Map open failed.\n");
			exit(EXIT_FAILURE);
		}
	}
	printf("\n********** Memory sharing started, attached at %X **********\n", shared);

	////////////////////// DON'T TOUCH IT - Default Setting
	shared->connected = true;
	shared->written = 0;
	////////////////////// END Default Setting

	////////////////////// Initialize
	shared->steerCmd = 0.0;
	shared->accelCmd = 0.0;
	shared->brakeCmd = 0.0;
	shared->backwardCmd = GEAR_FORWARD;
	////////////////////// END Initialize

	while (shared->connected){
		if (shared->written == 1) { // the new image data is ready to be read
			controlDriving(shared);
			shared->written = 0;
		}

		if (_kbhit()){
			char key = _getch();
			if (key == 'q' || key == 'Q'){
				break;
			}
		}
	}

	endShare(shared, hMap);

	return 0;
}


void RoadTest::initTrack(){
	//CG

	if ((int)shared->dist_track == 3185){
		FRICTION = 1.2;

		//steer
		steerLookaheadCoef1 = 17.0 ;
		steerLookaheadCoef2 = 0.33 ;
		steerOutlineCoef = 0.0;
	}
	//E1
	else if ((int)shared->dist_track == 3243){
		FRICTION = 1.2;

		//steer
		steerLookaheadCoef1 = 15.0 ;
		steerLookaheadCoef2 = 0.33 ;
		steerOutlineCoef = 0.0;
	}
	//E2
	else if ((int)shared->dist_track == 5380){
		FRICTION = 1.04;
		//steer
		steerLookaheadCoef1 = 17.0;
		steerLookaheadCoef2 = 0.33;
		steerOutlineCoef = 0.0;
	}
	//E6
	else if ((int)shared->dist_track == 4441){
		FRICTION = 1.08;

		//steer
		steerLookaheadCoef1 = 17.0;
		steerLookaheadCoef2 = 0.33;
		steerOutlineCoef = 0.0;
	}
	//E Road
	else {
		FRICTION = 1.2;

		//steer
		steerLookaheadCoef1 = 15.0;
		steerLookaheadCoef2 = 0.33;
		steerOutlineCoef = 0.0;
	}
}

double RoadTest::diffTrackDist(double longDist, double ShortDist){
	return (longDist > ShortDist ? longDist : longDist + shared->dist_track) - ShortDist;
}

double RoadTest::diffTrackAngle(double nextAngle, double curAngle){
	double rst = nextAngle - curAngle;

	NORM_PI_PI(rst);

	return rst;
}

void RoadTest::setAdditionalParam(){

	truncAngle();


	//frameNum ==1 일때는 쓰레기값이 많아서 처리하기 어려움
	//트랙정보로 세팅
	if (frameNum == 1){
		initTrack();
		track_current_dist = shared->toStart;
	}
	else if (prevTrack_forward_dist[0] != shared->track_forward_dists[0]){
		///*******************track_forward_dist2/angle2******************/
		double len;
		double len_margin = trackUnitLength / 2;
		int dist2Idx = 0;

		//init dist2Idx -> 항상 toStart 앞에 가장가까운 트랙 세그먼트가 놓일수있도록
		for (int i = 0; i < INPUT_FORWARD_TRACK_SIZE; i++){
			if (prevTrack_forward_dist[i] > shared->toStart){
				for (int j = i; j < INPUT_FORWARD_TRACK_SIZE; j++){
					if (prevTrack_forward_dist[j] >= shared->track_forward_dists[0])
						break;

					track_forward_angle2[j - i] = prevTrack_forward_angle[j];
					track_forward_dist2[j - i] = prevTrack_forward_dist[j];
					dist2Idx = j - i;
				}
				break;
			}
		}

		bool tmpFlag = false;
		for (int i = 1; i < INPUT_FORWARD_TRACK_SIZE; i++){
			len = diffTrackDist(shared->track_forward_dists[i], shared->track_forward_dists[i - 1]);
			if (len <= (trackUnitLength + len_margin)){
				track_forward_angle2[dist2Idx] = shared->track_forward_angles[i - 1];
				track_forward_dist2[dist2Idx++] = shared->track_forward_dists[i - 1];
			}
			else{
				tmpFlag = true;
				double tmpLen = len;

				//앵글은 나누어 들어가는 만큼으로 나눠서 넣는다.
				double tmpAngle = diffTrackAngle(shared->track_forward_angles[i], shared->track_forward_angles[i - 1]) / (double)((int)(len / trackUnitLength));

				track_forward_angle2[dist2Idx] = shared->track_forward_angles[i - 1];
				track_forward_dist2[dist2Idx++] = shared->track_forward_dists[i - 1];

				if (dist2Idx == INPUT_FORWARD_TRACK_SIZE) {
					break;
				}

				while (1){
					track_forward_angle2[dist2Idx] = track_forward_angle2[dist2Idx - 1] + tmpAngle;
					track_forward_angle2[dist2Idx] = diffTrackAngle(track_forward_angle2[dist2Idx], 0); //PI normalize
					track_forward_dist2[dist2Idx++] = trackUnitLength + track_forward_dist2[dist2Idx - 1] >= shared->dist_track ? trackUnitLength + track_forward_dist2[dist2Idx - 1] - shared->dist_track : trackUnitLength + track_forward_dist2[dist2Idx - 1];
					tmpLen -= trackUnitLength;

					if (dist2Idx == INPUT_FORWARD_TRACK_SIZE) {
						break;
					}
					else if (tmpLen <= trackUnitLength){
						break;
					}
				}
			}

			if (dist2Idx == INPUT_FORWARD_TRACK_SIZE) break;
		}

		//마지막 인덱스에 값이 안채워질수있으므로
		if (dist2Idx != INPUT_FORWARD_TRACK_SIZE){
			track_forward_angle2[INPUT_FORWARD_TRACK_SIZE - 1] = shared->track_forward_angles[INPUT_FORWARD_TRACK_SIZE - 1];
			track_forward_dist2[INPUT_FORWARD_TRACK_SIZE - 1] = shared->track_forward_dists[INPUT_FORWARD_TRACK_SIZE - 1];
		}
		///*******************track_forward_dist2/angle2******************/


		/*****************헤어핀커브 세팅**************/
		for (int i = 1; i < INPUT_FORWARD_TRACK_SIZE; i++){
			double diffAngle = fabs(diffTrackAngle(shared->track_forward_angles[i], shared->track_forward_angles[i - 1]));

			if (!in_hairpin_curve){
				if (dist_to_hairpin_curve > 0.0 && shared->toStart > dist_to_hairpin_curve){
					in_hairpin_curve = true;
					dist_to_hairpin_curve = 0.0;
					dist_to_not_hairpin_curve = shared->track_forward_dists[i - 1];
				}
			}
			else{
				if (shared->toStart > dist_to_not_hairpin_curve){
					in_hairpin_curve = false;
					dist_to_not_hairpin_curve = 0.0;
				}
			}

			if (diffAngle > hairpin_angle) {
				dist_to_hairpin_curve = shared->track_forward_dists[i - 1];
				break;
			}
		}
		/*****************헤어핀커브 세팅**************/

		//track_current_dist 세팅
		for (int i = 1; i < INPUT_FORWARD_TRACK_SIZE; i++){
			if (prevTrack_forward_dist[i] == track_forward_dist2[0]){
				track_current_dist = prevTrack_forward_dist[i - 1];
				break;
			}
		}
	}

	memcpy(prevTrack_forward_dist, track_forward_dist2, sizeof(track_forward_dist2));
	memcpy(prevTrack_forward_angle, track_forward_angle2, sizeof(track_forward_angle2));

	/*******************track radius******************/

	//current track radius
	double diffAngle;
	diffAngle = diffTrackAngle(track_forward_angle2[0], shared->track_current_angle);

	if (diffAngle == 0.0){
		track_current_curve_type = CURVE_TYPE_STRAIGHT;
		track_forward_type[0] = CURVE_TYPE_STRAIGHT;
	}
	else if (diffAngle < 0.0){
		track_current_curve_type = CURVE_TYPE_RIGHT;
		track_forward_type[0] = CURVE_TYPE_RIGHT;
	}
	else{
		track_current_curve_type = CURVE_TYPE_LEFT;
		track_forward_type[0] = CURVE_TYPE_LEFT;
	}

	diffAngle = fabs(diffAngle);

	if (diffAngle > 0){
		track_forward_radius[0] = diffTrackDist(track_forward_dist2[0], track_current_dist) / diffAngle;;
		track_forward_point[0][0] = track_forward_radius[0] * sin(diffAngle);
		track_forward_point[0][1] = track_forward_radius[0] * (1 - cos(diffAngle));
	}
	else{
		track_forward_radius[0] = 10000; //그냥 큰수
		track_forward_point[0][0] = diffTrackDist(track_forward_dist2[0], track_current_dist);
		track_forward_point[0][1] = 0;
	}

	//forward track radius
	for (int i = 1; i < INPUT_FORWARD_TRACK_SIZE; i++){
		double diffAngle;
		diffAngle = diffTrackAngle(track_forward_angle2[i], track_forward_angle2[i - 1]);

		if (diffAngle == 0.0){
			track_forward_type[i] = CURVE_TYPE_STRAIGHT;
		}
		else if (diffAngle < 0.0){
			track_forward_type[i] = CURVE_TYPE_RIGHT;
		}
		else{
			track_forward_type[i] = CURVE_TYPE_LEFT;
		}

		diffAngle = fabs(diffAngle);


		if (diffAngle > 0){
			track_forward_radius[i] = diffTrackDist(track_forward_dist2[i], track_forward_dist2[i - 1]) / diffAngle;
			track_forward_point[i][0] = track_forward_point[i - 1][0] + track_forward_radius[i] * sin(diffAngle);
			track_forward_point[i][1] = track_forward_point[i - 1][1] + track_forward_radius[i] * (1 - cos(diffAngle));
		}
		else{
			track_forward_radius[i] = 10000; //그냥 큰수
			track_forward_point[i][0] = track_forward_point[i - 1][0] + diffTrackDist(track_forward_dist2[i], track_forward_dist2[i - 1]);
			track_forward_point[i][1] = track_forward_point[i - 1][1];
		}
	}

	current_point[0] = shared->toStart - track_current_dist;
	if (track_current_curve_type == CURVE_TYPE_STRAIGHT){
		current_point[1] = shared->toMiddle*-1;
	}
	else if (track_current_curve_type == CURVE_TYPE_RIGHT){
		current_point[1] = shared->toMiddle*-1;
	}
	else{
		current_point[1] = shared->toMiddle;
	}
	/*******************track radius******************/


	// insert
	opponent_num.clear();
	backward_opponents_n = 0, forward_opponents_n = 0;
	for (int j = 0; j < INPUT_AICAR_SIZE; j++)
	{
		if (j <= 9){
			if (j % 2 == 0)
			if (shared->dist_cars[j] < 100)
			{
				forward_opponents_n++;
				opponent_num.push_back(j);
			}
		}
		else
		{
			if (j % 2 == 0)
			if (shared->dist_cars[j] > -100)
			{
				backward_opponents_n++;
				opponent_num.push_back(j);
			}
		}
	}
}

void RoadTest::updateTimer()
{
	double diff = clientTime - old_timer;
	if (diff >= 0.1) {
		old_timer += 0.1;
		tenthTimer = true;
	}
	else {
		tenthTimer = false;
	}
}
// Check if I'm stuck.

bool RoadTest::isStuck()
{
	
	if ((shared->speed > -9 && shared->speed < MAX_UNSTUCK_SPEED )&& 		
		((fabs(shared->angle) > MAX_UNSTUCK_ANGLE &&
		fabs(shared->toMiddle) > MIN_UNSTUCK_DIST)  ||		
		(opponent_inform.distance[0] < 6 || opponent_inform.distance[5] > -6) ) ) {
		if (stuck_count > MAX_UNSTUCK_COUNT && shared->toMiddle * shared->angle < 0.0) {
			return true;
			release_stuck_count = 0;
		}
		else {
			stuck_count++;
			return false;
		}
	}
	else {
		release_stuck_count += 12;
		stuck_count = 0;
		return false;
	}
}

// Init the friction coefficient of the the tires.
double RoadTest::initTireMu()
{

	// 0 : front left, 1 : front right, 2 : rear left, 3 : rear right
	double tire_mu[4] = { 1.6, 1.6, 1.6, 1.6 };

	double tm = DBL_MAX;

	for (int i = 0; i < 4; i++) {
		tm = MIN(tm, tire_mu[i]);
	}
	return tm;
}

double RoadTest::getAllowedSpeed(int trackIdx)
{
	//마찰력 * 타이어계수 * MU_FACTOR 을 이용하여 마찰계수 구함

	double mu = FRICTION * TIRE_MU * MU_FACTOR;

	//반지름
	double minRadius = 20.0; //왜냐면 급커브에서 반지름 계산이 잘 안됨. 너무 작은 반지름으로 인해 잘 안돔 실제와 차이가 크다.

	double toMiddleOffset = track_forward_type[trackIdx] == CURVE_TYPE_LEFT ? shared->toMiddle*-1 : shared->toMiddle;
	double r = track_forward_radius[trackIdx] + toMiddleOffset;
	double r2 = track_forward_radius[trackIdx] - toMiddleOffset;
	r = MAX(minRadius, r);
	r2 = MAX(minRadius, r2);

	return sqrt((mu*G*r) / (1.0f - MIN(1.0f, r*CA*mu / MASS)));

}

double RoadTest::getFilteredAccel(){
	int isStraight = 0;

	for (int i = 0; i < 18; i++)
	{
		if (getAllowedSpeed(i) > 350.0f)
			isStraight += i + 1;
	}
	//171
	if (shared->speed < 50)
		return getReleaseStuckAccel();


	if (isStraight >= 100){
		return getReleaseStuckAccel();
	}
	else if (isStraight > 20)
	{
		if (shared->speed > 60)
			return getReleaseStuckAccel() * isStraight / 171.0f;
		else
			return getReleaseStuckAccel() * isStraight / 18.0f;
	}
	else{
		if (shared->speed < 50)
			return getReleaseStuckAccel() * 0.63f;
		else
			return 0.0f;
	}
}

double RoadTest::getReleaseStuckAccel(){

	if (release_stuck_count < 100)
	{
		return getAccel() * 0.01f * release_stuck_count;
	}

	return getAccel();
}


double RoadTest::getAccel()
{
	int gear = (int)(shared->speed / 16.667f);

	double gr = 0;
	if (gear <= 1)
		gr = 3.5;
	else if (gear <= 2)
		gr = 2.6;
	else if (gear <= 3)
		gr = 1.9;
	else if (gear <= 4)
		gr = 1.54;
	else if (gear <= 5)
		gr = 1.25;
	else if (gear <= 6)
		gr = 1.02;

	if (gear  > 0) {

		double allowedspeed = getAllowedSpeed(0);
		if (allowedspeed > 100)
			allowedspeed = 100.f;

		if (allowedspeed > shared->speed + 1.0f) {
			if (fabs(shared->angle) < PI / 4)
				return allowedspeed / MAX_SPEED;
			else
				return 0.3f;
		}
		else {

			double grResult = allowedspeed * gr / 350;
			grResult = grResult > 1 ? 1 : grResult;

			if (fabs(shared->angle) < PI / 4)
				return grResult;
			else
				return 0.15;
		}
	}
	else {
		if (fabs(shared->angle) < PI / 12.0f && abs(shared->toMiddle) < shared->track_width / 2.1)
			return 0.45f;
		else
			return 0.3f;
	}
}

void RoadTest::setBackwardDist() {
	if (track_forward_dist2[0] != pre_forward_dist)
		track_backward_dist = pre_forward_dist;

	pre_forward_dist = track_forward_dist2[0];
}

// 이전 트랙의 끝 까지의 거리

double RoadTest::getDistToSegStart()
{
	return fabs(shared->toStart - track_backward_dist);
}

// 현재거리부터 첫번째 앞 트랙

double RoadTest::getDistToSegEnd()
{
	if (track_forward_dist2[0] < shared->toStart){
		return track_forward_dist2[0] - shared->toStart + shared->dist_track;
	}
	else{
		return track_forward_dist2[0] - shared->toStart;
	}
}

// Compute the needed distance to brake.
double RoadTest::brakedist(double allowedspeed, double mu)
{

	double c = mu*G;
	double d = mu / MASS;
	double v1sqr = shared->speed*shared->speed;
	double v2sqr = allowedspeed*allowedspeed;

	//return -log((c + v2sqr*d) / (c + v1sqr*d)) / (2.0f*d);
	return (v1sqr - v2sqr) / (2.0*c);
}


// Compute initial brake value.
double RoadTest::getBrake()
{
	// Car drives backward? 차가 뒤로 -18km 가면 풀브레이크 잡는다.
	if (shared->speed < -MAX_UNSTUCK_SPEED) {
		// Yes, brake.
		return 1.0;
	}


	//직진 또는 커브에서의 최대속도를 구함
	int trackIdx = 0;
	double allowedspeed = getAllowedSpeed(trackIdx);
	if (dist_to_hairpin_curve > 0 && dist_to_hairpin_curve < shared->toStart + shared->speed / 2){
		return 1.0f;
	}
	else if (allowedspeed < shared->speed) {

		return MIN(1.0f, (shared->speed - allowedspeed) / (FULL_ACCEL_MARGIN));
	}


	//트랙세그먼트는 트랙 정보를 저장 : 직진, 커브 정보 등
	double mu = FRICTION; //trackSurface, Coefficient of friction

	double maxlookaheaddist = (shared->speed * shared->speed) / (2 * mu*G); //현재속도의제곱 / (2 * 마찰계수*중력가속도)
	double lookaheaddist = getDistToSegEnd();

	trackIdx++;

	while (lookaheaddist < maxlookaheaddist) {
		if (trackIdx >= INPUT_FORWARD_TRACK_SIZE){
			return 0.0f;
		}
		allowedspeed = getAllowedSpeed(trackIdx);

		if (allowedspeed < shared->speed) {

			//브레이크에 걸리는 거리
			double tmpBrakingDist = brakedist(allowedspeed, mu);

			if (tmpBrakingDist > lookaheaddist) {
				return 1.0f;
			}
		}
		lookaheaddist += track_forward_dist2[trackIdx] - track_forward_dist2[trackIdx - 1];
		trackIdx++;
	}

	return 0.0f;
}

double RoadTest::filterBColl(double brake)
{
	double mu = FRICTION;
	double tmpBrakingDist;

	int i;
	int frontColIndex = -1;

	for (i = 0; i < 5; i++) {

		if (shared->dist_cars[2 * i] > 0 && (shared->dist_cars[2 * i] < 80 && shared->dist_cars[2 * i] > 10))
		{
			//1. tomiddle 판단
			if (fabs(shared->dist_cars[2 * i + 1] - shared->toMiddle) < (CAR_WIDTH / 2 + CAR_WIDTH / 2)){
				tmpBrakingDist = brakedist(0.0, mu) + CAR_LENGTH;

				if (tmpBrakingDist > shared->dist_cars[2 * i]) {
					return 1.0f;
				}
			}
		}
	}

	return brake;
}

// Reduces the brake value such that it fits the speed (more downforce -> more braking).
double RoadTest::filterBrakeSpeed(double brake)
{
	double weight = MASS*G;
	double maxForce = weight + CA*MAX_SPEED*MAX_SPEED;
	double force = weight + CA * shared->speed * shared->speed;
	return brake*force / maxForce;
}

double RoadTest::filterException(double brake){
	

	return brake;
}

void RoadTest::updateOpponentState()
{
	if (memcmp(shared->dist_cars, opponent_inform.pre_dist_cars, sizeof(shared->dist_cars)) == 0)
		return;

	if (DEBUG_MODE == TRUE)

	for (int i = 0; i < 10; i++)
	{
		opponent_inform.state[i] = OPP_IGNORE;

		opponent_inform.distance[i] = shared->dist_cars[i * 2];
		double side_distance = shared->dist_cars[i * 2 + 1];
		//TODO : need to check
		// car7-trb1 : 1.94, car1-ow1 : 2.4, car6-trb1 : 1.94, kc-dbs : 1.9, pw-corollawrc : 1.98 
		//float SIDECOLLDIST = MIN(car->_dimension_x, mycar->_dimension_x);
		//double SIDECOLLDIST = 1.9;

		// speed = meter / sec

		// car width = 1.94, length = 4.39
		double SIDECOLLDIST = 4.39;
		double car_margin = sqrt(pow((1.94 / 2), 2) + pow((4.39 / 2), 2)) * 2;

		// 상대속도계산

		if (fabs(shared->dist_cars[i * 2] - opponent_inform.pre_dist_cars[i * 2]) > 50.0 &&
			fabs(opponent_inform.pre_dist_cars[i * 2]) > fabs(shared->dist_cars[i * 2])) {
			opponent_inform.speed[i] = fabs(shared->speed) + (shared->dist_cars[i * 2]) * 10;
		}
		else{
			opponent_inform.speed[i] = fabs(shared->speed) + (shared->dist_cars[i * 2] - opponent_inform.pre_dist_cars[i * 2]) * 10;
		}
		opponent_inform.speed[i] = opponent_inform.speed[i] < 0 ? 0 : opponent_inform.speed[i];

		// Is opponent in relevant range -BACKCOLLDIST..FRONTCOLLDIST m.
		if (opponent_inform.distance[i] > -BACKCOLLDIST && opponent_inform.distance[i] < FRONTCOLLDIST)
		{
			if (opponent_inform.speed[i] < 10) {
				opponent_inform.state[i] |= OPP_LETPASS;
			}
			// Is opponent in front and slower.
			if (opponent_inform.distance[i] > SIDECOLLDIST && shared->speed > opponent_inform.speed[i]) {
				opponent_inform.state[i] |= OPP_FRONT;

				//opponent_inform.distance[i] -= CAR_LENGTH;
				//opponent_inform.distance[i] -= LENGTH_MARGIN;

				//distance = sqrt(pow(distance, 2) + pow((shared->toMiddle - side_distance), 2));
				// If the distance is small we compute it more accurate.
				if (opponent_inform.distance[i] < EXACT_DIST)
				{
					opponent_inform.distance[i] -= car_margin;
				}

				// catch distance 정보
				opponent_inform.catch_distance[i] = shared->speed * opponent_inform.distance[i] / (opponent_inform.speed[i] - shared->speed);

				double cardist = opponent_inform.pre_dist_cars[i * 2 + 1] - shared->toMiddle;
				opponent_inform.side_dist[i] = cardist;
				// opponent의 angle을 알 수 없기 때문에
				// 장확한 width를 구할 수 없음. 

				// margin 추가 0.5
				cardist = fabs(cardist) - fabs(CAR_LENGTH / 2.0) * 2 + 0.5;
				if (cardist < SIDE_MARGIN) {
					opponent_inform.state[i] |= OPP_COLL;
				}
			}
			else
				// Is opponent behind and faster.
			if (opponent_inform.distance[i] < -SIDECOLLDIST && shared->speed > opponent_inform.speed[i] - SPEED_PASS_MARGIN) {
				opponent_inform.catch_distance[i] = shared->speed * opponent_inform.distance[i] / (shared->speed - opponent_inform.speed[i]);
				opponent_inform.state[i] |= OPP_BACK;
				opponent_inform.distance[i] -= CAR_LENGTH;
				opponent_inform.distance[i] -= LENGTH_MARGIN;
			}
			else
				// Is opponent aside.
			if (opponent_inform.distance[i] > -SIDECOLLDIST &&
				opponent_inform.distance[i] < SIDECOLLDIST &&
				opponent_inform.speed[i] > shared->speed) {
				opponent_inform.side_dist[i] = shared->dist_cars[i * 2 + 1] - shared->toMiddle;
				opponent_inform.state[i] |= OPP_SIDE;
			}
			else
				// Opponent is in front and faster.
			if (opponent_inform.distance[i] > SIDECOLLDIST && shared->speed > opponent_inform.speed[i]) {
				opponent_inform.state[i] |= OPP_FRONT_FAST;
			}
		}
	}
	getOffset();
	memcpy(opponent_inform.pre_dist_cars, shared->dist_cars, sizeof(shared->dist_cars));
}

double RoadTest::getTargetPoint()
{
	double lookahead;
	double length = getDistToSegEnd();
	double offset = getOffset();

	// Usual lookahead.
	lookahead = LOOKAHEAD_CONST + shared->speed * LOOKAHEAD_FACTOR;
	// Prevent "snap back" of lookahead on harsh braking.
	double cmp_lookahead = old_lookahead - shared->speed * RCM_MAX_DT_ROBOTS;
	if (lookahead < cmp_lookahead) {
		lookahead = cmp_lookahead;
	}

	old_lookahead = lookahead;

	// Search for the segment containing the target point.
	int next_seg_index = 0;
	while (length < lookahead) {
		next_seg_index++;
		length += shared->track_forward_dists[next_seg_index] - shared->track_forward_dists[next_seg_index - 1];
	}

	// Compute the target point.
	offset = myoffset;




	double diff = diffTrackAngle(shared->track_current_angle, shared->track_forward_angles[next_seg_index]);
	return 0.0;
}

// Compute offset to normal target point for overtaking or let pass an opponent.
double RoadTest::getOffset()
{

	double catch_dist, min_catch_dist = DBL_MAX, min_dist = -1000.0;
	int opponent_index = -1;

	// Increment speed dependent.
	double incfactor = MAX_INC_FACTOR - MIN(fabs(shared->speed) / MAX_INC_FACTOR, (MAX_INC_FACTOR - 1.0));

	// Let overlap or let less damaged team mate pass.
	for (int i = 0; i < 10; i++) {
		if (opponent_inform.state[i] & OPP_LETPASS) {
			// Behind, larger distances are smaller ("more negative").
			if (opponent_inform.distance[i] > min_dist) {
				min_dist = opponent_inform.distance[i];
				opponent_index = i;
			}
		}
	}

	if (opponent_index != -1) {
		double side = shared->toMiddle - shared->dist_cars[opponent_index * 2 + 1];
		double w = shared->track_width / WIDTHDIV - BORDER_OVERTAKE_MARGIN;

		if (side > 0.0f) {
			if (myoffset < w) {
				myoffset += OVERTAKE_OFFSET_INC * incfactor;
			}
		}
		else {
			if (myoffset > -w) {
				myoffset -= OVERTAKE_OFFSET_INC * incfactor;
			}
		}
		if (DEBUG_MODE == TRUE)
			return myoffset;
	}

	// Overtake.
	for (int i = 0; i < 10; i++) {
		if (opponent_inform.state[i] & OPP_FRONT) {
			catch_dist = MIN(opponent_inform.catch_distance[i], opponent_inform.distance[i] * CATCH_FACTOR);

			if (catch_dist < min_catch_dist) {
				min_catch_dist = catch_dist;
				opponent_index = i;
			}
		}
	}

	if (opponent_index != -1) {
		// Compute the width around the middle which we can use for overtaking.
		double w = shared->track_width / WIDTHDIV - BORDER_OVERTAKE_MARGIN;
		// Compute the opponents distance to the middle.
		double otm = shared->dist_cars[opponent_index * 2 + 1];
		// Define the with of the middle range.
		double wm = shared->track_width * CENTERDIV;

		if (otm > wm && myoffset > -w) {
			myoffset -= OVERTAKE_OFFSET_INC * incfactor;
		}
		else if (otm < -wm && myoffset < w) {
			myoffset += OVERTAKE_OFFSET_INC * incfactor;
		}
		else {
			// If the opponent is near the middle we try to move the offset toward
			// the inside of the expected turn.
			// Try to find out the characteristic of the track up to catchdist.
			double length = getDistToSegEnd();
			double old_len, seg_len = length;
			double len_right = 0.0f, len_left = 0.0f;
			min_catch_dist = MIN(min_catch_dist, DISTCUTOFF);

			int next_track_index = 0;
			do {
				switch (track_forward_type[next_track_index]) {
				case CURVE_TYPE_LEFT:
					len_left += seg_len;
					break;
				case CURVE_TYPE_RIGHT:
					len_right += seg_len;
					break;
				default:
					// Do nothing.
					break;
				}
				next_track_index++;
				seg_len = track_forward_dist2[next_track_index] - track_forward_dist2[next_track_index - 1];
				old_len = length;
				length += seg_len;
			} while (old_len < min_catch_dist);

			// If we are on a straight look for the next turn.
			if (len_left == 0.0 && len_right == 0.0) {
				while (track_forward_type[next_track_index] == CURVE_TYPE_STRAIGHT) {
					next_track_index++;
				}
				// Assume: left or right if not straight.
				if (track_forward_type[next_track_index] == CURVE_TYPE_LEFT) {
					len_left = 1.0f;
				}
				else {
					len_right = 1.0f;
				}
			}
			// Because we are inside we can go to the border.
			double max_off = (shared->track_width - CAR_WIDTH) / 2.0 - BORDER_OVERTAKE_MARGIN;
			if (len_left > len_right) {
				if (myoffset < max_off) {
					myoffset += OVERTAKE_OFFSET_INC * incfactor;
				}
			}
			else {
				if (myoffset > -max_off) {
					myoffset -= OVERTAKE_OFFSET_INC * incfactor;
				}
			}
		}
	}
	else {
		// There is no opponent to overtake, so the offset goes slowly back to zero.
		if (myoffset > OVERTAKE_OFFSET_INC) {
			myoffset -= OVERTAKE_OFFSET_INC;
		}
		else if (myoffset < -OVERTAKE_OFFSET_INC) {
			myoffset += OVERTAKE_OFFSET_INC;
		}
		else {
			myoffset = 0.0f;
		}
	}
	if (DEBUG_MODE == TRUE)
		return myoffset;
}

double RoadTest::getSteer(){
	
	double aRatio = 1.0;
	double aForce;
	double bRatio = 0.0;
	double bForce = 0.0;
	double gain = 1.1;


	//========= a전략 ===========//
	if (abs(shared->angle - shared->toMiddle / shared->track_width) > CENTER_RANGE_1)
		aForce = (shared->angle - shared->toMiddle / shared->track_width) * MAX_CENTER_STEER;
	else
		aForce = (shared->angle - shared->toMiddle / shared->track_width) * MIN_CENTER_STEER;
	//========= a전략 ===========//


	//========= b전략 ===========//
	double targetAngle;

	double lookahead = steerLookaheadCoef1 + shared->speed * steerLookaheadCoef2;
	double length = getDistToSegEnd();

	if (length == 0) return 0.0;
	else if (track_forward_dist2[0] - shared->toStart <= 0) return 0.0;

	int targetIdx = 0;
	// Search for the segment containing the target point.
	while (length < lookahead) {
		targetIdx++;
		length += ((track_forward_dist2[targetIdx]>track_forward_dist2[targetIdx - 1] ? track_forward_dist2[targetIdx] : track_forward_dist2[targetIdx] + shared->dist_track) - track_forward_dist2[targetIdx - 1]);
	}

	double offsetY = track_current_dist - shared->toStart; //길이
	double offsetX = shared->toMiddle*-1.0;

	targetAngle = atan2(track_forward_point[targetIdx][0] - current_point[0], track_forward_point[targetIdx][1] - current_point[1]);


	double lookahead_angle;
	double outlineAngle = 0.0;

	if (track_current_curve_type == CURVE_TYPE_LEFT){
		lookahead_angle = (targetAngle - (PI / 2) - shared->angle);
		lookahead_angle = lookahead_angle > PI / 2 ? PI - lookahead_angle : lookahead_angle;
		lookahead_angle = lookahead_angle*-1;
	}
	else if (track_current_curve_type == CURVE_TYPE_RIGHT){
		lookahead_angle = (targetAngle - (PI / 2) + shared->angle);
		lookahead_angle = lookahead_angle > PI / 2 ? PI - lookahead_angle : lookahead_angle;
	}
	else{
		lookahead_angle = (targetAngle - (PI / 2) + shared->angle);
		lookahead_angle = lookahead_angle > PI / 2 ? PI - lookahead_angle : lookahead_angle;

		double sign = 1.0;
		if (shared->track_curve_type == CURVE_TYPE_LEFT){
			outlineAngle = atan2(shared->track_dist_straight, (shared->track_width / 2.0 + shared->toMiddle));
			if (outlineAngle < PI / 2 && lookahead_angle < outlineAngle)
				sign = -1.0;
		}
		else if (shared->track_curve_type == CURVE_TYPE_RIGHT){
			outlineAngle = atan2(shared->track_dist_straight, (shared->track_width / 2.0 - shared->toMiddle));
			if (outlineAngle < PI / 2 && lookahead_angle < outlineAngle)
				sign = 1.0;
		}

		//트랙끝에 쏠린경우는 적용안함
		if (abs(shared->toMiddle) > shared->track_width / 2.0 - CAR_WIDTH / 1.5){
			sign = 0.0;
		}
		//스트레이트 트랙 제한을 둠
		else if (shared->track_dist_straight > 200 || shared->track_dist_straight < shared->speed / 2){
			sign = 0.0;
		}

		lookahead_angle = lookahead_angle + (outlineAngle / (PI / 2))*steerOutlineCoef*sign; //outlineAngle은 0~90도 사이의 값이고 점점 영향력이 적게 준다.
	}


	/*******아웃인 아웃 전략********/

	//if (shared->track_dist_straight <= 7)// && shared->track_dist_straight > 1)
	//	targetAngle = atan2(track_forward_point[targetIdx][0] - current_point[0] + 1.5, track_forward_point[targetIdx][1] - current_point[1] + shared->track_width / 2.5);
	//else
	//	targetAngle = atan2(track_forward_point[targetIdx][0] - current_point[0] + 2, track_forward_point[targetIdx][1] - current_point[1]);
	/*******아웃인 아웃 전략********/


	bForce = lookahead_angle;

	//========= b전략 ===========//

	/************************현재상황판단***************************/

	if (in_hairpin_curve){
		aRatio = 1.0;
		bRatio = 0.0;
	}
	else{
		aRatio = 0.0;
		bRatio = 1.0;
	}

	/****************************현재상황판단*****************************/

	return (aForce * aRatio + bForce * bRatio) * gain;
}

void RoadTest::truncAngle(){
	shared->angle = floor(shared->angle*10000.0) / 10000.0;
	shared->track_current_angle = floor(shared->track_current_angle*10000.0) / 10000.0;
	for (int i = 0; i < INPUT_FORWARD_TRACK_SIZE; i++){
		shared->track_forward_angles[i] = floor(shared->track_forward_angles[i] * 10000.0) / 10000.0;
	}
}


double RoadTest::filterSColl(double steer)
{

	double lline = 0.0f, rline = 0.0f;
	double langle = 0.0f, rangle = 0.0f;
	double steerRatio = 1.75;
	double padding = 1.0;
	double gap = (CAR_WIDTH / 2 + CAR_LENGTH / 2) + padding;
	int opponent[5] = { -1, -1, -1, -1, -1 };
	int opponentNum = 0;

	//========= 출발전략 ===========//
	if (shared->my_rank > 1 && frameNum < 95){
		if (shared->toMiddle < 0)
			return (shared->angle - (shared->toMiddle + shared->track_width / 2.7f) / shared->track_width) * (1.0f / 0.541052);
		else
			return (shared->angle - (shared->toMiddle - shared->track_width / 2.7f) / shared->track_width) * (1.0f / 0.541052);
	}
	//========= 출발전략 ===========//

	for (int i = 0; i < 5; i++) {
		if (shared->dist_cars[2 * i] > 0 && shared->dist_cars[2 * i] < 23.7)
		{
			//1. tomiddle으로 부딪치는 차 인덱스 저장
			if (fabs(shared->dist_cars[2 * i + 1] - shared->toMiddle) < gap){
				opponent[opponentNum++] = 2 * i;
			}
		}
	}

	//전방차량 없으면 전방차량 없는 것으로 세팅
	if (opponentNum == 0){
		nearestCar = 100.0;
	}
	else{
		//todo : 직전 프레임에서 가장 가까운 부딪치는 차의 거리가 줄어들어야 한다.
		if (nearestCar > shared->dist_cars[opponent[0]]){

			//충돌예상 차들의 회피각도를 구함
			double avgToMiddle = 0.0;
			for (int i = 0; i < opponentNum; i++) {
				langle = MIN((atan2(shared->dist_cars[opponent[i]], fabs(shared->dist_cars[opponent[i] + 1] + gap - shared->toMiddle)) - (PI / 2)), langle) *-1;
				rangle = MIN((atan2(shared->dist_cars[opponent[i]], fabs(shared->dist_cars[opponent[i] + 1] - gap - shared->toMiddle)) - (PI / 2)), rangle);
				avgToMiddle += shared->dist_cars[opponent[i] + 1];
			}
			avgToMiddle /= opponentNum;


			if (steer>0){
				if (avgToMiddle < shared->toMiddle){
					steer += langle *steerRatio;

				}
				else{
					steer += rangle *steerRatio;
				}
			}
			else{
				if (avgToMiddle < shared->toMiddle){
					steer += langle *steerRatio;
				}
				else{
					steer += rangle *steerRatio;
				}
			}

		}

		nearestCar = shared->dist_cars[opponent[0]];
	}

	return steer;
}