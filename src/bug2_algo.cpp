// created by buenos at 2021.3.26
#include <Arduino.h>

#define MAX_ANGLE_VEL  0.2
#define MAX_LINEAR_VEL 0.1
#define DIS_THRESHOLD  0.1
#define OBST_THRESHOLD 0.10
#define SENSOR_RANGE   0.2
#define YAW_THRESHOLD  0.03
#define LOOP_RATE 5

//机器人状态
enum class State {
    toEnd = 0,
    followWall,
    turn,
    go,
    done
};
int robotState = 0;
State runState = State::turn;
//距离和位置
//三个传感器的数据
float dCenter, dLeft, dRight;
float distToLine;
float x = 0, y = 0, yaw = 0;
float xStart = 0, yStart = 0;
float xFinal = 15.26, yFinal = 56.21;


char frame[32];                //帧
char frameEnd[2] = {0x0a, 0x0d};  //帧尾
//时间
float timeDelay = 0;
float timeSec = 0;


float distFromLine(float, float);

void goToEnd();

void followWall();

void recv();

void pub(float, float);

void find_wall();

void go_left();

void follow_wall();

void correct_heading();

void go_straight();

void done();


void setup() {
    // put your setup code here, to run once:
    Serial.begin(2000000);
    Serial1.begin(2000000);
}

void loop() {
    //Serial.println(millis());

    recv();

    distToLine = distFromLine(x, y);

    if (robotState == 0) {
        goToEnd();
        Serial1.println("gotoend");
        if (dCenter < OBST_THRESHOLD && dCenter > 0.03) {                     
            timeDelay = 0;                        //沿着墙走了之后就重新计算时间
            timeSec = 0;
            robotState = 1;
        }
    } else if (robotState == 1) {
        followWall();
        Serial1.println(distToLine);
        Serial1.println(timeSec);
        if (timeSec > 5 && distToLine < 0.1)  //沿着墙走了10s后
            robotState = 0;
    }
    timeDelay = timeDelay + 1;
    if (timeDelay == 30) {   //timeSec记录秒数，20次timeDelay就是1秒
        timeSec = timeSec + 1;
        timeDelay = 0;
    }


    delay(1000 / LOOP_RATE);
}


float distFromLine(float x, float y) {
    float m = (yFinal - yStart) / (xFinal - xStart);
    float numerator = fabs(y - yStart - m * x + m * xStart);
    float denominator = (sqrt(1 + m * m));
    float dis = numerator / denominator;
    return dis;
}

void goToEnd() {
    if (runState == State::turn) {
        correct_heading();
        Serial1.println("turnning");
    } else if (runState == State::go) {
        go_straight();
        Serial1.println("going");
    } else if (runState == State::done) {
        done();
        Serial1.println("done");
    }
}

void followWall() {
  //1
  if(dRight>OBST_THRESHOLD && dCenter>OBST_THRESHOLD && dLeft>OBST_THRESHOLD){
    find_wall();
  }
  //2
  else if(dRight<OBST_THRESHOLD && dCenter>OBST_THRESHOLD && dLeft>OBST_THRESHOLD){
    follow_wall();
  }
  //3
  else if(dRight>OBST_THRESHOLD && dCenter<OBST_THRESHOLD && dLeft>OBST_THRESHOLD){
    go_left();
  }
  //4
  else if(dRight>OBST_THRESHOLD && dCenter>OBST_THRESHOLD && dLeft<OBST_THRESHOLD){
    go_left();
  }
  //5
  else if(dRight<OBST_THRESHOLD && dCenter<OBST_THRESHOLD && dLeft>OBST_THRESHOLD){
    go_left();
  }
  //6
  else if(dRight>OBST_THRESHOLD && dCenter<OBST_THRESHOLD && dLeft<OBST_THRESHOLD){
    go_left();
  }
  //7
  else if(dRight<OBST_THRESHOLD && dCenter>OBST_THRESHOLD && dLeft<OBST_THRESHOLD){
    find_wall();
  }
}

void recv() {
    if (Serial.available() != 32) return;

    Serial.readBytes(frame, 32);

    //终点坐标
    xFinal = *(float *) ((frame));
    yFinal = *(float *) ((frame + 4));

    //机器人位姿
    x = *(float *) ((frame + 8));
    y = *(float *) ((frame + 12));
    yaw = *(float *) ((frame + 16));

    //传感器数据
    dCenter = *(float *) ((frame + 20));
    dLeft = *(float *) ((frame + 24));
    dRight = *(float *) ((frame + 28));
}

//发送角速度和线速度
void pub(float omega, float vel) {
    Serial.write((char *) &vel, 4);
    Serial.write((char *) &omega, 4);
    Serial.write(frameEnd, 2);
}

//控制机器人运动
void find_wall() {
    Serial1.println("find_wall");
    pub(- MAX_ANGLE_VEL * 4, MAX_LINEAR_VEL);
}

void go_left() {
    Serial1.println("go_left");
    pub(MAX_ANGLE_VEL * 4, 0);
}

void follow_wall() {
    Serial1.println("follow_wall");
    pub(0, MAX_LINEAR_VEL);
}

void correct_heading() {
    float dYaw = atan2f(yFinal - y, xFinal - x);
    float angleOff = fmod(dYaw - yaw, 2 * PI);

    if (angleOff > PI) {
        angleOff = angleOff - 2 * PI;
    }

    if (fabs(angleOff) > YAW_THRESHOLD) {
        if (angleOff > 0) {
            pub(MAX_ANGLE_VEL, 0.0);
        } else {
            pub(-MAX_ANGLE_VEL, 0.0);
        }
    } else {
        done();
        runState = State::go;
    }
}

void go_straight() {
    float distOff = sqrtf(powf((yFinal - y), 2) + powf((xFinal - x), 2));
    float dYaw = atan2f(yFinal - y, xFinal - x);
    float angleOff = fmod(dYaw - yaw, 2 * PI);
    if (distOff > DIS_THRESHOLD) {
        pub(0, MAX_LINEAR_VEL);
    } else {
        runState = State::done;
        //结束
    }
    if (fabs(angleOff) > YAW_THRESHOLD) {
        runState = State::turn;
    }
}

void done() {
    pub(0.0, 0.0);
}