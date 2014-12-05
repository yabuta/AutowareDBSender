/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
自分から認識した物体の種類と位置情報をデータベースに送信する

簡単な仕様：
１、pedestrian_pos_xyzとcar_pos_xyzから画面上のxy座標とdistanceを取得する
２、取得したxyとdistanceから上から見たxy座標を求める
３、このxy座標はカメラから見た座標なのでvelodyneからみ見た座標に変換する
４、これでvelodyneから座標が得られるのでこれを東西南北を軸とする直交座標に変換する
５、直交座標を緯度・経度に変換する
６、データベースサーバに対して1秒ごとに送信する

送信データのフォーマットは
緯度、経度、物体の種類、自動車のid

データは認識した物体ごとに送る

*/



#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <pthread.h>
#include <vector>
#include "SendData.h"
#include "calcoordinates.h"
#include "axialMove.h"
#include "geo_pos_conv.hh"

typedef struct _CARPOS{
  int x1;
  int y1;
  int x2;
  int y2;
  float distance;
}CARPOS;

pthread_mutex_t mutex;

selfLocation sl;

std::vector<CARPOS> global_cp_vector;


//wrap SendData class
void* wrapSender(void *tsd){


  //get values from sample_corner_point , convert latitude and longitude,
  //and send database server.
  std::vector<CARPOS> car_position_vector(global_cp_vector.size());
  std::vector<CARPOS>::iterator cp_iterator;

  //thread safe process for vector
  pthread_mutex_lock( &mutex );
  std::copy(global_cp_vector.begin(), global_cp_vector.end(), car_position_vector.begin());
  pthread_mutex_unlock( &mutex );
  cp_iterator = car_position_vector.begin();

  while(cp_iterator != car_position_vector.end()){

    //middle of right-lower and left-upper
    double U = (cp_iterator->x1 + cp_iterator->x2)/2;
    double V = (cp_iterator->y1 + cp_iterator->y2)/2;

    //convert
    sl.setOriginalValue(U,V,cp_iterator->distance);
    LOCATION ress = sl.cal();

    //get axial rotation and movement vector from camera to velodyne.    
    ANGLE angle;
    angle.thiX = 0;
    angle.thiY = 0;
    angle.thiZ = 0;
    MoveVector mvvector;
    mvvector.X = 0;
    mvvector.Y = 0;
    mvvector.Z = 0;

    axiMove am;
    LOCATION velocoordinate = am.cal(ress,angle,mvvector);


    /*
      I got my GPS location too.it`s my_xloc,my_yloc.
      and I convert to plane rectangular coordinate  from latitude and longitude.
     */

    /*
      our rectangular plane is 6 in Japan.
     */


    double lat_plane = 36;//136.906565;
    double lon_plane = 137.1;//35.180188;

    //sample Longitude and Latitude 3513.1345669,N,13658.9971525,E
    double my_xloc= 3513.1345669;
    double my_yloc= 13658.9971525;
    double my_zloc= 0;

    /*
      process of conversion to absolute coodinate from relative coordinates.
      yes, I did! next...
     */
    geo_pos_conv geo;
    geo.set_plane(7);
    geo.set_llh_nmea_degrees(my_xloc,my_yloc,my_zloc);

    printf("geo : %f\t%f\n",geo.x(),geo.y());

    LOCATION rescoord = am.cal(velocoordinate,angle);
    rescoord.X = geo.x();
    rescoord.Y = 0;
    rescoord.Z = geo.y();

    calcoordinates cc;
    RESULT res = cc.cal(rescoord.X,rescoord.Z,lat_plane,lon_plane);
    

    //I assume that values has 4 value ex: "0 0 0 0"   "1 2 3 4"
    //And if setting the other number of value , sendData will be failed.
    char values[100];
    sprintf(values,"%f %f 0 0",res.lat,res.log);
    char dbn[100] = "prius_data_store";
    char ct[100] = "latitude logitude type id";
    SendData sd;
    sd.setData(values);
    sd.setDBName(dbn);
    sd.setColumnType(ct);

    sd.Sender();

    cp_iterator++;

  }

}


void* intervalCall(void *a){

  pthread_t th;

  while(1){
    //create new thread for socket communication.      
    if(pthread_create(&th, NULL, wrapSender, NULL)){
      printf("thread create error\n");
    }
    sleep(1);
    if(pthread_join(th,NULL)){
      printf("thread join error.\n");
    }
  }

}


void car_position_xyzCallback(const sensors_fusion::CarPositionXYZ& car_position_xyz)
//void car_position_xyzCallback()
{
    int i;
    CARPOS cp;

    pthread_mutex_lock( &mutex );

    global_cp_vector.clear();

    //認識した車の数だけ繰り返す
    //    for (i = 0; i < car_position_xyz.car_num; i++){
    for (i = 0; i < 1; i++){
        //各認識した車へのデータのアクセス例
      /*
      cp.x1 = 10*i;
      cp.y1 = 20*i;
      cp.x2 = 30*i;
      cp.y2 = 40*i;
      cp.distance = 20;
      */
      car_position_xyz.corner_point.at(0+i*4);//x-axis of the upper left
       car_position_xyz.corner_point.at(1+i*4);//y-axis of the upper left
      car_position_xyz.corner_point.at(2+i*4);//x-axis of the lower right
      car_position_xyz.corner_point.at(3+i*4);//y-axis of the lower right
      car_position_xyz.distance.at(i);

      global_cp_vector.push_back(cp);
      
    }
    pthread_mutex_unlock( &mutex );


}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */

  ros::init(argc, argv, "database");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called Callback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */


  //car_pos_xyzトピックが更新されるとcar_position_xyzCallback関数が呼ばれる
  //第二引数の100は受信側のバッファであり、最新の100個を保持している。自由に変更可能です。
  //100個以上になると古いものから欠損します
  ros::Subscriber database = n.subscribe("car_pos_xyz", 100, car_position_xyzCallback);

  Mat intrinsic;
  std::string camera_yaml; 
  n.param<std::string>("/scan_to_image/camera_yaml", camera_yaml, STR(CAMERA_YAML)); 
  cv::FileStorage fs_auto_file(camera_yaml.c_str(), cv::FileStorage::READ); 

  fs_auto_file["intrinsic"] >> intrinsic; 
  fs_auto_file.release(); 

  /*
  double fkx = 1;
  double fky = 1;
  double Ox = 1;
  double Oy = 1;
  */

  intrinsic.at<float>(0,0);  
  intrinsic.at<float>(1,1); 
  intrinsic.at<float>(0,2);
  intrinsic.at<float>(1,2); 
  sl.setCameraParam(fkx,fky,Ox,Oy);

  //car_position_xyzCallback();

  pthread_t th;
  if(pthread_create(&th, NULL, intervalCall, NULL)){
    printf("thread create error\n");
  }

  pthread_join(th,NULL);


  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}

