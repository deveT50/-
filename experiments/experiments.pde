
import java.util.Map;
import java.io.BufferedWriter;

import java.io.FileWriter;

import java.io.File;
import java.io.IOException;
BufferedWriter output = null;
BufferedReader reader;
String line;
String line2;
int exeNum=0;//実験実行回数
int exeScheduledNum=30;//実験予定回数
int exeInit=0;//実験準備完了＝１
int kk=1;
int no=0;
//https://forum.processing.org/two/discussion/561/easiest-way-to-append-to-a-file-in-processing
int robotVel=6;

boolean[] keyState=new boolean[256];
int[] keyState2=new int[256];
int screenX=960+200  ;
int screenY=640;//+100pxしてある
int objectSize=32;//robotや障害物のサイズ
int frame=30;  //フレームレート
int measurePosX=960;//測定表示用のエリア
PrintWriter fileWriter;  //ファイル出力


//画像ハンドル
PImage graph_robo;
PImage graph_obsB;
PImage graph_obsR;
PImage graph_obsG;

//障害物の数
int obstacleNum;
//残り数
int obstacleRestNum;
//robotの数
int robotNum=1;




//半加算器クラス(アウトプット)
class halfAdderOutput {
  int s;
  int c;
}

//ポイント型クラス
class Point {
  int x;
  int y;
}



//測定実験クラス
class Measure {

  int experiment;//０で実験の終了
  int timeCount;
  float time;
  int elaspedTime;//経過時間の結果保持
  int remainedTime;//残り時間の結果保持
  float calcedSatisfy;//時間に対する満足度の結果保持
  int limitCounter;
  int limitTime;//制限時間（秒）5分=３００秒

  float frameMin;//フレームレート最小値
  float frameMax;//最大値
  
  
  //コンストラクタ
  Measure() {
    experiment=1;
    timeCount=0;
    if(robotNum!=0)limitTime=420/robotNum;
    limitCounter=frame*limitTime;//3000;//frame*limitTime
    //9000フレームに固定
    frameMin=30.0f;
    frameMin=30.0f;
    
    remainedTime=0;
    elaspedTime=0;
  }

  //残り時間(秒)を測る
  float timer() {
    if (obstacleRestNum>0) {
      limitCounter--;
    }
    this.remainedTime=(limitCounter)/frame;

    return  this.remainedTime;
  }
  //経過時間を測る
  float passTime() {
    if (obstacleRestNum>0) {

      timeCount++;
    }
    this.elaspedTime=(timeCount)/frame;
    return this.elaspedTime;
  }

  //満足度の時間に対する割合を測る(一つのパーツに対して)
  float satisfyRate(int sum, int timeCountNum) {
    this.calcedSatisfy=sum/timeCountNum;
    return this.calcedSatisfy;
  }

  //実験終了
  int isExperimentOver() {
    if ((this.elaspedTime==limitTime&&this.remainedTime==0)||obstacleRestNum==0) {
      this.experiment=0;
      return 1;
    }
    return 0;
  }
}



//robotクラス
class Robot {
  //ナンバー
  int number;

  //電源
  int power;

  //測定対象(又は固有)パラメータ
  int x;//グローバルな位置
  int y;
  double theta;//ローカルな向き
  Point leftUpCorner;
  Point rightUpCorer;
  Point leftDownCorner;
  Point rightDownCorner;



  double dx;//ローカルな位置
  double dy;

  int energy;
  int energyMax;
  int energyShorted;
  int vel;//速度


  int sensorPosX;//左右衝突用センサの位置
  int sensorPosY;
  int beamLength;   //センサの検出長さ
  int catchLength;
  int hold;//キャッチしているか：初期:-1、キャッチ:キャッチしたオブジェクト番号
  int nearest;//一番近いオブジェクトの番号



  //各センサーの値
  int senseEnergy;
  int senseEnergySupply;


  int senseAvoidL;
  int senseAvoidR;
  int senseAvoidBoth;
  int senseSerch;
  int senseCatch;
  int senseHold;
  int senseRelease;//catchの上位機能。catchしてない状態では発現しない.
  
  
  
  int measure_isRefueled;
  int measure_refuelNum;
  //測定用
  int measure_satisfySum;
  int measure_scoreNum;
  float measure_satisfyRate;

  Map<String, Integer> measure_satisfy=new HashMap<String, Integer>();
  Map<String, Integer> measure_discontent=new HashMap<String, Integer>();


  //各機能の半加算器の計算結果のマップ
  Map<String, halfAdderOutput> funcs=new HashMap<String, halfAdderOutput>();
  //各センサーの値マップ
  Map<String, Integer> sensors=new HashMap<String, Integer>();
  //理想状態のマップ
  Map<String, Integer> ideal=new HashMap<String, Integer>();

  //エネルギー消費先判定用名前
  KEYNAME keyName;


  //コンストラクタ
  Robot() {

    this.power=0;
    this.x=screenX/2;
    this.y=screenY/2;
    this.energyMax=floor(30000/robotNum);
    this.energy=this.energyMax;
    this.energyShorted=floor(this.energyMax/2);
    this.dx=0;
    this.dy=0;

    leftUpCorner=new Point();
    rightUpCorer=new Point();
    leftDownCorner=new Point();
    rightDownCorner=new Point();

    this.measure_isRefueled=0;
    
    this.leftUpCorner.x=this.x-objectSize/2;
    this.leftUpCorner.y=this.y-objectSize/2;

    this.rightUpCorer.x=this.x+objectSize/2;
    this.rightUpCorer.y=this.y-objectSize/2;


    this.leftDownCorner.x=this.x-objectSize/2;
    this.leftDownCorner.y=this.y+objectSize/2;

    this.rightDownCorner.x=this.x+objectSize/2;
    this.rightDownCorner.y=this.x+objectSize/2;





    this.theta=-90;
    this.vel=robotVel;

    this.beamLength=30;
    this.catchLength=50;
    this.hold=-1;
    this.nearest=-1;

    //センサの値:初期化用
    this.senseEnergy=1;
    this.senseEnergySupply=-1;//起動していない
    this.senseAvoidL=0;
    this.senseAvoidR=0;
    this.senseAvoidBoth=0;
    this.senseSerch=0;
    this.senseCatch=-1;
    this.senseHold=0;
    this.senseRelease=-1;//起動していない


    measure_refuelNum=0;
    measure_scoreNum=0;
    measure_satisfyRate=0;
    measure_satisfySum=0;




    //半加算器.c==1のカウント数
    this.measure_satisfy.put("energy", 0);
    this.measure_satisfy.put("energySupply", 0);
    this.measure_satisfy.put("avoidL", 0);
    this.measure_satisfy.put("avoidR", 0);
    this.measure_satisfy.put("avoidBoth", 0);
    this.measure_satisfy.put("search", 0);
    this.measure_satisfy.put("taskHold", 0);
    this.measure_satisfy.put("taskCatch", 0);
    this.measure_satisfy.put("taskRelease", 0);

    //半加算器.s==1のカウント数
    this.measure_discontent.put("energy", 0);
    this.measure_discontent.put("energySupply", 0);
    this.measure_discontent.put("avoidL", 0);
    this.measure_discontent.put("avoidR", 0);
    this.measure_discontent.put("avoidBoth", 0);
    this.measure_discontent.put("search", 0);
    this.measure_discontent.put("taskHold", 0);
    this.measure_discontent.put("taskCatch", 0);
    this.measure_discontent.put("taskRelease", 0);




    //robotの持つセンサー状態を持っておくマップの初期化
    this.sensors.put("energy", this.senseEnergy);
    this.sensors.put("energySupply", this.senseEnergySupply);
    this.sensors.put("avoidL", this.senseAvoidL);
    this.sensors.put("avoidR", this.senseAvoidR);
    this.sensors.put("avoidBoth", this.senseAvoidBoth);
    this.sensors.put("search", this.senseSerch);
    this.sensors.put("taskCatch", this.senseCatch);
    this.sensors.put("taskHold", this.senseHold);
    this.sensors.put("taskRelease", this.senseRelease);


    //理想状態を持っておくマップの初期化
    /*
    this.ideal.put("energy",1);
     this.ideal.put("avoidL",0);
     this.ideal.put("avoidR",0);
     this.ideal.put("avoidBoth",0);
     this.ideal.put("search",0);
     this.ideal.put("taskCatch",1);
     this.ideal.put("taskRelease",0);
     */
    this.ideal.put("energy", 1);  //エネルギーが残っていることが望ましい：１
    this.ideal.put("energySupply", 1);//エネルギー位置を発見していることが望ましい:1(初期値ー-1)

    this.ideal.put("avoidL", 1);//障害物がないことが望ましい：0//-------------!測定用に１に変更中
    this.ideal.put("avoidR", 1);//障害物がないことが望ましい：0//-------------!測定用に１に変更中
    this.ideal.put("avoidBoth", 1);//同時に起動しないことが望ましい:0//-------------!測定用に１に変更中
    this.ideal.put("search", 1);//オブジェクトが残っていないことが望ましい:0//-------------!測定用に１に変更中
    this.ideal.put("taskCatch", 1);//オブジェクトは発見していることが望ましい:1（初期値ー１）
    this.ideal.put("taskHold", 1);//はじめ、持っていない時は、持っていることが望ましい：１
    //ホールドした瞬間に、持っていないことが望ましくなる：０
    //ホールドした瞬間に、持っていることが望ましくなる:1

      this.ideal.put("taskRelease", 1);//リリース場所は発見していることが望ましい:1(初期値ー１)


    //robotの持つ機能の状態を持っておくマップの初期化
    this.funcs.put("energy", halfAdder(this.power, sensors.get("energy")));
    this.funcs.put("energySupply", halfAdder(this.power, sensors.get("energySupply")));

    this.funcs.put("avoidL", halfAdder(this.power, sensors.get("avoidL")));
    this.funcs.put("avoidR", halfAdder(this.power, sensors.get("avoidR")));
    this.funcs.put("avoidBoth", halfAdder(this.power, sensors.get("avoidBoth")));
    this.funcs.put("search", halfAdder(this.power, sensors.get("search")));
    this.funcs.put("taskCatch", halfAdder(this.power, sensors.get("taskCatch")));
    this.funcs.put("taskHold", halfAdder(this.power, sensors.get("taskHold")));

    this.funcs.put("taskRelease", halfAdder(this.power, sensors.get("taskRelease")));
  }



  //測定
  //半加算器のc==1の時間を測る:満足
  int measureSatisfy(String keyName) {
    int buf=this.measure_satisfy.get(keyName);

    if (this.funcs.get(keyName).c==1) {
      buf++;
    }
    this.measure_satisfy.put(keyName, buf);
    int result=this.measure_satisfy.get(keyName);
    return result;
  }


  //測定
  //半加算器のs==1の時間を測る:不満
  int measureDiscontent(String keyName) {
    int buf=this.measure_discontent.get(keyName);

    if (this.funcs.get(keyName).s==1) {
      buf++;
    }
    this.measure_discontent.put(keyName, buf);
    int result=this.measure_discontent.get(keyName);
    return result;
  }


  //測定
  //満足値を合計する
  void sumSatisfy() {
    int sum=0;
    for (String keyName : this.measure_satisfy.keySet ()) {
      sum+=this.measure_satisfy.get(keyName);
    }

    this.measure_satisfySum=sum;
  }








  //robotの電源オンオフ
  void power() {
    if (this.power==1)this.power=0;
    else this.power=1;
  }







  //エネルギー消費
  void consume(KEYNAME keyName, int intention) {
    switch(keyName) {

    case AVOIDL: 
      this.energy-=1*intention;
      break;
    case AVOIDR: 
      this.energy-=1*intention;
      break;
    case AVOIDBOTH: 
      this.energy-=1*intention;
      break;
    case SEARCH: 
      this.energy-=2*intention;
      break;
    case TASKCATCH: 
      this.energy-=3*intention;
      break;
    case TASKHOLD: 
      this.energy-=2*intention;
      break;
    case TASKRELEASE: 
      break;
    case ENERGYSUPPLY:  
      this.energy+=10*intention;
      break;
    }
  }


  //センサーの更新
  void updateSensors(Robot robot[], Obstacle obstacle[]) {
    this.sensors.put("energy", this.setSensorEnergy());
    this.sensors.put("energySupply", this.setSensorSupply());
    this.sensors.put("avoidL", this.setSensorAvoid(-45, robot, obstacle));
    this.sensors.put("avoidR", this.setSensorAvoid(45, robot, obstacle));
    this.sensors.put("avoidBoth", this.setSensorAvoidBoth(funcs.get("avoidL").s, funcs.get("avoidR").s));
    this.sensors.put("search", this.setSensorSearch(obstacle));
    this.sensors.put("taskCatch", this.setSensorCatch(obstacle));
    this.sensors.put("taskHold", this.setSensorHold());
    this.sensors.put("taskRelease", this.setSensorRelease(obstacle));
  }

  //角位置の更新
  void updateCorners() {
    this.leftUpCorner.x=this.x-objectSize/2;
    this.leftUpCorner.y=this.y-objectSize/2;

    this.rightUpCorer.x=this.x+objectSize/2;
    this.rightUpCorer.y=this.y-objectSize/2;

    this.leftDownCorner.x=this.x-objectSize/2;
    this.leftDownCorner.y=this.y+objectSize/2;

    this.rightDownCorner.x=this.x+objectSize/2;
    this.rightDownCorner.y=this.x+objectSize/2;
  }


  //エネルギーは残っていることが望ましい:ideal=1
  //エネルギーセンサー：
  int setSensorEnergy() {
    if (this.energy<this.energyShorted) {
      return not((this.ideal.get("energy")));
    }
    return this.ideal.get("energy");
  }

  //エネルギー補給位置は発見していることが望ましい:ideal=1
  //エネルギ―補給位置判別センサ
  int setSensorSupply() {
    if (this.funcs.get("energy").s==1) {//エネルギー残量について、意志を持ったとき(不満)、位置探索センサが起動(認知フレームを持つ)
      if (get(this.x, this.y)!=color(0, 0, 255)) {
        return  not(this.ideal.get("energySupply"));
      }
      return this.ideal.get("energySupply");
    }
    return -1;
  }


  //両方同時に動かないことが望ましい(このセンサが起動しないことが望ましい):ideal=0
  //障害物センサが左右両方同時に反応しないためのセンサ
  //両方動作していることを感知して、起動し、これを参照してブレーキが動作する
  int setSensorAvoidBoth(int left, int right) {

    if (left*right==1) {
      return not(this.ideal.get("avoidBoth"));
    }
    return this.ideal.get("avoidBoth");
  }





  //障害物がないことが望ましい:ideal=0
  //障害物センサ：：左右兼用関数
  int setSensorAvoid(int installedAngle, Robot robot[], Obstacle obstacle[] ) {
    //左右どちらのセンサか
    String str="";
    if (installedAngle<0) {
      str="avoidL";
    } else {
      str="avoidR";
    }

    //センサ設置位置設定
    this.sensorPosX=this.x+(int)(Math.cos(radians((float)this.theta+installedAngle))*22);
    this.sensorPosY=this.y+(int)(Math.sin(radians((float)this.theta+installedAngle))*22);


    //他のロボットとの接触判定
    for (int s=0; s<robotNum; s++) {
      if (s==this.number)continue;
      if (Math.sqrt(Math.pow(robot[s].x-this.sensorPosX, 2)+Math.pow(robot[s].y-this.sensorPosY, 2))<32) {
        return not((this.ideal.get(str)));
      }
    }


    //オブジェクトとの当り判定
    for (int i=0; i<obstacleNum; i++) {
      if (i==this.hold)continue;      
      if (Math.sqrt(Math.pow(this.sensorPosX-obstacle[i].x, 2)+Math.pow(this.sensorPosY-obstacle[i].y, 2))<22) {
        return not((this.ideal.get(str)));
      }
    }        

    //試験領域外にでるようなら
    if (Math.sqrt(Math.pow(this.sensorPosX-screenX/2, 2)+Math.pow((this.sensorPosY-screenY/2), 2))>screenY/2-this.beamLength) {
      return not((this.ideal.get(str)));
    }

    //何もなければ理想状態である
    return this.ideal.get(str);
  }



  //オブジェクトが残っていないことが望ましい：ideal=0;
  //サーチセンサ：：障害物が残っていれば１：探索を続ける
  int setSensorSearch(Obstacle obstacle[]) {
    int counter=0;
    for (int i=0; i<9; i++) {
      if (obstacle[i].carriedBy==-2) {
        counter++;
      }
    }
    obstacleRestNum=obstacleNum-counter;
    if (obstacleRestNum>0) {
      return not((this.ideal.get("search")));
    }
    return this.ideal.get("search");
  }

  //オブジェクトを持っていないならば
  //オブジェクトをホールドすることが望ましい：:ideal=1
  //ホールドしているのならば、ホールドしていないことが望ましい:ideal=0    
  //オブジェクトホールド状態インジケータ(理想状態を書き換えるので、理想状態と反対の値を返すと、矛盾する)

  //→理想状態の書き換えは、あまりうまくいかない。任意の理想状態を設定して、その状態との合致を判定して、差異から意志を発生させているので、
  //理想状態のほうを書き換えてしまうと、センサの条件分岐に従うと、差異があるのに意志が発生しないことになるため。

  //センサーとしての使用。半加算器としては、常にs=1になるため、不使用。持っている時は、持っていない時は、という理想状態を表すために残してある。
  int setSensorHold() {
    if (this.hold==-1) {
      //return not((this.ideal.get("taskHold"))); 
      return 0;
    }
    //return this.ideal.get("taskHold"); 
    return 1;
  }



  //ホールドしていない時に初めて起動する(フレームを持つ)
  //キャッチ自体は、キャッチすることが望ましい：ideal=1
  //タスクキャッチセンサ::ホールドしてないことによって起動、前方のオブジェクトによって発火
  int setSensorCatch(Obstacle obstacle[]) {


    
    //catchLengthを求める
    int catchX=this.x+(int)(Math.cos(radians((float)this.theta))*(32));//16+this.catchLength
    int catchY=this.y+(int)(Math.sin(radians((float)this.theta))*(32));
    this.nearest=-1;
    int cnt=0;

    //オブジェクトが目前にあるなら
    for (int i=0; i<obstacleNum; i++) {
      if ((catchX<obstacle[i].x+16)&&(catchX>obstacle[i].x-16)&&
        (catchY<obstacle[i].y+16)&&(catchY>obstacle[i].y-16)) {

        //そのオブジェクトが運ばれていないなら、運ぶ対象に設定。運びたい。（理想と違う）
        //運ばれているなら、別に運びたいと思わない（理想）
        if (obstacle[i].carriedBy==-1) {
          this.nearest=i;
          return not((this.ideal.get("taskCatch")));
        }else cnt++;
      }
      
    }
    if (cnt==obstacleNum) {
      //何もなければ、理想的である
      return (this.ideal.get("taskCatch"));
    }

    //}
    //目前にないなら
    return -1;
  }



  //ホールドした時に初めて起動する(フレームを持つ)
  //リリース地点は、発見していることが望ましい：ideal=1;
  //タスクリリースセンサ::キャッチ済で起動し、床の色によって発火::赤色検知センサ
  int setSensorRelease(Obstacle obstacle[]) {
 
    
    if (get(this.x, this.y)==color(255, 0, 0)) {
     
       //キャッチ済であれば
    if (this.sensors.get("taskHold")==1) {
     
        return  not(this.ideal.get("taskRelease"));
      }
      return this.ideal.get("taskRelease");
    //return -1;
    }
    return -1;
  }




  //エネルギーが減ったら起動
  void controllActuater(Obstacle obstacle[]) {
   
    //エネルギー状態が理想でなければ
    if (this.funcs.get("energySupply").c==1) { //エネルギー地点を発見(満足)していれば
      keyName=KEYNAME.ENERGYSUPPLY;
      consume(keyName, 1);
      this.energyShorted=this.energyMax;
      if(this.measure_isRefueled!=2){
        this.measure_isRefueled=1;
      }
      
      //エネルギー切れの場合停止する(便宜上ここに書いただけ）
    } else if (this.energy<0) {
      this.power=0;
      //エネルギー状態が理想的ならば通常動作
    } else {
      if(this.measure_isRefueled==1){
        this.measure_refuelNum++;
        this.measure_isRefueled=2;
      }
      if(get(this.x, this.y)!=color(0, 0,255))this.measure_isRefueled=0;
      this.searchObstacle();
      this.avoidL();
      this.avoidR();
      this.taskCatch(obs);
      this.taskRelease(obs); 
      this.energyShorted=this.energyMax/2;
    }
  } 



  //モーター
  //直進動作::タスクの探索の必要があるなら起動
  void searchObstacle() {

    //一ループごとに移動する量
    this.dx=round((float)Math.cos(radians((float)this.theta))*this.vel);
    this.dy=round((float)Math.sin(radians((float)this.theta))*this.vel);
    //意志があれば半加算器のsは１になるので、進む
    this.x+=this.funcs.get("search").s*this.dx;
    this.y+=this.funcs.get("search").s*this.dy;

    //エネルギー消費
    keyName=KEYNAME.SEARCH;
    consume(keyName, this.funcs.get("search").s);
  }



  //モーターL
  //左側に接触時のみ回転動作 
  void avoidL() {
    this.theta+=(this.funcs.get("avoidL").s)*10;
    //エネルギー消費
    keyName=KEYNAME.AVOIDL;
    consume(keyName, this.funcs.get("avoidL").s);
  }

  //モーターR
  //右側に接触時のみ回転動作 
  //ブレーキRを含む
  //左右モーターが同時に起動したときRのみ停止する
  void avoidR() {
    this.theta-=(this.funcs.get("avoidR").s)*(1-this.funcs.get("avoidBoth").s)*10;

    //エネルギー消費
    keyName=KEYNAME.AVOIDL;
    consume(keyName, this.funcs.get("avoidR").s);

    keyName=KEYNAME.AVOIDBOTH;
    consume(keyName, this.funcs.get("avoidBoth").s);
  }


  //モーターキャッチ
  //オブジェクト取得
  void taskCatch(Obstacle obstacle[]) {


    if (this.funcs.get("taskCatch").s==1) {
      //配列参照のための安全策
      if (this.nearest>-1) {
        obstacle[this.nearest].carriedBy=this.number;
        obstacle[this.nearest].catchedAngle=atan2((float)pow(this.y-obstacle[this.nearest].y, 2), (float)pow(this.x-obstacle[this.nearest].x, 2));
        this.hold=this.nearest;
        this.beamLength=15;//領域外の判定でまだ使っている
        this.vel=Math.round(robotVel/2);


        //オブジェクトを持った瞬間、持っていないことが望ましくなる（書き換え）
        this.ideal.put("taskHold", 0);
        //エネルギー消費
        keyName=KEYNAME.TASKCATCH;
        consume(keyName, 1);
      }
    }
  }

  //モーターホールド
  //オブジェクト保持しているなら、エネルギー消費
  void taskHold() {
    keyName=KEYNAME.TASKHOLD;
    consume(keyName, this.sensors.get("taskHold"));  //this.funcs.get("taskHold").s
  }



  //モーターリリース
  //オブジェクト放棄
  void taskRelease(Obstacle obstacle[]) {

    if (this.funcs.get("taskRelease").s==1) {
      //安全策
      if (this.hold!=-1) {
        //一度リリースしたものは拾わない
        obstacle[this.hold].carriedBy=-2;
        obstacleRestNum--;
        this.hold=-1;
        this.beamLength=30;
        this.vel=robotVel;
        //オブジェクトをはなした瞬間、持っていることが望ましくなる（書き換え）
        this.ideal.put("taskHold", 1);

        //成果
        this.measure_scoreNum+=1;
      }
    }
  }
}

//運ぶべきオブジェクト
class Obstacle {

  int x;
  int y;
  float dx;
  float dy;
  int carriedBy;//誰かに運ばれているか、だれに運ばれているか
  int number;
  float pushedAngle;
  float catchedAngle;//キャッチされた瞬間のロボットとthisのアングル

  int isDrawn;  //描画済みかどうか

  //当り判定用：角の位置
  Point leftUpCorner;
  Point rightUpCorner;
  Point leftDownCorner;
  Point rightDownCorner;


  //コンストラクタ
  Obstacle() {

    x=int((screenY-200)/random(5));
    y=int((screenY-200)/random(5));
    carriedBy=-1;
    pushedAngle=0;
    catchedAngle=0;  
    isDrawn=0; 

    leftUpCorner=new Point();
    rightUpCorner=new Point();
    leftDownCorner=new Point();
    rightDownCorner=new Point();


    this.leftUpCorner.x=this.x-objectSize/2;
    this.leftUpCorner.y=this.y-objectSize/2;

    this.rightUpCorner.x=this.x+objectSize/2;
    this.rightUpCorner.y=this.y-objectSize/2;

    this.leftDownCorner.x=this.x-objectSize/2;
    this.leftDownCorner.y=this.y+objectSize/2;

    this.rightDownCorner.x=this.x+objectSize/2;
    this.rightDownCorner.y=this.x+objectSize/2;
  }

  //robotに押される、又はオブジェクトを押す関数
  void moved(Robot[] robot, Obstacle[]obstacle) {

    for (int s=0; s<robotNum; s++) {
      this.pushedAngle=0;
      //robotに運ばれている時は、ロボットの位置に従う  
      if (this.carriedBy==s&&robot[s].hold==this.number) {
        this.x=robot[s].x+round((float)Math.cos(radians((float)robot[s].theta)))*32;
        this.y=robot[s].y+round((float)Math.sin(radians((float)robot[s].theta)))*32;
        //robotに運ばれていないなら、robotによって押される
      } else if (Math.sqrt(Math.pow(robot[s].x-this.x, 2)+Math.pow(robot[s].y-this.y, 2))<30) {
        //運ばれていて、ぶつかってきたら、運んでいるロボットの角度を変える
        if (this.carriedBy>0)robot[this.carriedBy].theta+=robot[s].theta/2;
        this.x+=round((float)Math.cos(radians((float)Math.atan2((float)Math.pow(robot[s].y-this.y, 2), (float)Math.pow(robot[s].x-this.x, 2))))*Math.round(r[0].vel/2));
        this.y+=round((float)Math.sin(radians((float)Math.atan2((float)Math.pow(robot[s].y-this.y, 2), (float)Math.pow(robot[s].x-this.x, 2))))*Math.round(r[0].vel/2));
        this.pushedAngle=(float)robot[s].theta;
      }



      //他のオブジェクトを押す
      for (int i=0; i<obstacleNum; i++) {
        if (i==this.number||this.carriedBy<0)continue;
        if (obstacle[i].carriedBy<0) {
          if (Math.sqrt(Math.pow(obstacle[i].x-this.x, 2)+Math.pow(obstacle[i].y-this.y, 2))<30) {
            int xx=round((float)Math.cos(radians((float)Math.atan2((float)Math.pow(obstacle[i].y-this.y, 2), (float)Math.pow(obstacle[i].x-this.x, 2))-this.pushedAngle))*(r[s].vel));
            int yy=round((float)Math.sin(radians((float)Math.atan2((float)Math.pow(obstacle[i].y-this.y, 2), (float)Math.pow(obstacle[i].x-this.x, 2))-this.pushedAngle))*(this.y-obstacle[i].y));
             if(Math.sqrt(Math.pow(obstacle[i].x-xx, 2)+Math.pow(obstacle[i].y+yy, 2))>200){
                obstacle[i].x-=xx;
                obstacle[i].y+=yy;
                obstacle[i].pushedAngle=this.pushedAngle;
           
            }
          
          }
        }
      }
    }
  }
}




//半加算器
halfAdderOutput halfAdder(int ideal, int state) {

  halfAdderOutput output =  new halfAdderOutput();
  if (ideal!=-1&&state!=-1) {
    output.s=ideal ^ state;
    output.c=ideal * state;
  } else {
    output.s=-1;
    output.c=-1;
  }
  return output;
}







//NOT素子
int not(int input) {

  switch(input) {
  case 0:
    return 1;
  case 1:
    return 0;  
  default: 
    return -1;
  }
}

//xor素子
int xor(int a, int b) {
  if (a==b)return 1;
  else return 0;
}



//---------------------------------------------------------
// FPS描画
//---------------------------------------------------------
// int TEXT_SIZE テキストサイズ
// color TEXT_COLOR_G テキストカラー 緑
// color TEXT_COLOR_Y テキストカラー 黄
// color TEXT_COLOR_R テキストカラー 赤
// void Update() 更新
// void Drawing(float x, float y) 描画
/*
  使うときはFPS FPS = new FPS(FRAME_RATE);
 ゲームのフレームレートを渡してインストラクタ呼んでくださいな。
 Update()は毎フレーム呼んで下さい。
 Drawing(float x, float y)のx, yはFPSの描画座標です。
 */



class FPS
{
  int TEXT_SIZE = 24;
  color TEXT_COLOR_G = color(0, 255, 0);
  color TEXT_COLOR_Y = color(255, 255, 0);
  color TEXT_COLOR_R = color(255, 0, 0);

  int FRAME_RATE;
  float FPS, sumFPS;
  int cntFPS;
  FPS(int _FRAME_RATE) {
    FRAME_RATE = _FRAME_RATE;
    FPS = _FRAME_RATE;
  }
  void Update() {
    if (cntFPS < FRAME_RATE) {
      sumFPS += frameRate;
      cntFPS++;
    } else {
      FPS = round(sumFPS / FRAME_RATE * 10) / 10.0;
      sumFPS = cntFPS = 0;
    }
  }
  void Drawing(float x, float y) {
    if (FPS < FRAME_RATE / 2) {
      fill(TEXT_COLOR_R);
    } else if (FPS < FRAME_RATE / 3 * 2) {
      fill(TEXT_COLOR_Y);
    } else {
      fill(TEXT_COLOR_G);
    }
    //textSize(TEXT_SIZE);
    text("" + FPS, x, y);
    if(measure.timeCount>90){
      if(measure.frameMin>FPS){
        measure.frameMin=FPS;
      }else if(measure.frameMax<FPS){
        measure.frameMax=FPS;
      }
    }
  }
}



Robot[] r=new Robot[9];
Obstacle[] obs=new Obstacle[9];
Measure measure=new Measure();
FPS fps=new FPS(frame);


//setup内本体
void prepare(){

  //キー状態の初期化
  for (int i=0; i<256; i++) {
    keyState[i]=false;
    keyState2[i]=0;
  }

  //robotNum=5;
  obstacleNum=9;
  obstacleRestNum=9;
    robotNum=floor(random(5));
    r=new Robot[9];
    obs=new Obstacle[9];
    measure=new Measure();
    fps=new FPS(frame);

  Point[] array=new Point[obstacleNum];
  
  for (int i=0; i<obstacleNum; i++) {
    array[i]=new Point();
    array[i].x=0;
    array[i].y=0;
    
  }
  
  for (int i=0; i<robotNum; i++) {

    r[i]=new Robot();
    r[i].number=i;
    r[i].x=screenX/2-screenY/2+(screenY/(robotNum+1))*(i+1);//Y座標中央、X座標：ロボットの数等分した位置
    r[i].y=screenY/2;
  }


int flag=0;
  for (int i=0; i<obstacleNum; i++) {

    obs[i]=new Obstacle();
    obs[i].number=i;
   
     //obs[i].x=screenX/2+i*30;
    //obs[i].y=screenY/4+i*30;
    if(i>0){
       //obs[i].x=screenX/2+(int)random((-screenY+32)/2,(screenY-32)/2);screenY/2-64
       //obs[i].y=screenY/2+(int)random((-screenY+32)/2,(screenY-32)/2);
      obs[i].x=screenX/2+floor((200)*cos(radians(random(360))));
      obs[i].y=screenY/2+floor((200)*sin(radians(random(360))));
      //array[i].x=obs[i].x;
      //array[i].y=obs[i].x;
      while(flag==1){
      flag=0;
       for(int s=0;s<obstacleNum;s++){
         if(abs(obs[i].x-obs[s].x)<32&&abs(obs[i].y-obs[s].y)<32){
           obs[i].x=screenX/2+floor((200)*cos(radians(random(360))));
           obs[i].y=screenY/2+floor((200)*sin(radians(random(360))));
           flag=1;
         
         }
       
       }
      }
     
    }else{
           obs[i].x=screenX/2+floor((200)*cos(radians(random(360))));
           obs[i].y=screenY/2+floor((200)*sin(radians(random(360))));
     
    }
  
  }

  graph_robo=loadImage("robo_rect.png");
  graph_obsB=loadImage("obsB.png");
  graph_obsR=loadImage("obsR.png");
  graph_obsG=loadImage("obsG.png");
  
  //ファイル作成
    
   
       line="";
       line2="";
        // text.textの中身を読み込む
        //reader = createReader("results.csv");
        String[] line3=loadStrings("results.csv"); 
        //line =line3;
        
        for(int u=0;u<line3.length;u++){
        line+=line3[u];
        
        
        
        }
       /* 
        if(line==null){
          line="";
        }
        */
        
        /*
        // IOException例外を投げる可能性があるので、try～catchを記述しておく
        while(true){
          kk++;
          try{
            // 一行ずつ読み込む
            line[kk] = reader.readLine();
          }catch (IOException e){
            line = null;
          }
          
          println(line);
          if(line==null)break;
        }
        
*/   
  


    exeInit=1;
 
}


//draw内実験本体
void execute(){
 KeyCount();
  background(255);

  //床の色
  for (int i=0; i<16; i++) {
    fill(255, 0, 0);
    noStroke();
    ellipse(i*10+screenX/2, i*10+screenY/2, 80, 80);
    fill(0, 0, 255);
    ellipse(-i*10+screenX/2, -i*10+screenY/2, 80, 80);
  }

  fill(0); 

 

  //０ボタンでスイッチオン
  if (keyState2['0']==1) {
    r[0].power();
  }
  if (keyState2['4']==1) {
    r[1].power();
  }
  if (keyState2['5']==1) {
    r[2].power();
  }






  //robotの分だけ描画
  for (int i=0; i<robotNum; i++) {

    //robotの電源が入っているなら
    if (r[i].power==1) {
      //各効果器が動作
      r[i].controllActuater(obs);
      //センサーの状態を更新
      r[i].updateSensors(r, obs);
      r[i].updateCorners();
      //エネルギーが切れていないなら、半加算器の計算結果を更新
      if (r[i].energy>0) {
        for (String keyName : r[i].funcs.keySet ()) {
          r[i].funcs.put(keyName, halfAdder(r[i].ideal.get(keyName), r[i].sensors.get(keyName)));
          //満足度生値の計測
          r[i].measureSatisfy(keyName);
          //不満足度
          r[i].measureDiscontent(keyName);
        }
      }
    }



  if(measure.timeCount<30){
     for(int e=0;e<robotNum;e++){
    r[e].measure_refuelNum=0;
    }
  
  }


    //robotの描画(ローカル座標)
    pushMatrix();
    translate(r[i].x, r[i].y);
    rotate(PI/180*(float)r[i].theta);
    image(graph_robo, (float)r[i].dx, (float)r[i].dy);
    popMatrix();


    //fill(100,255,100);
    text(i, r[i].x-24, r[i].y-24);
    fill(0);

    //触角の描画
    int xtest=r[i].x+(int)(Math.cos(radians((float)r[i].theta+45))*54);
    int ytest=r[i].y+(int)(Math.sin(radians((float)r[i].theta+45))*54);
    stroke(10);
    line(r[i].x, r[i].y, xtest, ytest);
    xtest=r[i].x+(int)(Math.cos(radians((float)r[i].theta-45))*54);
    ytest=r[i].y+(int)(Math.sin(radians((float)r[i].theta-45))*54);
    line(r[i].x, r[i].y, xtest, ytest);


    text("power="+r[i].power, 150, 10+i*200);
    text("restOfEnergy:"+r[i].energy, 230, 20+i*200);

    //半加算器の状態（XOR）
    text("energy="+r[i].funcs.get("energy").s, 150, 20+i*200);
    text("energySupply="+r[i].funcs.get("energySupply").s, 150, 30+i*200);
    text("search="+r[i].funcs.get("search").s, 150, 40+i*200);
    text("avoidL="+r[i].funcs.get("avoidL").s, 150, 50+i*200);
    text("avoidR="+r[i].funcs.get("avoidR").s, 150, 60+i*200);
    text("avoidBoth="+r[i].funcs.get("avoidBoth").s, 150, 70+i*200);
    text("taskHold="+r[i].funcs.get("taskHold").s, 150, 80+i*200);
    text("taskCatch="+r[i].funcs.get("taskCatch").s, 150, 90+i*200);
    text("taskRelease="+r[i].funcs.get("taskRelease").s, 150, 100+i*200);

   // text("taskHoldSensor="+r[i].sensors.get("taskHold"), 250, 80+i*200);//ひっかけるバグ対応
    text("refueledNum="+r[i].measure_refuelNum, 250, 30+i*200);

    //ラベル
    text("電源状態", 10, 10+i*200);
    text("エネ確保行動：残量", 10, 20+i*200);
    text("エネ補給位置探索", 10, 30+i*200);
    text("オブジェクト探索行動", 10, 40+i*200);
    text("回避行動L", 10, 50+i*200);
    text("回避行動R", 10, 60+i*200);
    text("回避行動Both", 10, 70+i*200);
    text("オブジェクト把持", 10, 80+i*200);
    text("オブジェクト取得", 10, 90+i*200);
    text("オブジェクト放出", 10, 100+i*200);
    text("オブジェクトリリース数", 10, 110+i*200);
    text("満足度/不満足度", 10, 120+i*200);

    fill(255, 100, 100);
    //各個体のスコア、満足度の生値 
    text(i+" measure_scoreNum "+r[i].measure_scoreNum, 150, 110+i*200);
    int ii=0;
    for (String keyName : r[i].measure_satisfy.keySet ()) {
      text(keyName+" :"+r[i].measure_discontent.get(keyName), 250, ii*10+120+i*200);
      text(keyName+" :"+r[i].measure_satisfy.get(keyName), 150, ii*10+120+i*200);
      text("満足/時間 :"+(double)r[i].measure_satisfy.get(keyName)/(double)measure.timeCount, measurePosX, ii*10+120+i*200);
      text("不満/時間 :"+(double)r[i].measure_discontent.get(keyName)/(double)measure.timeCount, measurePosX+100, ii*10+120+i*200);

      ii++;
    }


    fill(0);
  }

  //オブジェクトが押される
  for (int j=0; j<obstacleNum; j++) {
    obs[j].moved(r, obs);
    obs[j].isDrawn=0;
  }
  //オブジェクト描画
  for (int j=0; j<obstacleNum; j++) {
    //imageMode(CENTER);
    //はこばれているなら
    for (int i=0; i<robotNum; i++) {
      if (obs[j].carriedBy==i&&r[i].hold==j) {
        pushMatrix();  
        translate(r[i].x, r[i].y);
        rotate(PI/180*(float)r[i].theta);
        image(graph_obsR, cos(0)*32, sin(0)*32);  
        obs[j].isDrawn=1;
        obs[j].x+=r[i].dx;
        obs[j].y+=r[i].dy;
        popMatrix();
      }
    }
    if (obs[j].isDrawn==0) { 

      if (obs[j].carriedBy==-2) {
        image(graph_obsG, obs[j].x, obs[j].y);
        obs[j].isDrawn=1;
      } else {
        image(graph_obsB, obs[j].x, obs[j].y);
        obs[j].isDrawn=1;
      }
    }
  }




  if (keyState2['3']==1)r[0].theta+=90;

  fill(0);


  text("exeNum:"+exeNum+"回目", measurePosX, 10);
  text("exeSheduledNum:"+exeScheduledNum+"回予定", measurePosX, 20);

  text("elaspedTime: "+measure.elaspedTime+"秒", measurePosX, 30);
  text("remainedTime: "+measure.remainedTime+"秒", measurePosX, 40);
  text("timeCount :"+measure.timeCount+"フレーム", measurePosX, 50);
  


  fps.Update();
  fps.Drawing(measurePosX-40, 10);
  //textSize(12);

  //試験領域
  noFill();
  stroke(1);
  ellipse(screenX/2, screenY/2, screenY, screenY);
  
    //測定
  //時間切れ、又はタスク完了にて実験終了
  if (measure.isExperimentOver()==1) {
    
      

      //fileWriter=createWriter("results.csv");
         line+=(",\n\n");
         line+=(year()+"年"+month()+"月"+day()+"日"+"");
         line+=(hour()+"時"+minute()+"分"+second()+"秒"+",");
         line+=("timeCount,"+measure.timeCount+",");
         line+=("elaspedTime,"+measure.elaspedTime+",");
         line+=("remainedTime,"+measure.remainedTime+",");
         line+=("robotNum,"+robotNum+",");
         line+=("obstacleNum,"+obstacleNum+",");
         line+=("obstacleRestNum,"+obstacleRestNum+",");
         
         line+=("frameMin,"+measure.frameMin+",");
         line+=("frameMax,"+measure.frameMax+",");
         
         line+=("exeNum,"+exeNum+",");
         line+=("exeScheduledNum,"+exeScheduledNum+",");
        
         
         line+=(",\n\n");
      
       
       
        for (int i=0; i<robotNum; i++) {
        r[i].power=0;
        //line[kk]+=("robot"+i+"-----------"+",\n");
        //line[kk]+=("Satisfaction"+",\n");
        line+=("robot,"+i+",");
        line+=("score,"+r[i].measure_scoreNum+",");
        line+=("refueledNum"+','+r[i].measure_refuelNum+",");
        
        for (String keyName : r[i].funcs.keySet ()) {
            //満足
             line+=("sat"+keyName+","+r[i].measure_satisfy.get(keyName)+",");
              
        }
      
        for (String keyName : r[i].funcs.keySet ()) {
              //不満
              line+=("dis"+keyName+","+r[i].measure_discontent.get(keyName)+",");
              
        }
        /*
        for (String keyName : r[i].funcs.keySet ()) {
              //満足/時間
              line+=("sat/t"+keyName+','+(double)r[i].measure_satisfy.get(keyName)/(double)measure.timeCount+",");
    
       }
       for (String keyName : r[i].funcs.keySet ()) {
              //不満/時間
              line+=("dis/t"+keyName+','+(double)r[i].measure_discontent.get(keyName)/(double)measure.timeCount+",");
    
       }
        */
    }
    
      
    
    
    
   
     line+=("\n");
        
     line+=("\n\n");
   
     //experiment_scShot
     // if(no==0)save("experiment_scShot/result_"+year()+"年"+month()+"月"+day()+"日"+hour()+"時"+minute()+"分"+second()+"秒"+exeNum+".jpg");
       //  no++;
     //save("experiment_scShot/result_"+year()+"年"+month()+"月"+day()+"日"+hour()+"時"+minute()+"分"+second()+"秒"+exeNum+".jpg");
     
       
    // for (int l=0;l<kk+1;l++){
           println(line);
           //saveStrings("results.csv",line);
           fileWriter.println(line);
    // }
    // fileWriter.flush(); //残りを出力する
    // fileWriter.close(); // ファイルを閉じる
     
      
     //exit();
     exeNum++;
     exeInit=0;
     
  } else {
    measure.timer();
    measure.passTime();
    for (int i=0; i<robotNum; i++) {
      r[i].power=1;
    }
  }




}


//セットアップ
void setup() {
  size(screenX, screenY);
  frameRate(frame);
  smooth();
  imageMode(CENTER);

}
    

void draw() {
 
  if(exeInit==0){
    prepare();
  }
  
  if(exeNum<exeScheduledNum){
     execute();
 
  
  }
 
 
 
}






//押下キーの取得関数
void keyPressed() {
  if (key>=0&&key<256) {//ASCIIのとき
    keyState[key]=true;
  } else if (keyCode>=0&&keyCode<256) {//それ以外
    keyState[keyCode]=true;
  }
}
//放されたキーの取得関数
void keyReleased() {
  if (key>=0&&key<256) {
    keyState[key]=false;
  } else if (keyCode>=0&&keyCode<256) {
    keyState[keyCode]=false;
  }
}

//キーカウント関数
void KeyCount() {
  for (int i=0; i<256; i++) {
    if (keyState[i]==true) {
      keyState2[i]++;
    } else {
      keyState2[i]=0;
    }
  }
}

