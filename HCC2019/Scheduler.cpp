#include "Scheduler.h"
#include "fstream"

Scheduler::Scheduler(DataCenter &dc)
{
	num_CarsScheduling = dc.m_car_num;//鑾峰緱闇�瑕佽皟搴︾殑杞︽暟閲�
	num_Roads = dc.m_road_num;
	num_Crosses = dc.m_cross_num;
	num_Cars = dc.m_car_num;
	roads = dc.road;
	crosses = dc.cross;
	cars = dc.car;
	time_Scheduler = 0;//璋冨害鍣ㄥ垵濮嬫椂闂磋缃负0
	vexnum = dc.getCrossNum();
	edge = dc.getRoadNum();
	tmp = dc.getArc(); //寰楀埌閭绘帴鐭╅樀
	tmp1 = dc.getRoadvArc(); //寰楀埌閬撹矾闄愰�熼偦鎺ョ煩闃�
	//灏唃raphC2R澶у皬璁剧疆涓�36*36
	graphC2R = dc.graphC2R;
	num_CarsPut = 0;//宸茬粡鍙戣溅鐨勬暟閲忛缃负0
	graphRoadStatusByDS = dc.graphRoadStatusByDS;
	for (int i = 0; i < graphRoadStatusByDS.size(); i++)
	{
		for (int j = 0; j < graphRoadStatusByDS[0].size(); j++)
		{
			graphRoadStatusByDS[i][j] = 0;
		}
	}

}

Scheduler::~Scheduler()
{
}

int Scheduler::getSysTime()
{
	while (num_CarsScheduling > 0)
	{
		PRINT("***********time_Scheduler:%d************\n", time_Scheduler);//鎵撳嵃绯荤粺鏃堕棿

		/*绗竴姝ワ細鍏堝鐞嗘墍鏈夐亾璺笂鐨勮溅杈嗭紝杩涜閬嶅巻鎵弿*/
		driveAllCarsJustOnRoadToEndState();

		/*绗簩姝ワ細鍏堝鐞嗘墍鏈夐亾璺笂鐨勮溅杈嗭紝杩涜閬嶅巻鎵弿*/
		while (1)//缁堟鏉′欢涓猴細涓�涓惊鐜悗锛屾病鏈変换浣曡溅琚皟搴�
		{
			bool isWorkingCross = false;//鏍囧織鍙橀噺锛屽鏋滀竴涓惊鐜悗娌℃湁浠讳綍涓�杈嗚溅琚皟搴︼紝鍒欓��鍑哄惊鐜�
			for (int i = 0; i < num_Crosses; ++i)////鎸夌収鍗囧簭璋冨害鎵�鏈夎矾鍙�
			{
				int idCross = crosses[i].id;//鑾峰緱璺彛ID
				while (1)//寰幆璋冨害璺彛鍥涗釜鏂瑰悜鐨勮溅锛岀洿鍒板叏閮ㄨ溅杈嗗畬鎴愯皟搴︼紝鎴栬�呴樆濉�
				{
					bool isWorkingRoad = false;
					for (int j = 0; j < 4; ++j)//杩欓噷鎸夎姹傛槸鏍规嵁閬撹矾id杩涜鍗囧簭璋冨害
					{
						int idRoad = getFirstRoadFromCross(idCross, j);
						if (idRoad != -1)
						{
							int idStartLane = 0;//濡傛灉cross涓洪亾璺殑鍑烘柟鍚戯紝闇�瑕佽皟搴� 0 1 2杞﹂亾锛屽惁鍒欒皟搴� 3 4 5杞﹂亾
							if (roads[idRoad - 5000].idFrom == crosses[i].id)//濡傛灉cross涓洪亾璺殑鍏ユ柟鍚�
							{
								idStartLane = roads[idRoad - 5000].channel;
								if (roads[idRoad - 5000].isDuplex != 1)
									continue;//濡傛灉闈炲弻杞﹂亾锛岄��鍑烘湰娆″惊鐜�
							}
							while (1)
							{
								bool isWorkingLane = false;
								for (int m = idStartLane; m < idStartLane + roads[idRoad - 5000].channel; ++m)//閬嶅巻鎵�鏈塴ane
								{
									if (roads[idRoad - 5000].lane[m].laneCar.size() != 0)
									{
										Car car = roads[idRoad - 5000].lane[m].laneCar[0];
										if (car.status == WAITTING)//鍙鐞嗗湪璺彛涓斾负绛夊緟鐘舵�佺殑杞�
										{
											assert(car.status == WAITTING);//杞﹁締鍦ㄨ矾鍙ｈ皟搴︽椂涓�瀹氳鏄疻AITTING鐘舵��
											int dirConflict = 0;
											int dirTarget = 0;
											switch (roads[idRoad - 5000].lane[m].laneCar[0].dirCross)
											{
											case NONE:
												break;
											case DD://鐩磋>宸﹁浆>鍙宠浆
												dirTarget = getDirByRoadCrossDir(idCross,idRoad) + 2;//鐩爣鏂瑰悜
												if (dirTarget > 3) dirTarget -= 4;//淇鏂瑰悜
												if (isCanDriveToNextRoad(car, dirTarget, idCross))
												{
													isWorkingCross = true;
													isWorkingRoad = true;
													isWorkingLane = true;
													driveAllCarsJustOnOneChannelToEndState(idRoad, idCross,m);
												}
												//鍒ゆ柇杞叆鐨剅oad鏄惁鍙互琛岄┒

												break;
											case LEFT://宸﹁浆>鍙宠浆
												//鍒ゆ柇鍗冲皢杞叆鐨勬柟鍚戞槸鍚︽湁鐩磋杩涘叆鐨勮溅杈�
												dirConflict = getDirByRoadCrossDir(idCross, idRoad) - 1;//鍐茬獊鏂瑰悜
												if (dirConflict < 0) dirConflict += 4;//淇鏂瑰悜
												if (!isBeDD(crosses[i].roadID[dirConflict], idCross))
												{
													dirTarget = getDirByRoadCrossDir(idCross, idRoad) + 1;//鐩爣鏂瑰悜
													if (dirTarget > 3) dirTarget -= 4;//淇鏂瑰悜
													if (isCanDriveToNextRoad(car, dirTarget, idCross))//鍒ゆ柇杞叆鐨剅oad鏄惁鍙互琛岄┒
													{
														isWorkingCross = true;
														isWorkingRoad = true;
														isWorkingLane = true;
														driveAllCarsJustOnOneChannelToEndState(idRoad, idCross, m);
													}
												}
												break;
											case RIGHT://鍙宠浆浼樺厛绾ф渶浣�
												//鍒ゆ柇鍗冲皢杞叆鐨勬柟鍚戞槸鍚︽湁鐩磋杩涘叆鐨勮溅杈�
												dirConflict = getDirByRoadCrossDir(idCross, idRoad) + 1;//鍐茬獊鏂瑰悜
												if (dirConflict > 3) dirConflict -= 4;//淇鏂瑰悜
												if (!isBeDD(crosses[i].roadID[dirConflict], idCross))
												{
													dirConflict = getDirByRoadCrossDir(idCross, idRoad) + 2;//鍐茬獊鏂瑰悜
													if (dirConflict > 3) dirConflict -= 4;//淇鏂瑰悜
													//鍒ゆ柇鍗冲皢杞叆鐨勬柟鍚戞槸鍚︽湁宸﹁浆杩涘叆鐨勮溅杈�
													if (!isBeLEFT(crosses[i].roadID[dirConflict], idCross))
													{
														dirTarget = getDirByRoadCrossDir(idCross, idRoad) - 1;//鐩爣鏂瑰悜
														if (dirTarget < 0) dirTarget += 4;//淇鏂瑰悜
														if (isCanDriveToNextRoad(car, dirTarget, idCross))//鍒ゆ柇杞叆鐨剅oad鏄惁鍙互琛岄┒
														{
															isWorkingCross = true;
															isWorkingRoad = true;
															isWorkingLane = true;
															driveAllCarsJustOnOneChannelToEndState(idRoad, idCross, m);
														}
													}
												}
												break;
											default:
												break;
											}
										}
									}
								}
								if (!isWorkingLane)
									break;
							}
						}
					}
					if (!isWorkingRoad)
						break;
				}
			}
			if (!isWorkingCross)//濡傛灉涓�涓惊鐜悗娌℃湁浠讳綍涓�杈嗚溅琚皟搴︼紝鍒欓��鍑鸿皟搴﹀惊鐜�
				break;
		}
		driverCarInGarage();//杞﹀簱涓殑涓婅矾琛岄┒
		//putAllCarStatus();//杈撳嚭鎵�鏈夎溅鐨勭姸鎬�
		//putAllRoadStatus();
		time_Scheduler++;//鏇存柊璋冨害鍣ㄦ椂闂�
	}
	return time_Scheduler;
}

void Scheduler::getPathByScheduler()
{
	Graph_DG graph(vexnum, edge);
	graph.createArcGraph(tmp);
	graph.createArcRoadvGraph(tmp1);
	while (num_CarsScheduling > 0)
	{
		PRINT("***********time_Scheduler:%d************\n", time_Scheduler);//鎵撳嵃绯荤粺鏃堕棿

		/*绗竴姝ワ細鍏堝鐞嗘墍鏈夐亾璺笂鐨勮溅杈嗭紝杩涜閬嶅巻鎵弿*/
		driveAllCarsJustOnRoadToEndState();

		/*绗簩姝ワ細鍏堝鐞嗘墍鏈夐亾璺笂鐨勮溅杈嗭紝杩涜閬嶅巻鎵弿*/
		while (1)//缁堟鏉′欢涓猴細涓�涓惊鐜悗锛屾病鏈変换浣曡溅琚皟搴�
		{
			bool isWorkingCross = false;//鏍囧織鍙橀噺锛屽鏋滀竴涓惊鐜悗娌℃湁浠讳綍涓�杈嗚溅琚皟搴︼紝鍒欓��鍑哄惊鐜�
			for (int i = 0; i < num_Crosses; ++i)////鎸夌収鍗囧簭璋冨害鎵�鏈夎矾鍙�
			{
				int idCross = crosses[i].id;//鑾峰緱璺彛ID
				while (1)//寰幆璋冨害璺彛鍥涗釜鏂瑰悜鐨勮溅锛岀洿鍒板叏閮ㄨ溅杈嗗畬鎴愯皟搴︼紝鎴栬�呴樆濉�
				{
					bool isWorkingRoad = false;
					for (int j = 0; j < 4; ++j)//杩欓噷鎸夎姹傛槸鏍规嵁閬撹矾id杩涜鍗囧簭璋冨害
					{
						int idRoad = getFirstRoadFromCross(idCross, j);
						if (idRoad != -1)
						{
							int idStartLane = 0;//濡傛灉cross涓洪亾璺殑鍑烘柟鍚戯紝闇�瑕佽皟搴� 0 1 2杞﹂亾锛屽惁鍒欒皟搴� 3 4 5杞﹂亾
							if (roads[idRoad - 5000].idFrom == crosses[i].id)//濡傛灉cross涓洪亾璺殑鍏ユ柟鍚�
							{
								idStartLane = roads[idRoad - 5000].channel;
								if (roads[idRoad - 5000].isDuplex != 1)
									continue;//濡傛灉闈炲弻杞﹂亾锛岄��鍑烘湰娆″惊鐜�
							}
							while (1)
							{
								bool isWorkingLane = false;
								for (int m = idStartLane; m < idStartLane + roads[idRoad - 5000].channel; ++m)//閬嶅巻鎵�鏈塴ane
								{
									if (roads[idRoad - 5000].lane[m].laneCar.size() != 0)
									{
										Car car = roads[idRoad - 5000].lane[m].laneCar[0];
										if (car.status == WAITTING)//鍙鐞嗗湪璺彛涓斾负绛夊緟鐘舵�佺殑杞�
										{
											assert(car.status == WAITTING);//杞﹁締鍦ㄨ矾鍙ｈ皟搴︽椂涓�瀹氳鏄疻AITTING鐘舵��
											int dirConflict = 0;
											int dirTarget = 0;
											switch (roads[idRoad - 5000].lane[m].laneCar[0].dirCross)
											{
											case NONE:
												break;
											case DD://鐩磋>宸﹁浆>鍙宠浆
												dirTarget = getDirByRoadCrossDir(idCross, idRoad) + 2;//鐩爣鏂瑰悜
												if (dirTarget > 3) dirTarget -= 4;//淇鏂瑰悜
												if (isCanDriveToNextRoad(car, dirTarget, idCross))
												{
													isWorkingCross = true;
													isWorkingRoad = true;
													isWorkingLane = true;
													driveAllCarsJustOnOneChannelToEndState(idRoad, idCross, m);
												}
												//鍒ゆ柇杞叆鐨剅oad鏄惁鍙互琛岄┒

												break;
											case LEFT://宸﹁浆>鍙宠浆
												//鍒ゆ柇鍗冲皢杞叆鐨勬柟鍚戞槸鍚︽湁鐩磋杩涘叆鐨勮溅杈�
												dirConflict = getDirByRoadCrossDir(idCross, idRoad) - 1;//鍐茬獊鏂瑰悜
												if (dirConflict < 0) dirConflict += 4;//淇鏂瑰悜
												if (!isBeDD(crosses[i].roadID[dirConflict], idCross))
												{
													dirTarget = getDirByRoadCrossDir(idCross, idRoad) + 1;//鐩爣鏂瑰悜
													if (dirTarget > 3) dirTarget -= 4;//淇鏂瑰悜
													if (isCanDriveToNextRoad(car, dirTarget, idCross))//鍒ゆ柇杞叆鐨剅oad鏄惁鍙互琛岄┒
													{
														isWorkingCross = true;
														isWorkingRoad = true;
														isWorkingLane = true;
														driveAllCarsJustOnOneChannelToEndState(idRoad, idCross, m);
													}
												}
												break;
											case RIGHT://鍙宠浆浼樺厛绾ф渶浣�
												//鍒ゆ柇鍗冲皢杞叆鐨勬柟鍚戞槸鍚︽湁鐩磋杩涘叆鐨勮溅杈�
												dirConflict = getDirByRoadCrossDir(idCross, idRoad) + 1;//鍐茬獊鏂瑰悜
												if (dirConflict > 3) dirConflict -= 4;//淇鏂瑰悜
												if (!isBeDD(crosses[i].roadID[dirConflict], idCross))
												{
													dirConflict = getDirByRoadCrossDir(idCross, idRoad) + 2;//鍐茬獊鏂瑰悜
													if (dirConflict > 3) dirConflict -= 4;//淇鏂瑰悜
													//鍒ゆ柇鍗冲皢杞叆鐨勬柟鍚戞槸鍚︽湁宸﹁浆杩涘叆鐨勮溅杈�
													if (!isBeLEFT(crosses[i].roadID[dirConflict], idCross))
													{
														dirTarget = getDirByRoadCrossDir(idCross, idRoad) - 1;//鐩爣鏂瑰悜
														if (dirTarget < 0) dirTarget += 4;//淇鏂瑰悜
														if (isCanDriveToNextRoad(car, dirTarget, idCross))//鍒ゆ柇杞叆鐨剅oad鏄惁鍙互琛岄┒
														{
															isWorkingCross = true;
															isWorkingRoad = true;
															isWorkingLane = true;
															driveAllCarsJustOnOneChannelToEndState(idRoad, idCross, m);
														}
													}
												}
												break;
											default:
												break;
											}
										}
									}
								}
								if (!isWorkingLane)
									break;
							}
						}
					}
					if (!isWorkingRoad)
						break;
				}
			}
			if (!isWorkingCross)//濡傛灉涓�涓惊鐜悗娌℃湁浠讳綍涓�杈嗚溅琚皟搴︼紝鍒欓��鍑鸿皟搴﹀惊鐜�
				break;
		}

		driverCarInGarageDynamic(graph);
		//putAllCarStatus();//杈撳嚭鎵�鏈夎溅鐨勭姸鎬�
		//putAllRoadStatus();
		time_Scheduler++;//鏇存柊璋冨害鍣ㄦ椂闂�
		if (time_Scheduler % 2 == 0)//姣�50涓椂闂寸墖鏇存柊涓�娆¤矾寰�
		{
			putAllRoadStatus();
		}
	}
}

void Scheduler::driveAllCarsJustOnRoadToEndState()
{
	for (int i = 0; i < num_Roads; ++i)//鎸夐亾璺疘D鍗囧簭杩涜璋冨害
	{
		for (int j = 0; j < roads[i].channel * (1 + roads[i].isDuplex); ++j)
		{
			if (roads[i].lane[j].laneCar.size() != 0)//鍏堝垽鏂杞﹂亾鏄惁鏈夎溅
			{
				for (int m = 0; m < roads[i].lane[j].laneCar.size(); ++m)
				{
					if (roads[i].lane[j].laneCar[m].dirCross == NONE)
					{
						int result = driveCar(roads[i].lane[j].laneCar[m], m);//浠庣涓�杈嗚溅寮�濮嬪線鍚庤皟搴�
						if (result == 2)
							m--;
					}
					//roads[i].lane[j] = lane;//杩欓噷涓�瀹氳灏唋ane鍐欏洖锛屽惁鍒欒溅杈嗚皟搴︽棤鏁�
					//杩欓噷涓嶈鍐欏洖锛屽啓鍥炲鏄撹鎿嶄綔锛屽彧闇�瑕佷繚璇佹洿鏂癱ar鐨勭姸鎬佹槸閫氳繃road->lane->car鐨勬柟寮忕储寮曞嵆鍙�
					//姝ゅ鏈夐棶棰橈紝濡傛灉涓�涓溅閬撴湁涓よ締杞︼紝浣嗘槸绗竴杈嗚溅琚紑鍒颁笅涓�杞﹂亾锛屽鏋滆溅閬撲俊鎭病鏈夊強鏃舵洿鏂�
					//浼氬嚭鐜拌鎿嶄綔锛岀储寮曞埌骞朵笉瀛樺湪鐨勭浜岃締杞�
					//0320濡傛灉鍓嶈溅鍒拌揪缁堢偣锛岃鍒犻櫎锛岀储寮曠殑鏃跺�欎細閬楁紡涓�杈嗚溅
				}
			}
		}
	}
}

int Scheduler::driveCar(Car car, int indexCar)
{
	//杩欓噷鍋囪AA锛氬鏋滄煇杈嗚溅浠庤矾鍙ｉ┒鍏ヤ笅涓�閬撹矾锛屼笉鍙兘鍦ㄤ竴涓椂闂寸墖鍐呴┒瀹屼笅涓�閬撹矾鍏ㄧ▼
	//涔熷氨鏄鍙湁澶勪簬NONE鐘舵�乄AITTING鐨勮溅鎵嶆湁鍙兘鍗冲皢鍒拌揪缁堢偣
	if (car.dirCross == NONE)//璇ヨ溅涓嶆槸鍦ㄨ矾鍙ｇ瓑寰�
	{
		assert(indexCar != -1);//姝ゆ儏鍐典笅indexCar涓嶈兘涓�-1
		assert(roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.size()!=0);//鏂█璇ヨ溅閬撲笂鑷冲皯鏈夎嚜宸变竴杈嗚溅
		assert(indexCar < roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.size());//鏂█杞︾殑搴忓彿灏忎簬杞﹂亾涓婅溅鏁伴噺
		if (indexCar == 0 || roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.size()== 1)//璇ヨ溅涓鸿杞﹂亾鐨勭涓�杈嗚溅锛屼笖涓婁釜鏃堕棿鐗囦笉鍑嗗閫氳繃璺彛
		{
			//鍒ゆ柇姝よ溅浼氫笉浼氶�氳繃璺彛
			if (car.location + std::min(roads[car.idCurRoad - 5000].speed, car.speed) <= roads[car.idCurRoad - 5000].length)//涓嶄細椹跺嚭璺彛
			{
				roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].location += std::min(roads[car.idCurRoad - 5000].speed, car.speed);//杞︽甯歌椹�
				roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].status = FINESHED;//杞︽爣璁颁负缁堟鐘舵��
				roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].dirCross = NONE;
			}
			else
			{
				//姝よ溅涔熷皢琛岄┒鍑鸿矾鍙ｏ紝闇�瑕佸垽鏂杞﹀湪璺彛鐨勬柟鍚�
				//鍒ゆ柇杞︾殑鏂瑰悜
				int idNextCross = 0;
				if(car.idCurLane >= roads[car.idCurRoad - 5000].channel)//閫嗗悜
					idNextCross = roads[car.idCurRoad - 5000].idFrom;//姝よ溅鍗冲皢椹跺叆鐨勮矾鍙�
				else
					idNextCross = roads[car.idCurRoad - 5000].idTo;//姝よ溅鍗冲皢椹跺叆鐨勮矾鍙�
				//鏍规嵁鍋囪AA锛屾鏃跺彲鑳芥湁杞﹁締椹跺叆缁堢偣
				if (idNextCross == car.idCrossTo)//濡傛灉姝よ溅灏嗚椹跺嚭鍑哄彛
				{
					num_CarsScheduling -= 1;//姝ｅ湪璋冨害鐨勮溅杈嗘暟鍑忎竴
					std::vector<Car>::iterator it = roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.begin();
					roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.erase(it);//鍒犻櫎璇ラ亾璺涓�杈嗚溅
					return 2;//浠ｈ〃鍓嶉潰涓�杈嗚溅鍒拌揪缁堢偣
				}
				else
				{
					int idNextRoad = car.path[0];//姝よ溅鍗冲皢椹跺叆鐨勯亾璺�
					int idCurRoad = car.idCurRoad;//姝よ溅褰撳墠閬撹矾
					roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].dirCross = getCrossDir(idCurRoad, idNextRoad, idNextCross);//璁剧疆璺彛鏂瑰悜
					roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].status = WAITTING;//姝よ溅鍙樹负绛夊緟鐘舵��
				}
			}
		}
		else//璇ヨ溅鍓嶉潰鏈夎溅
		{
			Car carNext = roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar - 1];
			if (car.location + std::min(roads[car.idCurRoad - 5000].speed, car.speed) < carNext.location)
			{//鍓嶉潰鐨勮溅涓嶅舰鎴愰樆鎸�
				roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].location += std::min(roads[car.idCurRoad - 5000].speed, car.speed);//杞︽甯歌椹�
				roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].status = FINESHED;//杞︽爣璁颁负缁堟鐘舵��
				roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].dirCross = NONE;
			}
			else
			{//鍓嶉潰鐨勮溅褰㈡垚闃绘尅
				//鍒ゆ柇姝よ溅浼氫笉浼氶�氳繃璺彛
				if (car.location + std::min(roads[car.idCurRoad - 5000].speed, car.speed) <= roads[car.idCurRoad - 5000].length)//涓嶄細椹跺嚭璺彛
				{
					if (carNext.status == FINESHED)
					{
						roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].location = roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar - 1].location - 1;//琛岄┒鍒板墠杞︾殑鍚庝竴涓綅缃�
						roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].status = FINESHED;//杞︽爣璁颁负缁堟鐘舵��
						roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].dirCross = NONE;
					}
					else
					{
						roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].status = WAITTING;//杞︽爣璁颁负WAITTING
					}
				}
				else
				{
					//姝よ溅涔熷皢琛岄┒鍑鸿矾鍙ｏ紝闇�瑕佸垽鏂杞﹀湪璺彛鐨勬柟鍚�
					//鍒ゆ柇杞︾殑鏂瑰悜
					int idNextCross = 0;
					if (car.idCurLane >= roads[car.idCurRoad - 5000].channel)//閫嗗悜
						idNextCross = roads[car.idCurRoad - 5000].idFrom;//姝よ溅鍗冲皢椹跺叆鐨勮矾鍙�
					else
						idNextCross = roads[car.idCurRoad - 5000].idTo;//姝よ溅鍗冲皢椹跺叆鐨勮矾鍙�

					if (idNextCross == car.idCrossTo)//濡傛灉姝よ溅灏嗚鍒拌揪缁堢偣
					{
						roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].status = WAITTING;//杞︽爣璁颁负WAITTING
						roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].dirCross = NONE;
					}
					else
					{
						int idNextRoad = car.path[0];//姝よ溅鍗冲皢椹跺叆鐨勯亾璺�
						int idCurRoad = car.idCurRoad;//姝よ溅褰撳墠閬撹矾
						roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].dirCross = getCrossDir(idCurRoad, idNextRoad, idNextCross);//璁剧疆璺彛鏂瑰悜
						roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].status = WAITTING;//姝よ溅鍙樹负绛夊緟鐘舵��
					}
				}
			}
		}
	}
	else//璇ヨ溅鍑嗗椹跺嚭璺彛
	{
		assert(car.status == WAITTING);//鍙湁澶勪簬绛夊緟鐘舵�佺殑杞︽墠鑳藉嚭璺彛
		int idNextCross = 0;
		if (car.idCurLane >= roads[car.idCurRoad - 5000].channel)//閫嗗悜
			idNextCross = roads[car.idCurRoad - 5000].idFrom;//姝よ溅鍗冲皢椹跺叆鐨勮矾鍙�
		else
			idNextCross = roads[car.idCurRoad - 5000].idTo;//姝よ溅鍗冲皢椹跺叆鐨勮矾鍙�
		assert(idNextCross != car.idCrossTo);
		int idNextRoad = car.path[0];//鑾峰彇鐩爣閬撹矾
		int idNextLane = isCanEnter(idNextRoad, idNextCross);
		if (idNextLane != -1)//濡傛灉璇ラ亾璺彲鍔犲叆杞�
		{
			int disNextRoad = getCrossDistance(car, car.idCurRoad, idNextRoad);
			if (disNextRoad == 0)//鍙椹惰窛绂讳负0锛屽垯鍋滃湪褰撳墠璺彛
			{
				//姝ゆ椂姣旇緝鐗规畩锛屽洜涓烘病鏈夊彂鐢烺oad鍙樺寲锛屾墍浠ar渚濈劧鍦ㄥ綋鍓峫ane锛屼絾鏄叾location闇�瑕佹洿鏂�
				roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[0].location = roads[car.idCurRoad - 5000].length;
				roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[0].status = FINESHED;//璇ヨ溅璋冨害瀹屾垚锛岀瓑寰呬笅涓�鏃堕棿鐗囧啀琛岄┒
				roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[0].dirCross = NONE;
				return 1;
			}
			else
			{
				if (roads[idNextRoad-5000].lane[idNextLane].laneCar.size() != 0)//濡傛灉璇ヨ溅閬撴湁杞�
				{
					//鍒ゆ柇瀵瑰簲杞﹂亾杞︾殑浣嶇疆
					Car carNext = roads[idNextRoad - 5000].lane[idNextLane].laneCar[roads[idNextRoad - 5000].lane[idNextLane].laneCar.size() - 1];
					if (disNextRoad < carNext.location)//涓嶅舰鎴愰樆鎸�
					{
						driverToNextRoad(car, idNextRoad, idNextLane, disNextRoad);//琛岄┒鍒颁笅涓矾鍙�
						return 1;
					}
					else//褰㈡垚闃绘尅
					{
						if (carNext.status = FINESHED)
						{
							driverToNextRoad(car, idNextRoad, idNextLane, carNext.location - 1);//琛岄┒鍒颁笅涓矾鍙ｏ紝鍓嶈溅涔嬪悗
							assert(carNext.location >= 1);
							return 1;
						}
						//濡傛灉鍓嶈溅澶勪簬绛夊緟鐘舵�侊紝閭ｄ箞姝よ溅涔熶笉琛岄┒锛岀户缁瓑寰�
					}
				}
				else//濡傛灉璇ヨ溅閬撴病鏈夐樆鎸�
				{
					driverToNextRoad(car, idNextRoad, idNextLane, disNextRoad);//琛岄┒鍒颁笅涓矾鍙�
					return 1;
				}
			}
		}
		//濡傛灉鐩爣杞﹂亾鏃犳硶椹跺叆锛屼繚鎸乄AITTING鐘舵��
	}
	return false;
}

void Scheduler::addCar(Car car, int i)
{
	assert(car.status == SLEEPING);//鍙湁SLEEPING鐘舵�佺殑杞﹀彲浠ュ姞鍏ュ湴鍥捐椹�
	int idRoadTarget = car.path[0];//鑾峰彇鐩爣閬撹矾
	int idCrossTarget = car.idCrossFrom;//鑾峰緱璇ヨ溅鍑哄彂璺彛
	int idLaneTarget = isCanEnter(idRoadTarget, idCrossTarget);
	if (idLaneTarget != -1)//濡傛灉璇ラ亾璺彲鍔犲叆杞�
	{
		car.status = FINESHED;//鍒囨崲car鐨勭姸鎬�
		car.idCurRoad = idRoadTarget;
		car.idCurLane = idLaneTarget;
		car.location = 0;
		car.dirCross = NONE;
		std::vector<int>::iterator itPath = car.path.begin();
		car.path.erase(itPath);//宸茬粡椹跺悜涓嬩竴涓矾鍙ｏ紝鎵�浠ュ垹闄ath涓涓�椤�
		int indexCar = roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.size();//璇ヨ溅涓烘湯灏�
		roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.push_back(car);//灏嗚杞﹀姞鍏ュ搴旈亾璺�,瀵瑰簲杞﹂亾,鍔犲叆闃熷熬
		driveCar(car, indexCar);//car琛岄┒ indexCar涓�-1锛岃〃绀鸿杞﹀湪lane涓繕娌℃湁浣嶇疆
		cars[car.id - 10000].starttime = time_Scheduler;
		num_CarsPut += 1;
		//roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar - 1].starttime = time_Scheduler;
		//璁板綍瀹為檯鍑哄彂鏃堕棿
	}
	else//濡傛灉鍔犲叆澶辫触锛屽垯灏嗗嚭鍙戞椂闂村欢鍚庝竴涓�1涓椂闂寸墖
	{
		cars[i].plantime += 1;
	}
}

void Scheduler::addCar(Car car, int i, Graph_DG & graph)
{
	assert(car.status == SLEEPING);//鍙湁SLEEPING鐘舵�佺殑杞﹀彲浠ュ姞鍏ュ湴鍥捐椹�
	int idRoadTarget = car.path[0];//鑾峰彇鐩爣閬撹矾
	int idCrossTarget = car.idCrossFrom;//鑾峰緱璇ヨ溅鍑哄彂璺彛
	int idLaneTarget = isCanEnter(idRoadTarget, idCrossTarget);
	if (idLaneTarget != -1)//濡傛灉璇ラ亾璺彲鍔犲叆杞�
	{
		car.status = FINESHED;//鍒囨崲car鐨勭姸鎬�
		car.idCurRoad = idRoadTarget;
		car.idCurLane = idLaneTarget;
		car.location = 0;
		car.dirCross = NONE;
		std::vector<int>::iterator itPath = car.path.begin();
		car.path.erase(itPath);//宸茬粡椹跺悜涓嬩竴涓矾鍙ｏ紝鎵�浠ュ垹闄ath涓涓�椤�
		int indexCar = roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.size();//璇ヨ溅涓烘湯灏�
		roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.push_back(car);//灏嗚杞﹀姞鍏ュ搴旈亾璺�,瀵瑰簲杞﹂亾,鍔犲叆闃熷熬
		driveCar(car, indexCar);//car琛岄┒ indexCar涓�-1锛岃〃绀鸿杞﹀湪lane涓繕娌℃湁浣嶇疆
		cars[car.id - 10000].starttime = time_Scheduler;
		num_CarsPut += 1;
		//roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar - 1].starttime = time_Scheduler;
		//璁板綍瀹為檯鍑哄彂鏃堕棿
	}
	else//濡傛灉鍔犲叆澶辫触锛屽垯灏嗗嚭鍙戞椂闂村欢鍚庝竴涓�1涓椂闂寸墖
	{
		cars[i].plantime += 1;
	}
}

int Scheduler::isCanEnter(int idRoad, int idCross)
{
	// 鈥斺�斺�斺�斺�斺�斺�斺��
	//     3
	//  <鈥� 鈥� 鈥� 鈥�
	//     2
	//鈥斺�斺�斺�斺�斺�斺�斺�斺�斺��
	//     0
	//  鈥� 鈥� 鈥� 鈥�>
	//     1
	//鈥斺�斺�斺�斺�斺�斺�斺�斺�斺��
	int idStartLane = 0;//濡傛灉cross涓洪亾璺殑鍏ユ柟鍚戯紝闇�瑕佽皟搴� 0 1 2杞﹂亾锛屽惁鍒欒皟搴� 3 4 5杞﹂亾
	if (roads[idRoad - 5000].idTo == crosses[idCross - 1].id)//濡傛灉cross涓洪亾璺殑鍏ユ柟鍚�
	{
		idStartLane = roads[idRoad - 5000].channel;
		assert(roads[idRoad - 5000].isDuplex == 1);//濡傛灉cross涓簉oad鐨勫嚭鏂瑰悜锛屼絾鏄嵈涓嶆槸鍙屽悜閬擄紝閭ｄ箞寰堝彲鑳借矾寰勮鍒掗敊璇�
		if (roads[idRoad - 5000].isDuplex != 1)
			return -1;
	}
	for (int j = idStartLane; j < idStartLane + roads[idRoad - 5000].channel; ++j)//閬嶅巻鎵�鏈塴ane
	{
		if (roads[idRoad - 5000].lane[j].laneCar.size() < roads[idRoad - 5000].length)
		{
			//杩欓噷鏈夐棶棰橈紝杞︾殑鏁伴噺鍙兘灏忎簬杞﹂亾闀垮害锛屼絾鏄渶鍚庝竴杈嗚溅鏈夊彲鑳藉牭鍦ㄦ渶鍚庝竴浣�
			if (roads[idRoad - 5000].lane[j].laneCar.size() > 0)
			{
				Car carNext = roads[idRoad - 5000].lane[j].laneCar[roads[idRoad - 5000].lane[j].laneCar.size() - 1];
				if (carNext.location == 0)
					continue;//淇姝ゅ閲嶅ぇbug锛屽師鏈槸return -1;
				else
					return j;
			}
			else
			{
				return j;//瀛樺湪绌轰綅锛屽彲鍔犲叆,杩斿洖杞﹂亾id
			}
		}
	}
	return -1;//涓嶅瓨鍦ㄧ┖浣�
}

bool Scheduler::isBeDD(int idRoad, int idCross)//娉ㄦ剰杩欓噷鐨処D涓嶉渶瑕佸噺1
{
	if (idRoad == -1)//濡傛灉鍐茬獊鏂瑰悜鏃犻亾璺紝鍒欎换鍔℃棤鍐茬獊杞﹁締
		return false;
	int idStartLane = 0;//濡傛灉cross涓洪亾璺殑鍑烘柟鍚戯紝闇�瑕佽皟搴� 0 1 2杞﹂亾锛屽惁鍒欒皟搴� 3 4 5杞﹂亾
	if (roads[idRoad - 5000].idFrom == crosses[idCross - 1].id)//濡傛灉cross涓洪亾璺殑鍑烘柟鍚�
	{
		idStartLane = roads[idRoad - 5000].channel;
		if (roads[idRoad - 5000].isDuplex != 1)
			return false;
	}
	//鍘熸湰浠ヤ负闇�瑕佸垽鏂墍鏈夎溅閬擄紝鐜板湪鍙垽鏂紭鍏堢骇鏈�楂樼殑杞﹂亾
	if (roads[idRoad - 5000].lane[idStartLane].laneCar.size() != 0)
	{
		if (roads[idRoad - 5000].lane[idStartLane].laneCar[0].dirCross == DD)
		{
			//鍙湁绛夊緟鐘舵�佺殑杞︿細褰㈡垚鍐茬獊
			if (roads[idRoad - 5000].lane[idStartLane].laneCar[0].status == WAITTING)
				return true;//瀛樺湪宸﹁浆杞﹁締
		}

	}
	return false;
}

bool Scheduler::isBeLEFT(int idRoad, int idCross)//娉ㄦ剰杩欓噷鐨処D涓嶉渶瑕佸噺1
{
	if (idRoad == -1)//濡傛灉鍐茬獊鏂瑰悜鏃犻亾璺紝鍒欎换鍔℃棤鍐茬獊杞﹁締
		return false;
	int idStartLane = 0;//濡傛灉cross涓洪亾璺殑鍑烘柟鍚戯紝闇�瑕佽皟搴� 0 1 2杞﹂亾锛屽惁鍒欒皟搴� 3 4 5杞﹂亾
	if (roads[idRoad - 5000].idFrom == crosses[idCross - 1].id)//濡傛灉cross涓洪亾璺殑鍑烘柟鍚�
	{
		idStartLane = roads[idRoad - 5000].channel;
		if (roads[idRoad - 5000].isDuplex != 1)
			return false;
	}
	//鍘熸湰浠ヤ负闇�瑕佸垽鏂墍鏈夎溅閬擄紝鐜板湪鍙垽鏂紭鍏堢骇鏈�楂樼殑杞﹂亾
	if (roads[idRoad - 5000].lane[idStartLane].laneCar.size() != 0)
	{
		if (roads[idRoad - 5000].lane[idStartLane].laneCar[0].dirCross == LEFT)
		{
			//鍙湁绛夊緟鐘舵�佺殑杞︿細褰㈡垚鍐茬獊
			if(roads[idRoad - 5000].lane[idStartLane].laneCar[0].status == WAITTING)
				return true;//瀛樺湪宸﹁浆杞﹁締
		}
	}
	return false;
}

int Scheduler::getCrossDir(int idCurRoad, int idNextRoad, int idNextCross)
{
	int dirCurRoad = 0;//褰撳墠閬撹矾鍦ㄨ矾鍙ｇ殑鏂瑰悜
	int dirNextRoad = 0;//鍗冲皢椹跺叆閬撹矾鍦ㄨ矾鍙ｇ殑鏂瑰悜
	if (crosses[idNextCross - 1].roadID_T == idCurRoad)
		dirCurRoad = 0;
	else if (crosses[idNextCross - 1].roadID_R == idCurRoad)
		dirCurRoad = 1;
	else if (crosses[idNextCross - 1].roadID_D == idCurRoad)
		dirCurRoad = 2;
	else if (crosses[idNextCross - 1].roadID_L == idCurRoad)
		dirCurRoad = 3;

	if (crosses[idNextCross - 1].roadID_T == idNextRoad)
		dirNextRoad = 0;
	else if (crosses[idNextCross - 1].roadID_R == idNextRoad)
		dirNextRoad = 1;
	else if (crosses[idNextCross - 1].roadID_D == idNextRoad)
		dirNextRoad = 2;
	else if (crosses[idNextCross - 1].roadID_L == idNextRoad)
		dirNextRoad = 3;

	switch (dirNextRoad - dirCurRoad)
	{
	case 0:
		break;
	case 1:
		return LEFT;
		break;
	case -1:
		return RIGHT;
		break;
	case 2:
		return DD;
		break;
	case -2:
		return DD;
		break;
	case 3:
		return RIGHT;
		break;
	case -3:
		return LEFT;
		break;
	default:
		break;
	}
}

int Scheduler::getCrossDistance(Car car, int idCurRoad, int idNextRoad)
{
	int disCurRoad = roads[car.idCurRoad - 5000].length - car.location;//褰撳墠閬撹矾鍙互琛岄┒鐨勮窛绂�
	int disNextRoadMax = std::min(roads[idNextRoad - 5000].speed, car.speed);//涓嬩竴閬撹矾鍙互琛岄┒鐨勬渶澶ц窛绂�
	assert(disCurRoad >= 0);
	assert(disNextRoadMax >= 0);
	if (disCurRoad >= disNextRoadMax)
		return 0;
	else
		return disNextRoadMax - disCurRoad;
}

void Scheduler::driverToNextRoad(Car car, int idNextRoad, int idNextLane, int location)
{
	std::vector<Car>::iterator it = roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.begin();
	roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.erase(it);	//灏嗚杞︿粠褰撳墠lane鍒犻櫎,璇ヨ溅鑲畾鏄綋鍓峫ane鐨勭涓�杈嗚溅
	car.idCurRoad = idNextRoad;
	car.idCurLane = idNextLane;
	car.location = location;
	car.dirCross = NONE;//璇ヨ溅璺彛鐘舵�佽缃负NONE,浠ｈ〃宸茬粡椹剁璺彛
	car.status = FINESHED;//璇ヨ溅璋冨害瀹屾垚锛岀瓑寰呬笅涓�鏃堕棿鐗囧啀琛岄┒
	std::vector<int>::iterator itPath = car.path.begin();
	car.path.erase(itPath);//宸茬粡椹跺悜涓嬩竴涓矾鍙ｏ紝鎵�浠ュ垹闄ath涓涓�椤�
	roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.push_back(car);//灏嗚杞﹀姞鍏ヤ笅涓�涓猯ane,鍔犲叆闃熷熬
}

bool Scheduler::isCanDriveToNextRoad(Car car, int dir, int idCross)
{
	if (crosses[idCross-1].roadID[dir] != -1)
	{
		int idNextLane = isCanEnter(crosses[idCross - 1].roadID[dir], idCross);
		if (idNextLane != -1)
		{
			int result = driveCar(car, -1);//璇ヨ溅鍙互杞悜锛屼絾鏄笉浠ｈ〃璇ヨ溅杞悜鍚庝笉浼氬洜涓烘湁杞﹂樆鎸＄劧鍚嶹AITTING
			//瀹為檯涓奷riverCar鍚庡彲鑳芥湁濡備笅涓夌鎯呭喌
			//1.鎴愬姛琛岄┒鍒颁笅涓矾鍙�
			//2.鍥犱负涓嬩釜璺彛鏈夎溅waitting,姝よ溅鏃犳硶琛岄┒
			//3.鍥犱负琛岄┒閲岀▼涓嶅锛屽彧鑳界户缁瓑鍦ㄨ矾鍙�
			//if (car_original.idCurRoad != car.idCurRoad || car_original.location != car.location)//road鍙戠敓鍙樺寲鎴栬�卨ocation鍙戠敓鍙樺寲锛岃涓鸿浆鍚戞垚鍔�
			if(result == 1)
				return true;
			//绗笁绉嶆儏鍐佃涓烘垚鍔熸垨鑰呬笉鎴愬姛锛屽缁撴灉搴旇褰卞搷涓嶅ぇ
		}
	}
	return false;
}

void Scheduler::driverCarInGarage()
{
	for (int i = 0; i < num_Cars; ++i)
	{
		if (cars[i].plantime == time_Scheduler&&cars[i].status==SLEEPING)
			addCar(cars[i],i);
	}
}

void Scheduler::driverCarInGarageDynamic(Graph_DG &graph)
{
	for (int i = 0; i < num_Cars; ++i)
	{
		if (cars[i].plantime == time_Scheduler && cars[i].status == SLEEPING)
		{
			vector<int> pathCross = graph.Dijkstra(cars[i].idCrossFrom, cars[i].idCrossTo, cars[i].speed, graphRoadStatusByDS, 0.2);
			//cross鐭╅樀杞瑀oad鐭╅樀
			vector<int> pathRoad(pathCross.size() - 1);
			for (int j = 0; j < pathRoad.size(); ++j)
			{
				pathRoad[j] = graphC2R[pathCross[j] - 1][pathCross[j + 1] - 1];
				//assert(pathRoad[j] != 0);
			}
			cars[i].path = pathRoad;
			addCar(cars[i], i, graph);
			timecount++;
		}
	}
}

void Scheduler::putCarStatus(Car car)
{
	if(car.path.size()!=0)
		PRINT("CarID:%d  idFrom:%d  idTo:%d idCurRoad:%d  idCurLane:%d  location:%d  NextRoad:%d  Status:%d\n",
		car.id,car.idCrossFrom,car.idCrossTo,car.idCurRoad,car.idCurLane,car.location,car.path[0], car.status);//鎵撳嵃绯荤粺鏃堕棿
	else
		PRINT("CarID:%d  idFrom:%d  idTo:%d idCurRoad:%d  idCurLane:%d  location:%d  Status:%d\n",
			car.id, car.idCrossFrom, car.idCrossTo, car.idCurRoad, car.idCurLane, car.location, car.status);//鎵撳嵃绯荤粺鏃堕棿
}

void Scheduler::putAllCarStatus()
{
	for (int i = 0; i < num_Roads; ++i)//鎸夐亾璺疘D鍗囧簭杩涜璋冨害
	{
		for (int j = 0; j < roads[i].channel * (1 + roads[i].isDuplex); ++j)
		{
			Lane lane = roads[i].lane[j];
			
			if (lane.laneCar.size() != 0)//鍏堝垽鏂杞﹂亾鏄惁鏈夎溅
			{
				for (int m = 0; m < lane.laneCar.size(); ++m)
				{
					putCarStatus(lane.laneCar[m]);
				}
			}
		}
	}
}

void Scheduler::putAllRoadStatus()
{
	int num_road_jam = 0;//鍫典綇鐨勯亾璺暟閲忥紝per楂樹簬0.7鍒欒涓哄牭杞�
	for (int i = 0; i < num_Roads; ++i)
	{
		float perRoad = 0;
		if (roads[i].isDuplex)
		{
			for (int j = 0; j < roads[i].channel; ++j)
			{
				float per = (float)roads[i].lane[j].laneCar.size() / (float)roads[i].length;
				perRoad += per;
			}
			perRoad = perRoad / roads[i].channel;
			graphRoadStatusByDS[roads[i].idFrom - 1][roads[i].idTo - 1] = perRoad;//鏇存柊鎷ュ牭鎯呭喌鐭╅樀
			perRoad = 0;
			for (int j = roads[i].channel; j < 2 * roads[i].channel; ++j)
			{
				float per = (float)roads[i].lane[j].laneCar.size() / (float)roads[i].length;
				perRoad += per;
			}
			perRoad = perRoad / roads[i].channel;
			graphRoadStatusByDS[roads[i].idTo - 1][roads[i].idFrom - 1] = perRoad;//鏇存柊鎷ュ牭鎯呭喌鐭╅樀
			perRoad = 0;
		}
		else
		{
			for (int j = 0; j < roads[i].channel; ++j)
			{
				float per = (float)roads[i].lane[j].laneCar.size() / (float)roads[i].length;
				perRoad += per;
			}
			perRoad = perRoad / roads[i].channel;
			graphRoadStatusByDS[roads[i].idFrom - 1][roads[i].idTo - 1] = perRoad;//鏇存柊鎷ュ牭鎯呭喌鐭╅樀
			perRoad = 0;
		}
	}
}

int Scheduler::getFirstRoadFromCross(int idCross, int index)
{
	std::vector<int> idRoad;
	for (int i = 0; i < 4; ++i)
	{
		if (crosses[idCross - 1].roadID[i] == -1)
			idRoad.push_back(INT_MAX);
		else
			idRoad.push_back(crosses[idCross - 1].roadID[i]);
	}
	std::sort(idRoad.begin(),idRoad.end());
	for (int i = 0; i < 4; ++i)
	{
		if (idRoad[i] == INT_MAX)
			idRoad[i] = -1;
	}
	return idRoad[index];
}

int Scheduler::getDirByRoadCrossDir(int idCross, int idRoad)
{
	int dirFrom = -1;//璁剧疆鍒濆�间负-1锛屼究浜庡悗缁敊璇娴�
	for (int i = 0; i < 4; ++i)
	{
		if (crosses[idCross - 1].roadID[i] == idRoad)
			dirFrom = i;
	}
	assert(dirFrom != -1);//濡傛灉鏈湪璇ross鎵惧埌璇oad锛屽垯鎶ラ敊

	return dirFrom;
}

void Scheduler::driveAllCarsJustOnOneChannelToEndState(int idRoad, int idCross, int idChannel)
{
	if (roads[idRoad - 5000].lane[idChannel].laneCar.size() != 0)
	{
		for (int i = 0; i < roads[idRoad - 5000].lane[idChannel].laneCar.size(); ++i)
		{
			if (roads[idRoad - 5000].lane[idChannel].laneCar[i].status == WAITTING)//鍙鐞嗙瓑寰呯姸鎬佺殑杞�
			{
				Car car = roads[idRoad - 5000].lane[idChannel].laneCar[i];
				if (car.location + std::min(roads[idRoad - 5000].speed, car.speed) <= roads[idRoad - 5000].length)//涓嶄細椹跺嚭璺彛
				{
					//鍙鐞嗚椹跺悗涓嶉�氳繃璺彛鐨勮溅
					if (i != 0)//濡傛灉璇ヨ溅涓嶆槸绗竴杈嗚溅
					{
						Car carNext = roads[idRoad - 5000].lane[idChannel].laneCar[i - 1];
						if (car.location + std::min(roads[idRoad - 5000].speed, car.speed) < carNext.location)
						{
							//鍓嶈溅涓嶅舰鎴愰樆鎸�
							roads[idRoad - 5000].lane[idChannel].laneCar[i].location += std::min(roads[idRoad - 5000].speed, car.speed);//杞︽甯歌椹�
							roads[idRoad - 5000].lane[idChannel].laneCar[i].status = FINESHED;//杞︽爣璁颁负缁堟鐘舵��
							roads[idRoad - 5000].lane[idChannel].laneCar[i].dirCross = NONE;
						}
						else if (carNext.status == FINESHED)
						{
							roads[idRoad - 5000].lane[idChannel].laneCar[i].location = carNext.location - 1;//杞︽甯歌椹�
							roads[idRoad - 5000].lane[idChannel].laneCar[i].status = FINESHED;//杞︽爣璁颁负缁堟鐘舵��
							roads[idRoad - 5000].lane[idChannel].laneCar[i].dirCross = NONE;
						}
					}
					else
					{
						//鍓嶈溅涓嶅舰鎴愰樆鎸�
						roads[idRoad - 5000].lane[idChannel].laneCar[i].location += std::min(roads[idRoad - 5000].speed, car.speed);//杞︽甯歌椹�
						roads[idRoad - 5000].lane[idChannel].laneCar[i].status = FINESHED;//杞︽爣璁颁负缁堟鐘舵��
						roads[idRoad - 5000].lane[idChannel].laneCar[i].dirCross = NONE;
					}
				}
				/*
				else
				{
					//姝よ溅涔熷皢琛岄┒鍑鸿矾鍙ｏ紝闇�瑕佸垽鏂杞﹀湪璺彛鐨勬柟鍚�
					//鍒ゆ柇杞︾殑鏂瑰悜
					int idNextCross = 0;
					if (car.idCurLane >= roads[car.idCurRoad - 5000].channel)//閫嗗悜
						idNextCross = roads[car.idCurRoad - 5000].idFrom;//姝よ溅鍗冲皢椹跺叆鐨勮矾鍙�
					else
						idNextCross = roads[car.idCurRoad - 5000].idTo;//姝よ溅鍗冲皢椹跺叆鐨勮矾鍙�
					//鏍规嵁鍋囪AA锛屾鏃跺彲鑳芥湁杞﹁締椹跺叆缁堢偣
					if (idNextCross == car.idCrossTo)//濡傛灉姝よ溅灏嗚椹跺嚭鍑哄彛
					{
						num_CarsScheduling -= 1;//姝ｅ湪璋冨害鐨勮溅杈嗘暟鍑忎竴
						std::vector<Car>::iterator it = roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.begin();
						roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.erase(it);//鍒犻櫎璇ラ亾璺涓�杈嗚溅
					}
					else
					{
						int idNextRoad = car.path[0];//姝よ溅鍗冲皢椹跺叆鐨勯亾璺�
						int idCurRoad = car.idCurRoad;//姝よ溅褰撳墠閬撹矾
						roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[i].dirCross = getCrossDir(idCurRoad, idNextRoad, idNextCross);//璁剧疆璺彛鏂瑰悜
						roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[i].status = WAITTING;//姝よ溅鍙樹负绛夊緟鐘舵��
					}
				}
				*/
			}
		}
	}
}

void Scheduler::getPath()
{
	Graph_DG graph(vexnum, edge);
	graph.createArcGraph(tmp);

	for (int i = 0; i < num_Cars; ++i)
	{
		vector<int> pathCross = graph.Dijkstra(cars[i].idCrossFrom, cars[i].idCrossTo);
		vector<int> pathRoad(pathCross.size() - 1);
		for (int j = 0; j < pathRoad.size(); ++j)
		{
			pathRoad[j] = graphC2R[pathCross[j] - 1][pathCross[j + 1] - 1];
			//assert(pathRoad[j] != 0);
		}
		cars[i].path = pathRoad;
	}
}

void Scheduler::getPathByTime()
{
	static int num = 0;
  
	static int flagnum = 0;
	static int flag[100] = { 0 };
	static int flag_road[120] = { 0 };
  
	Graph_DG graph(vexnum, edge);
	graph.createArcGraph(tmp);
	graph.createArcRoadvGraph(tmp1);

	for (int i = 0; i < num_Cars; ++i)
	{
		vector<int> pathCross = graph.Dijkstra(cars[i].idCrossFrom, cars[i].idCrossTo, cars[i].speed);

		//缁熻杞﹁締鎯呭喌锛屾瘡100杈嗚溅鏇存柊涓�娆amDegree鐨勭煩闃�
		num++;
		flagnum++;
		if (num == 200)
		{
			num = 0;
			graph.upDateJam();
		}
		for (int i = 0; i < pathCross.size(); i++) 
		{
			flag[pathCross.at(i) - 1]++;//璁板綍64涓猚ross鐨勪娇鐢ㄦ儏鍐�
		}
		//灏嗙粺璁＄殑鎯呭喌鏀惧埌jamDegreeTmp鐨勭煩闃典腑

		for (int i = 0, j = 1; j < pathCross.size(); i++, j++)
		{
			graph.jamDegreeTmp[pathCross.at(i) - 1][pathCross.at(j) - 1]++;
		}

		vector<int> pathRoad(pathCross.size() - 1);
		for (int j = 0; j < pathRoad.size(); ++j)
		{
			pathRoad[j] = graphC2R[pathCross[j] - 1][pathCross[j + 1] - 1];
			//assert(pathRoad[j] != 0);
		}
		/*
		//缁熻road鐨勬儏鍐�
		for (int i = 0; i < pathRoad.size(); i++)
		{
			flag_road[pathRoad.at(i) - 5000]++;//璁板綍road鐨勪娇鐢ㄦ儏鍐�
		}


		//缁熻cross鐨勬儏鍐�
		if (flagnum == 200)
		{
			ofstream oFile;
			oFile.open("testcross100.csv", ios::out | ios::trunc);
			for (int i=0; i < 100; i++)
			{
				oFile << flag[i] << endl;
			}

			oFile.close();

			for (int i = 0; i < 100; i++)
			{
				flag[i] = 0;
			}

			ofstream oFile1;
			oFile1.open("testroad100.csv", ios::out | ios::trunc);
			for (int i = 0; i < 120; i++)
			{
				oFile1 << flag_road[i] << endl;
			}
			oFile1.close();

			for (int i = 0; i < 120; i++)
			{
				flag_road[i] = 0;
			}
		}
		*/
		/*鍐欏嚭100~200杈嗚溅鐨勭粺璁℃儏鍐�*/
		/*
		if (flagnum == 400)
		{
			ofstream oFile2;
			oFile2.open("testcross200.csv", ios::out | ios::trunc);
			for (int i = 0; i < 100; i++)
			{
				oFile2 << flag[i] << endl;
			}

			oFile2.close();


			ofstream oFile3;
			oFile3.open("testroad200.csv", ios::out | ios::trunc);
			for (int i = 0; i < 120; i++)
			{
				oFile3 << flag_road[i] << endl;
			}

			oFile3.close();
		}
		*/
		cars[i].path = pathRoad;
	}
}

void Scheduler ::getPathByTime_dynamic()
{
	static int num = 0;
	//static int flag[100] = { 0 };

	Graph_DG graph(vexnum, edge);
	graph.createArcGraph(tmp);
	graph.createArcRoadvGraph(tmp1);

	for (int i = 0; i < num_Cars; ++i)
	{
		vector<int> pathCross = graph.DijkstraNor(cars[i].idCrossFrom, cars[i].idCrossTo, cars[i].speed);

		num++;
		//定时更新交通拥堵邻接矩阵jamDegreeLongBefore
		if (num == 200)
		{
			num = 0;
			graph.upDateJamStatic();
			graph.cleanUpJamDegreeBefore();

		}

		//将统计的情况放到 jamDegreeBefore的矩阵中
		for (int i = 0, j = 1; j < pathCross.size(); i++, j++)
		{
			graph.jamDegreeBefore[pathCross.at(i) - 1][pathCross.at(j) - 1]++;
		}

		graph.upDateJamDynamic();

		//if (num == 100)
		//{
		//	ofstream oFile;
		//	oFile.open("test.csv", ios::out | ios::trunc);
		//	for (int i=0; i < 100; i++)
		//	{
		//		oFile << flag[i] << endl;
		//	}
		//	
		//	for (int i = 0; i < 64; i++)
		//	{
		//		flag[i] = 0;
		//	}
		//	graph.upDateJam();
		//	oFile.close();
		//}

		///*写出100~200辆车的统计情况*/
		//if (num == 200)
		//{
		//	ofstream oFile;
		//	oFile.open("test2.csv", ios::out | ios::trunc);
		//	for (int i = 0; i < 100; i++)
		//	{
		//		oFile << flag[i] << endl;
		//	}

		//	oFile.close();
		//}

		vector<int> pathRoad(pathCross.size() - 1);
		for (int j = 0; j < pathRoad.size(); ++j)
		{
			pathRoad[j] = graphC2R[pathCross[j] - 1][pathCross[j + 1] - 1];
			//assert(pathRoad[j] != 0);
		}
		cars[i].path = pathRoad;
	}
}