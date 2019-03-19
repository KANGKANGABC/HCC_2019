#include "Scheduler.h"

Scheduler::Scheduler(DataCenter &dc)
{
	num_CarsScheduling = dc.m_car_num;//�����Ҫ���ȵĳ�����
	num_Roads = dc.m_road_num;
	num_Crosses = dc.m_cross_num;
	num_Cars = dc.m_car_num;
	roads = dc.road;
	crosses = dc.cross;
	cars = dc.car;
	time_Scheduler = 0;//��������ʼʱ������Ϊ0
	vexnum = dc.getCrossNum();
	edge = dc.getRoadNum();
	tmp = dc.getArc(); //�õ��ڽӾ���
	tmp1 = dc.getRoadvArc(); //�õ���·�����ڽӾ���
	//��graphC2R��С����Ϊ36*36
	graphC2R = dc.graphC2R;
	num_CarsPut = 0;//�Ѿ�����������Ԥ��Ϊ0

}


Scheduler::~Scheduler()
{
}

int Scheduler::getSysTime()
{
	while (num_CarsScheduling > 0)
	{
		PRINT("***********time_Scheduler:%d************\n", time_Scheduler);//��ӡϵͳʱ��

		/*��һ�����ȴ������е�·�ϵĳ��������б���ɨ��*/
		driveAllCarsJustOnRoadToEndState();

		/*�ڶ������ȴ������е�·�ϵĳ��������б���ɨ��*/
		while (1)//��ֹ����Ϊ��һ��ѭ����û���κγ�������
		{
			bool isWorkingCross = false;//��־���������һ��ѭ����û���κ�һ���������ȣ����˳�ѭ��
			for (int i = 0; i < num_Crosses; ++i)////���������������·��
			{
				int idCross = crosses[i].id;//���·��ID
				while (1)//ѭ������·���ĸ�����ĳ���ֱ��ȫ��������ɵ��ȣ���������
				{
					bool isWorkingRoad = false;
					for (int j = 0; j < 4; ++j)//���ﰴҪ���Ǹ��ݵ�·id�����������
					{
						int idRoad = getFirstRoadFromCross(idCross, j);
						if (idRoad != -1)
						{
							int idStartLane = 0;//���crossΪ��·�ĳ�������Ҫ���� 0 1 2������������� 3 4 5����
							if (roads[idRoad - 5000].idFrom == crosses[i].id)//���crossΪ��·���뷽��
							{
								idStartLane = roads[idRoad - 5000].channel;
								if (roads[idRoad - 5000].isDuplex != 1)
									continue;//�����˫�������˳�����ѭ��
							}
							while (1)
							{
								bool isWorkingLane = false;
								for (int m = idStartLane; m < idStartLane + roads[idRoad - 5000].channel; ++m)//��������lane
								{
									if (roads[idRoad - 5000].lane[m].laneCar.size() != 0)
									{
										Car car = roads[idRoad - 5000].lane[m].laneCar[0];
										if (car.status == WAITTING)//ֻ������·����Ϊ�ȴ�״̬�ĳ�
										{
											assert(car.status == WAITTING);//������·�ڵ���ʱһ��Ҫ��WAITTING״̬
											int dirConflict = 0;
											int dirTarget = 0;
											switch (roads[idRoad - 5000].lane[m].laneCar[0].dirCross)
											{
											case NONE:
												break;
											case DD://ֱ��>��ת>��ת
												dirTarget = getDirByRoadCrossDir(idCross,idRoad) + 2;//Ŀ�귽��
												if (dirTarget > 3) dirTarget -= 4;//��������
												if (isCanDriveToNextRoad(car, dirTarget, idCross))
												{
													isWorkingCross = true;
													isWorkingRoad = true;
													isWorkingLane = true;
													driveAllCarsJustOnOneChannelToEndState(idRoad, idCross,m);
												}
												//�ж�ת���road�Ƿ������ʻ

												break;
											case LEFT://��ת>��ת
												//�жϼ���ת��ķ����Ƿ���ֱ�н���ĳ���
												dirConflict = getDirByRoadCrossDir(idCross, idRoad) - 1;//��ͻ����
												if (dirConflict < 0) dirConflict += 4;//��������
												if (!isBeDD(crosses[i].roadID[dirConflict], i))
												{
													dirTarget = getDirByRoadCrossDir(idCross, idRoad) + 1;//Ŀ�귽��
													if (dirTarget > 3) dirTarget -= 4;//��������
													if (isCanDriveToNextRoad(car, dirTarget, idCross))//�ж�ת���road�Ƿ������ʻ
													{
														isWorkingCross = true;
														isWorkingRoad = true;
														isWorkingLane = true;
														driveAllCarsJustOnOneChannelToEndState(idRoad, idCross, m);
													}
												}
												break;
											case RIGHT://��ת���ȼ����
												//�жϼ���ת��ķ����Ƿ���ֱ�н���ĳ���
												dirConflict = getDirByRoadCrossDir(idCross, idRoad) + 1;//��ͻ����
												if (dirConflict > 3) dirConflict -= 4;//��������
												if (!isBeDD(crosses[i].roadID[dirConflict], i))
												{
													dirConflict = getDirByRoadCrossDir(idCross, idRoad) + 2;//��ͻ����
													if (dirConflict > 3) dirConflict -= 4;//��������
													//�жϼ���ת��ķ����Ƿ�����ת����ĳ���
													if (!isBeLEFT(crosses[i].roadID[dirConflict], i))
													{
														dirTarget = getDirByRoadCrossDir(idCross, idRoad) - 1;//Ŀ�귽��
														if (dirTarget < 0) dirTarget += 4;//��������
														if (isCanDriveToNextRoad(car, dirTarget, idCross))//�ж�ת���road�Ƿ������ʻ
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
								else
								{
									//����ô˵����г���·�ڣ���ô�ø�Road������lane��ʻ
									//driveAllCarsJustOnOneRoadToEndState(idRoad, idCross);
								}
							}
						}
					}
					if (!isWorkingRoad)
						break;
				}
			}
			if (!isWorkingCross)//���һ��ѭ����û���κ�һ���������ȣ����˳�����ѭ��
				break;
		}
		driverCarInGarage();//�����е���·��ʻ
		putAllCarStatus();//������г���״̬
		putAllRoadStatus();
		time_Scheduler++;//���µ�����ʱ��
	}
	return time_Scheduler;
}

void Scheduler::driveAllCarsJustOnRoadToEndState()
{
	for (int i = 0; i < num_Roads; ++i)//����·ID������е���
	{
		for (int j = 0; j < roads[i].channel * (1 + roads[i].isDuplex); ++j)
		{
			if (roads[i].lane[j].laneCar.size() != 0)//���жϸó����Ƿ��г�
			{
				for (int m = 0; m < roads[i].lane[j].laneCar.size(); ++m)
				{
					driveCar(roads[i].lane[j].laneCar[m], m);
					//roads[i].lane[j] = lane;//����һ��Ҫ��laneд�أ�������������Ч
					//���ﲻҪд�أ�д�������������ֻ��Ҫ��֤����car��״̬��ͨ��road->lane->car�ķ�ʽ��������
					//�˴������⣬���һ�������������������ǵ�һ������������һ���������������Ϣû�м�ʱ����
					//�������������������������ڵĵڶ�����
					
				}
			}
		}
	}

}

bool Scheduler::driveCar(Car car, int indexCar)
{
	//�������AA�����ĳ������·��ʻ����һ��·����������һ��ʱ��Ƭ��ʻ����һ��·ȫ��
	//Ҳ����˵ֻ�д���NONE״̬WAITTING�ĳ����п��ܼ��������յ�
	if (car.dirCross == NONE)//�ó�������·�ڵȴ�
	{
		assert(indexCar != -1);//�������indexCar����Ϊ-1
		assert(roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.size()!=0);//���Ըó������������Լ�һ����
		assert(indexCar < roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.size());//���Գ������С�ڳ����ϳ�����
		if (indexCar == 0 || roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.size()== 1)//�ó�Ϊ�ó����ĵ�һ���������ϸ�ʱ��Ƭ��׼��ͨ��·��
		{
			//�жϴ˳��᲻��ͨ��·��
			if (car.location + std::min(roads[car.idCurRoad - 5000].speed, car.speed) <= roads[car.idCurRoad - 5000].length)//����ʻ��·��
			{
				roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].location += std::min(roads[car.idCurRoad - 5000].speed, car.speed);//��������ʻ
				roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].status = FINESHED;//�����Ϊ��ֹ״̬
				roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].dirCross = NONE;
				return true;
			}
			else
			{
				//�˳�Ҳ����ʻ��·�ڣ���Ҫ�жϴ˳���·�ڵķ���
				//�жϳ��ķ���
				int idNextCross = 0;
				if(car.idCurLane >= roads[car.idCurRoad - 5000].channel)//����
					idNextCross = roads[car.idCurRoad - 5000].idFrom;//�˳�����ʻ���·��
				else
					idNextCross = roads[car.idCurRoad - 5000].idTo;//�˳�����ʻ���·��
				//���ݼ���AA����ʱ�����г���ʻ���յ�
				if (idNextCross == car.idCrossTo)//����˳���Ҫʻ������
				{
					num_CarsScheduling -= 1;//���ڵ��ȵĳ�������һ
					std::vector<Car>::iterator it = roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.begin();
					roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.erase(it);//ɾ���õ�·��һ����
				}
				else
				{
					int idNextRoad = car.path[0];//�˳�����ʻ��ĵ�·
					int idCurRoad = car.idCurRoad;//�˳���ǰ��·
					roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].dirCross = getCrossDir(idCurRoad, idNextRoad, idNextCross);//����·�ڷ���
					roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].status = WAITTING;//�˳���Ϊ�ȴ�״̬
				}
			}
		}
		else//�ó�ǰ���г�
		{
			Car carNext = roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar - 1];
			if (car.location + std::min(roads[car.idCurRoad - 5000].speed, car.speed) < carNext.location)
			{//ǰ��ĳ����γ��赲
				roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].location += std::min(roads[car.idCurRoad - 5000].speed, car.speed);//��������ʻ
				roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].status = FINESHED;//�����Ϊ��ֹ״̬
				roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].dirCross = NONE;
				return true;
			}
			else
			{//ǰ��ĳ��γ��赲
				//�жϴ˳��᲻��ͨ��·��
				if (car.location + std::min(roads[car.idCurRoad - 5000].speed, car.speed) <= roads[car.idCurRoad - 5000].length)//����ʻ��·��
				{
					if (carNext.status == FINESHED)
					{
						roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].location = roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar - 1].location - 1;//��ʻ��ǰ���ĺ�һ��λ��
						roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].status = FINESHED;//�����Ϊ��ֹ״̬
						roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].dirCross = NONE;
						return true;
					}
					else
					{
						roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].status = WAITTING;//�����ΪWAITTING
						return false;
					}
				}
				/*
				else
				{
					//�˳�Ҳ����ʻ��·��,��ô�жϴ˳���·�ڵķ���
					int idNextCross = 0;
					if (car.idCurLane >= roads[car.idCurRoad - 5000].channel)//����
						idNextCross = roads[car.idCurRoad - 5000].idFrom;//�˳�����ʻ���·��
					else
						idNextCross = roads[car.idCurRoad - 5000].idTo;//�˳�����ʻ���·��
					//���ݼ���AA����ʱ�����г���ʻ���յ�
					if (idNextCross == car.idCrossTo)//����˳���Ҫʻ������
					{
						num_CarsScheduling -= 1;//���ڵ��ȵĳ�������һ
						std::vector<Car>::iterator it = roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.begin();
						roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.erase(it);//ɾ���õ�·��һ����
					}
					else
					{
						int idNextRoad = car.path[0];//�˳�����ʻ��ĵ�·
						int idCurRoad = car.idCurRoad;//�˳���ǰ��·
						roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].dirCross = getCrossDir(idCurRoad, idNextRoad, idNextCross);//����·�ڷ���
						roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].status = WAITTING;//�˳���Ϊ�ȴ�״̬
					}
				}
				*/
			}
		}
	}
	else//�ó�׼��ʻ��·��
	{
		assert(car.status == WAITTING);//ֻ�д��ڵȴ�״̬�ĳ����ܳ�·��
		int idNextCross = 0;
		if (car.idCurLane >= roads[car.idCurRoad - 5000].channel)//����
			idNextCross = roads[car.idCurRoad - 5000].idFrom;//�˳�����ʻ���·��
		else
			idNextCross = roads[car.idCurRoad - 5000].idTo;//�˳�����ʻ���·��
		if (idNextCross == car.idCrossTo)
			PRINT("DONE!");
		assert(idNextCross != car.idCrossTo);
		int idNextRoad = car.path[0];//��ȡĿ���·
		int idNextLane = isCanEnter(idNextRoad, idNextCross);
		if (idNextLane != -1)//����õ�·�ɼ��복
		{
			int disNextRoad = getCrossDistance(car, car.idCurRoad, idNextRoad);
			if (disNextRoad == 0)//����ʻ����Ϊ0����ͣ�ڵ�ǰ·��
			{
				//��ʱ�Ƚ����⣬��Ϊû�з���Road�仯������car��Ȼ�ڵ�ǰlane��������location��Ҫ����
				roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[0].location = roads[car.idCurRoad - 5000].length;
				roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[0].status = FINESHED;//�ó�������ɣ��ȴ���һʱ��Ƭ����ʻ
				roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[0].dirCross = NONE;
				return true;
			}
			else
			{
				if (roads[idNextRoad-5000].lane[idNextLane].laneCar.size() != 0)//����ó����г�
				{
					//�ж϶�Ӧ��������λ��
					Car carNext = roads[idNextRoad - 5000].lane[idNextLane].laneCar[roads[idNextRoad - 5000].lane[idNextLane].laneCar.size() - 1];
					if (disNextRoad < carNext.location)//���γ��赲
					{
						driverToNextRoad(car, idNextRoad, idNextLane, disNextRoad);//��ʻ���¸�·��
						return true;
					}
					else//�γ��赲
					{
						if (carNext.status = FINESHED)
						{
							driverToNextRoad(car, idNextRoad, idNextLane, carNext.location - 1);//��ʻ���¸�·�ڣ�ǰ��֮��
							assert(carNext.location >= 1);
							return true;
						}

						//���ǰ�����ڵȴ�״̬����ô�˳�Ҳ����ʻ�������ȴ�
					}
				}
				else//����ó���û���赲
				{
					driverToNextRoad(car, idNextRoad, idNextLane, disNextRoad);//��ʻ���¸�·��
					return true;
				}
			}
		}
		//���Ŀ�공���޷�ʻ�룬����WAITTING״̬
	}
	return false;
}

void Scheduler::addCar(Car car, int i)
{
	assert(car.status == SLEEPING);//ֻ��SLEEPING״̬�ĳ����Լ����ͼ��ʻ
	int idRoadTarget = car.path[0];//��ȡĿ���·
	int idCrossTarget = car.idCrossFrom;//��øó�����·��
	int idLaneTarget = isCanEnter(idRoadTarget, idCrossTarget);
	if (idLaneTarget != -1)//����õ�·�ɼ��복
	{
		car.status = FINESHED;//�л�car��״̬
		car.idCurRoad = idRoadTarget;
		car.idCurLane = idLaneTarget;
		car.location = 0;
		car.dirCross = NONE;
		std::vector<int>::iterator itPath = car.path.begin();
		car.path.erase(itPath);//�Ѿ�ʻ����һ��·�ڣ�����ɾ��path�е�һ��
		int indexCar = roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.size();//�ó�Ϊĩβ
		roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.push_back(car);//���ó������Ӧ��·,��Ӧ����,�����β
		driveCar(car, indexCar);//car��ʻ indexCarΪ-1����ʾ�ó���lane�л�û��λ��
		cars[car.id - 10000].starttime = time_Scheduler;
		num_CarsPut += 1;
		//roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar - 1].starttime = time_Scheduler;
		//��¼ʵ�ʳ���ʱ��

	}
	else//�������ʧ�ܣ��򽫳���ʱ���Ӻ�һ��1��ʱ��Ƭ
	{
		cars[i].plantime += 1;
	}
}

int Scheduler::isCanEnter(int idRoad, int idCross)
{
	// ����������������
	//     3
	//  <�� �� �� ��
	//     2
	//��������������������
	//     0
	//  �� �� �� ��>
	//     1
	//��������������������
	int idStartLane = 0;//���crossΪ��·���뷽����Ҫ���� 0 1 2������������� 3 4 5����
	if (roads[idRoad - 5000].idTo == crosses[idCross - 1].id)//���crossΪ��·���뷽��
	{
		idStartLane = roads[idRoad - 5000].channel;
		assert(roads[idRoad - 5000].isDuplex == 1);//���crossΪroad�ĳ����򣬵���ȴ����˫�������ô�ܿ���·���滮����
		if (roads[idRoad - 5000].isDuplex != 1)
			return -1;
	}
	for (int j = idStartLane; j < idStartLane + roads[idRoad - 5000].channel; ++j)//��������lane
	{
		if (roads[idRoad - 5000].lane[j].laneCar.size() < roads[idRoad - 5000].length)
		{
			//���������⣬������������С�ڳ������ȣ��������һ�����п��ܶ������һλ
			if (roads[idRoad - 5000].lane[j].laneCar.size() > 0)
			{
				Car carNext = roads[idRoad - 5000].lane[j].laneCar[roads[idRoad - 5000].lane[j].laneCar.size() - 1];
				if (carNext.location == 0)
					return -1;
			}
			return j;//���ڿ�λ���ɼ���,���س���id
		}
	}
	return -1;//�����ڿ�λ
}

bool Scheduler::isBeDD(int idRoad, int idCross)
{
	if (idRoad == -1)//�����ͻ�����޵�·���������޳�ͻ����
		return true;
	int idStartLane = 0;//���crossΪ��·�ĳ�������Ҫ���� 0 1 2������������� 3 4 5����
	if (roads[idRoad - 5000].idTo == crosses[idCross-1].id)//���crossΪ��·�ĳ�����
		idStartLane = roads[idRoad - 5000].channel;
	/*
	for (int j = idStartLane; j < idStartLane + roads[idRoad - 5000].channel; ++j)//��������lane
	{
		if (roads[idRoad - 5000].lane[j].laneCar.size() != 0)
		{
			if (roads[idRoad - 5000].lane[j].laneCar[0].dirCross == DD)
				return true;//����ֱ�г���
		}
	}
	*/
	//ԭ����Ϊ��Ҫ�ж����г���������ֻ�ж����ȼ���ߵĳ���
	if (roads[idRoad - 5000].lane[idStartLane].laneCar.size() != 0)
	{
		if (roads[idRoad - 5000].lane[idStartLane].laneCar[0].dirCross == DD)
		{
			//ֻ�еȴ�״̬�ĳ����γɳ�ͻ
			if (roads[idRoad - 5000].lane[idStartLane].laneCar[0].status == WAITTING)
				return true;//������ת����
		}

	}
	return false;
}

bool Scheduler::isBeLEFT(int idRoad, int idCross)
{
	if (idRoad == -1)//�����ͻ�����޵�·���������޳�ͻ����
		return true;
	int idStartLane = 0;//���crossΪ��·�ĳ�������Ҫ���� 0 1 2������������� 3 4 5����
	if (roads[idRoad - 5000].idTo == crosses[idCross-1].id)//���crossΪ��·�ĳ�����
		idStartLane = roads[idRoad - 5000].channel;
	/*
	for (int j = idStartLane; j < idStartLane + roads[idRoad - 5000].channel; ++j)//��������lane
	{
		if (roads[idRoad - 5000].lane[j].laneCar.size() != 0)
		{
			if (roads[idRoad - 5000].lane[j].laneCar[0].dirCross == LEFT)
				return true;//������ת����
		}
	}
	*/
	//ԭ����Ϊ��Ҫ�ж����г���������ֻ�ж����ȼ���ߵĳ���
	if (roads[idRoad - 5000].lane[idStartLane].laneCar.size() != 0)
	{
		if (roads[idRoad - 5000].lane[idStartLane].laneCar[0].dirCross == LEFT)
		{
			//ֻ�еȴ�״̬�ĳ����γɳ�ͻ
			if(roads[idRoad - 5000].lane[idStartLane].laneCar[0].status == WAITTING)
				return true;//������ת����
		}
	}
	return false;
}

int Scheduler::getCrossDir(int idCurRoad, int idNextRoad, int idNextCross)
{
	int dirCurRoad = 0;//��ǰ��·��·�ڵķ���
	int dirNextRoad = 0;//����ʻ���·��·�ڵķ���
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
	int disCurRoad = roads[car.idCurRoad - 5000].length - car.location;//��ǰ��·������ʻ�ľ���
	int disNextRoadMax = std::min(roads[idNextRoad - 5000].speed, car.speed);//��һ��·������ʻ��������
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
	roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.erase(it);	//���ó��ӵ�ǰlaneɾ��,�ó��϶��ǵ�ǰlane�ĵ�һ����
	car.idCurRoad = idNextRoad;
	car.idCurLane = idNextLane;
	car.location = location;
	car.dirCross = NONE;//�ó�·��״̬����ΪNONE,�����Ѿ�ʻ��·��
	car.status = FINESHED;//�ó�������ɣ��ȴ���һʱ��Ƭ����ʻ
	std::vector<int>::iterator itPath = car.path.begin();
	car.path.erase(itPath);//�Ѿ�ʻ����һ��·�ڣ�����ɾ��path�е�һ��
	int size = roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.size();
	Lane lane = roads[car.idCurRoad - 5000].lane[car.idCurLane];
	roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.push_back(car);//���ó�������һ��lane,�����β
}

bool Scheduler::isCanDriveToNextRoad(Car car, int dir, int idCross)
{
	if (crosses[idCross-1].roadID[dir] != -1)
	{
		int idNextLane = isCanEnter(crosses[idCross - 1].roadID[dir], idCross);
		if (idNextLane != -1)
		{
			Car car_original= car;//��¼carת��֮ǰ��״̬
			bool result = driveCar(car, -1);//�ó�����ת�򣬵��ǲ�����ó�ת��󲻻���Ϊ�г��赲Ȼ��WAITTING
			//ʵ����driverCar������������������
			//1.�ɹ���ʻ���¸�·��
			//2.��Ϊ�¸�·���г�waitting,�˳��޷���ʻ
			//3.��Ϊ��ʻ��̲�����ֻ�ܼ�������·��
			//if (car_original.idCurRoad != car.idCurRoad || car_original.location != car.location)//road�����仯����location�����仯����Ϊת��ɹ�
			if(result)
				return true;
			//�����������Ϊ�ɹ����߲��ɹ����Խ��Ӧ��Ӱ�첻��
		}
	}
	return false;
}

void Scheduler::driveAllCarsJustOnOneRoadToEndState(int idRoad, int idCross)
{
	Road road = roads[idRoad - 5000];
	if (road.idTo = idCross)//�����·����road�ĳ�������ô����road��������
	{
		for (int j = 0; j < road.channel; ++j)
		{
			Lane lane = road.lane[j];
			for (int m = 0; m < lane.laneCar.size(); ++m)
			{
				driveCar(lane.laneCar[m], m);
			}
		}
	}
	else //����road�ķ�����
	{
		if (road.isDuplex)
		{
			for (int j = road.channel; j < 2 * road.channel; ++j)
			{
				Lane lane = road.lane[j];
				for (int m = 0; m < lane.laneCar.size(); ++m)
				{
					driveCar(lane.laneCar[m], m);
				}
			}
		}
	}
}

void Scheduler::driverCarInGarage()
{
	for (int i = 0; i < num_Cars; ++i)
	{
		if (cars[i].plantime == time_Scheduler&&cars[i].status==SLEEPING)
			addCar(cars[i],i);
	}
}

void Scheduler::putCarStatus(Car car)
{
	if(car.path.size()!=0)
		PRINT("CarID:%d  idFrom:%d  idTo:%d idCurRoad:%d  idCurLane:%d  location:%d  NextRoad:%d  Status:%d\n",
		car.id,car.idCrossFrom,car.idCrossTo,car.idCurRoad,car.idCurLane,car.location,car.path[0], car.status);//��ӡϵͳʱ��
	else
		PRINT("CarID:%d  idFrom:%d  idTo:%d idCurRoad:%d  idCurLane:%d  location:%d  Status:%d\n",
			car.id, car.idCrossFrom, car.idCrossTo, car.idCurRoad, car.idCurLane, car.location, car.status);//��ӡϵͳʱ��
}

void Scheduler::putAllCarStatus()
{
	for (int i = 0; i < num_Roads; ++i)//����·ID������е���
	{
		for (int j = 0; j < roads[i].channel * (1 + roads[i].isDuplex); ++j)
		{
			Lane lane = roads[i].lane[j];
			
			if (lane.laneCar.size() != 0)//���жϸó����Ƿ��г�
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
	for (int i = 0; i < num_Roads; ++i)
	{
		for (int j = 0; j < roads[i].channel * (1 + roads[i].isDuplex); ++j)
		{
			float per = (float)roads[i].lane[j].laneCar.size() / (float)roads[i].length;
			if(per > 0.5)
				PRINT("ROAD ID:%d  CHANNEL ID:%d  PERCENT:%f\n",roads[i].id,j,per);
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
	int dirFrom = -1;//���ó�ֵΪ-1�����ں���������
	for (int i = 0; i < 4; ++i)
	{
		if (crosses[idCross - 1].roadID[i] == idRoad)
			dirFrom = i;
	}
	assert(dirFrom != -1);//���δ�ڸ�cross�ҵ���road���򱨴�

	return dirFrom;
}

void Scheduler::driveAllCarsJustOnOneChannelToEndState(int idRoad, int idCross, int idChannel)
{
	if (roads[idRoad - 5000].lane[idChannel].laneCar.size() != 0)
	{
		for (int i = 0; i < roads[idRoad - 5000].lane[idChannel].laneCar.size(); ++i)
		{
			if (roads[idRoad - 5000].lane[idChannel].laneCar[i].status == WAITTING)//ֻ����ȴ�״̬�ĳ�
			{
				Car car = roads[idRoad - 5000].lane[idChannel].laneCar[i];
				if (car.location + std::min(roads[idRoad - 5000].speed, car.speed) <= roads[idRoad - 5000].length)//����ʻ��·��
				{
					//ֻ������ʻ��ͨ��·�ڵĳ�
					if (i != 0)//����ó����ǵ�һ����
					{
						Car carNext = roads[idRoad - 5000].lane[idChannel].laneCar[i - 1];
						if (car.location + std::min(roads[idRoad - 5000].speed, car.speed) < carNext.location)
						{
							//ǰ�����γ��赲
							roads[idRoad - 5000].lane[idChannel].laneCar[i].location += std::min(roads[idRoad - 5000].speed, car.speed);//��������ʻ
							roads[idRoad - 5000].lane[idChannel].laneCar[i].status = FINESHED;//�����Ϊ��ֹ״̬
							roads[idRoad - 5000].lane[idChannel].laneCar[i].dirCross = NONE;
						}
						else if (carNext.status == FINESHED)
						{
							roads[idRoad - 5000].lane[idChannel].laneCar[i].location = carNext.location - 1;//��������ʻ
							roads[idRoad - 5000].lane[idChannel].laneCar[i].status = FINESHED;//�����Ϊ��ֹ״̬
							roads[idRoad - 5000].lane[idChannel].laneCar[i].dirCross = NONE;
						}
					}
					else
					{
						//ǰ�����γ��赲
						roads[idRoad - 5000].lane[idChannel].laneCar[i].location += std::min(roads[idRoad - 5000].speed, car.speed);//��������ʻ
						roads[idRoad - 5000].lane[idChannel].laneCar[i].status = FINESHED;//�����Ϊ��ֹ״̬
						roads[idRoad - 5000].lane[idChannel].laneCar[i].dirCross = NONE;
					}
				}
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

	Graph_DG graph(vexnum, edge);
	graph.createArcGraph(tmp);
	graph.createArcRoadvGraph(tmp1);

	for (int i = 0; i < num_Cars; ++i)
	{
		vector<int> pathCross = graph.Dijkstra(cars[i].idCrossFrom, cars[i].idCrossTo, cars[i].speed);
		vector<int> pathRoad(pathCross.size() - 1);
		for (int j = 0; j < pathRoad.size(); ++j)
		{
			pathRoad[j] = graphC2R[pathCross[j] - 1][pathCross[j + 1] - 1];
			//assert(pathRoad[j] != 0);
		}
		cars[i].path = pathRoad;
	}

}
