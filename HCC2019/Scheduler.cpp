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
	Lane lane = roads[0].lane[0];
	time_Scheduler = 0;//��������ʼʱ������Ϊ0
}


Scheduler::~Scheduler()
{
}

int Scheduler::getSysTime()
{
	while (num_CarsScheduling > 0)
	{
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
					for (int j = 0; j < 4; ++j)
					{
						if (crosses[i].roadID[j] != -1)
						{
							int idRoad = crosses[i].roadID[j];//�����ȵĵ�·id
							int idStartLane = 0;//���crossΪ��·�ĳ�������Ҫ���� 0 1 2������������� 3 4 5����
							if (roads[idRoad - 5000].idTo == crosses[i].id)//���crossΪ��·�ĳ�����
								idStartLane = roads[idRoad - 5000].channel;
							while (1)
							{
								bool isWorkingLane = false;
								for (int m = idStartLane; m < idStartLane + roads[idRoad - 5000].channel; ++m)//��������lane
								{
									if (roads[idRoad - 5000].lane[m].laneCar.size() != 0)
									{
										Car car = roads[idRoad - 5000].lane[m].laneCar[0];
										int dirConflict = 0;
										int dirTarget = 0;
										switch (roads[idRoad - 5000].lane[m].laneCar[0].getDirCross())
										{
										case NONE:
											break;
										case DD://ֱ��>��ת>��ת
											dirTarget = j + 2;//Ŀ�귽��
											if (dirTarget < 0) dirTarget += 4;//��������
											if (isCanDriveToNextRoad(car, dirTarget, idCross))
											{
												isWorkingCross = true;
												isWorkingRoad = true;
												isWorkingLane = true;
											}
										   //�ж�ת���road�Ƿ������ʻ

											break;
										case LEFT://��ת>��ת
											//�жϼ���ת��ķ����Ƿ���ֱ�н���ĳ���
											dirConflict = j - 1;//��ͻ����
											if (dirConflict < 0) dirConflict += 4;//��������
											if (!isBeDD(crosses[i].roadID[dirConflict], i))
											{
												dirTarget = j + 1;//Ŀ�귽��
												if (dirTarget > 3) dirTarget -= 4;//��������
												if (isCanDriveToNextRoad(car, dirTarget, idCross))//�ж�ת���road�Ƿ������ʻ
												{
													isWorkingCross = true;
													isWorkingRoad = true;
													isWorkingLane = true;
												}
											}
											break;
										case RIGHT://��ת���ȼ����
											//�жϼ���ת��ķ����Ƿ���ֱ�н���ĳ���
											dirConflict = j + 1;//��ͻ����
											if (dirConflict > 3) dirConflict -= 4;//��������
											if (!isBeDD(crosses[i].roadID[dirConflict], i))
											{
												dirConflict = j + 2;//��ͻ����
												if (dirConflict > 3) dirConflict -= 4;//��������
												//�жϼ���ת��ķ����Ƿ�����ת����ĳ���
												if (!isBeLEFT(crosses[i].roadID[dirConflict], i))
												{
													dirTarget = j - 1;//Ŀ�귽��
													if (dirTarget < 0) dirTarget += 4;//��������
													if (isCanDriveToNextRoad(car, dirTarget, idCross))//�ж�ת���road�Ƿ������ʻ
													{
														isWorkingCross = true;
														isWorkingRoad = true;
														isWorkingLane = true;
													}
												}
											}
											break;
										default:
											break;
										}
									}
								}
								if (!isWorkingLane)
									break;
								else
								{
									//����ô˵����г���·�ڣ���ô�ø�Road������lane��ʻ
									driveAllCarsJustOnOneRoadToEndState(idRoad, idCross);
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
			Lane lane = roads[i].lane[j];
			if (lane.laneCar.size() != 0)//���жϸó����Ƿ��г�
			{
				for (int m = 0; m < lane.laneCar.size(); ++m)
				{
					driveCar(lane.laneCar[m], m);
				}
			}
		}
	}

}

void Scheduler::driveCar(Car car, int indexCar)
{
	//�������AA�����ĳ������·��ʻ����һ��·����������һ��ʱ��Ƭ��ʻ����һ��·ȫ��
	//Ҳ����˵ֻ�д���NONE״̬WAITTING�ĳ����п��ܼ��������յ�
	if (car.getDirCross() == NONE)//�ó�������·�ڵȴ�
	{
		if (indexCar == 0)//�ó�Ϊ�ó����ĵ�һ���������ϸ�ʱ��Ƭ��׼��ͨ��·��
		{
			//�жϴ˳��᲻��ͨ��·��
			if (car.getLocation + std::min(roads[car.getIdCurRoad()].speed, car.getSpeed()) <= roads[car.getIdCurRoad()].length)//����ʻ��·��
			{
				car.putLocation ( car.getLocation() + std::min(roads[car.getIdCurRoad()].speed, car.getSpeed()) );//��������ʻ
				car.putStatus(FINESHED);//�����Ϊ��ֹ״̬
			}
			else
			{
				//�˳�Ҳ����ʻ��·�ڣ���Ҫ�жϴ˳���·�ڵķ���
				int idNextCross = roads[car.getIdCurRoad() - 5000].idTo;//�˳�����ʻ���·��
				//���ݼ���AA����ʱ�����г���ʻ���յ�
				if (idNextCross = car.getIdTo())//����˳���Ҫʻ������
				{
					num_CarsScheduling -= 1;//���ڵ��ȵĳ�������һ
					std::vector<Car>::iterator it = roads[car.getIdCurRoad()].lane[car.getIdCurLane()].laneCar.begin();
					roads[car.getIdCurRoad()].lane[car.getIdCurLane()].laneCar.erase(it);//ɾ���õ�·��һ����
				}
				else
				{
					int idNextRoad = car.path[car.path.size() - 1];//�˳�����ʻ��ĵ�·
					int idCurRoad = car.getIdCurRoad();//�˳���ǰ��·
					car.putDirCross(getCrossDir(idCurRoad, idNextRoad, idNextCross));//����·�ڷ���
					car.putStatus(WAITTING);//�˳���Ϊ�ȴ�״̬
				}
			}
		}
		else//�ó�ǰ���г�
		{
			Car carNext = roads[car.getIdCurRoad()].lane[car.getIdCurLane()].laneCar[indexCar - 1];
			if (car.getLocation() + std::min(roads[car.getIdCurRoad()].speed, car.getSpeed()) < carNext.getLocation())
			{//ǰ��ĳ����γ��赲
				car.putLocation( car.getLocation() + std::min(roads[car.getIdCurRoad()].speed, car.getSpeed()));//��������ʻ
				car.putLocation(FINESHED);//�����Ϊ��ֹ״̬
			}
			else
			{//ǰ��ĳ��γ��赲
				//�жϴ˳��᲻��ͨ��·��
				if (car.getLocation() + std::min(roads[car.getIdCurRoad()].speed, car.getSpeed()) <= roads[car.getIdCurRoad()].length)//����ʻ��·��
				{
					if (carNext.getStatus() == FINESHED)
					{
						car.putLocation(roads[car.getIdCurRoad()].lane[car.getIdCurLane()].laneCar[indexCar - 1].getLocation() - 1);//��ʻ��ǰ���ĺ�һ��λ��
						car.putStatus(FINESHED);//�����Ϊ��ֹ״̬
					}
					else
					{
						car.putStatus(WAITTING);//�����ΪWAITTING
					}
				}
				else
				{
					//�˳�Ҳ����ʻ��·��,��ô�жϴ˳���·�ڵķ���
					int idNextCross = roads[car.getIdCurRoad() - 5000].idTo;//�˳�����ʻ���·��
					//���ݼ���AA����ʱ�����г���ʻ���յ�
					if (idNextCross = car.getIdTo())//����˳���Ҫʻ������
					{
						num_CarsScheduling -= 1;//���ڵ��ȵĳ�������һ
						std::vector<Car>::iterator it = roads[car.getIdCurRoad()].lane[car.getIdCurLane()].laneCar.begin();
						roads[car.getIdCurRoad()].lane[car.getIdCurLane()].laneCar.erase(it);//ɾ���õ�·��һ����
					}
					else
					{
						int idNextRoad = car.path[car.path.size() - 1];//�˳�����ʻ��ĵ�·
						int idCurRoad = car.getIdCurRoad();//�˳���ǰ��·
						car.putDirCross(getCrossDir(idCurRoad, idNextRoad, idNextCross));//����·�ڷ���
						car.putStatus(WAITTING);//�˳���Ϊ�ȴ�״̬
					}
				}
			}
		}
	}
	else//�ó�׼��ʻ��·��
	{
		int idNextRoad = car.path[car.path.size() - 1];//��ȡĿ���·
		int idNextCross = roads[car.idCurRoad - 5000].idTo;//��øó�����·��
		if (int idNextLane = isCanEnter(idNextRoad, idNextCross) != -1)//����õ�·�ɼ��복
		{
			int disNextRoad = getCrossDistance(car, car.idCurRoad, idNextRoad);
			if (disNextRoad == 0)//����ʻ����Ϊ0����ͣ�ڵ�ǰ·��
			{
				car.location = roads[car.idCurRoad].length;
				car.status = FINESHED;//�ó�������ɣ��ȴ���һʱ��Ƭ����ʻ
			}
			else
			{
				if (roads[idNextRoad].lane[idNextLane].laneCar.size() != 0)//����ó����г�
				{
					//�ж϶�Ӧ��������λ��
					Car carNext = roads[idNextRoad].lane[idNextLane].laneCar[roads[idNextRoad].lane[idNextLane].laneCar.size() - 1];
					if (disNextRoad < carNext.location)//���γ��赲
					{
						driverToNextRoad(car, idNextRoad, idNextLane, disNextRoad);//��ʻ���¸�·��
					}
					else//�γ��赲
					{
						if (carNext.status = FINESHED)
							driverToNextRoad(car, idNextRoad, idNextLane, carNext.location - 1);//��ʻ���¸�·�ڣ�ǰ��֮��
						//���ǰ�����ڵȴ�״̬����ô�˳�Ҳ����ʻ�������ȴ�
					}
				}
				else//����ó���û���赲
				{
					int disNextRoad = getCrossDistance(car, car.idCurRoad, idNextRoad);
					driverToNextRoad(car, idNextRoad, idNextLane, disNextRoad);//��ʻ���¸�·��
				}
			}
		}
		//���Ŀ�공���޷�ʻ�룬����WAITTING״̬
	}
}

void Scheduler::addCar(Car car)
{
	assert(car.status == SLEEPING);//ֻ��SLEEPING״̬�ĳ����Լ����ͼ��ʻ
	int idRoadTarget = car.path[car.path.size() - 1];//��ȡĿ���·
	int idCrossTarget = car.idCrossFrom;//��øó�����·��
	if (isCanEnter(idRoadTarget, idCrossTarget) != -1)//����õ�·�ɼ��복
	{
		car.status = WAITTING;//�л�car��״̬
		driveCar(car, -1);//car��ʻ indexCarΪ-1����ʾ�ó���lane�л�û��λ��
	}
}

int Scheduler::isCanEnter(int idRoad, int idCross)
{
	int idStartLane = 0;//���crossΪ��·�ĳ�������Ҫ���� 0 1 2������������� 3 4 5����
	if (roads[idRoad - 5000].idFrom == crosses[idCross].id)//���crossΪ��·���뷽��
		idStartLane = roads[idRoad - 5000].channel;
	for (int j = idStartLane; j < idStartLane + roads[idRoad - 5000].channel; ++j)//��������lane
	{
		if (roads[idRoad - 5000].lane[j].laneCar.size() < roads[idRoad - 5000].length)
		{
			return j;//���ڿ�λ���ɼ���,���س���id
		}
	}
	return -1;//�����ڿ�λ
}

bool Scheduler::isBeDD(int idRoad, int idCross)
{
	int idStartLane = 0;//���crossΪ��·�ĳ�������Ҫ���� 0 1 2������������� 3 4 5����
	if (roads[idRoad - 5000].idTo == crosses[idCross].id)//���crossΪ��·�ĳ�����
		idStartLane = roads[idRoad - 5000].channel;
	for (int j = idStartLane; j < idStartLane + roads[idRoad - 5000].channel; ++j)//��������lane
	{
		if (roads[idRoad - 5000].lane[j].laneCar.size() != 0)
		{
			if (roads[idRoad - 5000].lane[j].laneCar[0].dirCross == DD)
				return true;//����ֱ�г���
		}
	}
	return false;
}

bool Scheduler::isBeLEFT(int idRoad, int idCross)
{
	int idStartLane = 0;//���crossΪ��·�ĳ�������Ҫ���� 0 1 2������������� 3 4 5����
	if (roads[idRoad - 5000].idTo == crosses[idCross].id)//���crossΪ��·�ĳ�����
		idStartLane = roads[idRoad - 5000].channel;
	for (int j = idStartLane; j < idStartLane + roads[idRoad - 5000].channel; ++j)//��������lane
	{
		if (roads[idRoad - 5000].lane[j].laneCar.size() != 0)
		{
			if (roads[idRoad - 5000].lane[j].laneCar[0].dirCross == LEFT)
				return true;//������ת����
		}
	}
	return false;
}

int Scheduler::getCrossDir(int idCurRoad, int idNextRoad, int idNextCross)
{
	int dirCurRoad = 0;//��ǰ��·��·�ڵķ���
	int dirNextRoad = 0;//����ʻ���·��·�ڵķ���
	if (crosses[idNextCross].roadID_T == idCurRoad)
		dirCurRoad = 0;
	else if (crosses[idNextCross].roadID_R == idCurRoad)
		dirCurRoad = 1;
	else if (crosses[idNextCross].roadID_D == idCurRoad)
		dirCurRoad = 2;
	else if (crosses[idNextCross].roadID_L == idCurRoad)
		dirCurRoad = 3;

	if (crosses[idNextCross].roadID_T == idNextRoad)
		dirNextRoad = 0;
	else if (crosses[idNextCross].roadID_R == idNextRoad)
		dirNextRoad = 1;
	else if (crosses[idNextCross].roadID_D == idNextRoad)
		dirNextRoad = 2;
	else if (crosses[idNextCross].roadID_L == idNextRoad)
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
	int disNextRoadMax = std::min(roads[idNextRoad - 5000].length, car.speed);//��һ��·������ʻ��������
	if (disCurRoad >= disNextRoadMax)
		return 0;
	else
		return disNextRoadMax - disCurRoad;
}

void Scheduler::driverToNextRoad(Car car, int idNextRoad, int idNextLane, int location)
{
	std::vector<Car>::iterator it = roads[car.idCurRoad].lane[car.idCurLane].laneCar.begin();
	roads[car.idCurRoad].lane[car.idCurLane].laneCar.erase(it);	//���ó��ӵ�ǰlaneɾ��,�ó��϶��ǵ�ǰlane�ĵ�һ����
	car.idCurRoad = idNextRoad;
	car.idCurLane = idNextLane;
	car.location = location;
	car.dirCross = NONE;//�ó�·��״̬����ΪNONE,�����Ѿ�ʻ��·��
	car.status = FINESHED;//�ó�������ɣ��ȴ���һʱ��Ƭ����ʻ
	roads[car.idCurRoad].lane[car.idCurLane].laneCar.push_back(car);//���ó�������һ��lane,�����β
}

bool Scheduler::isCanDriveToNextRoad(Car car, int dir, int idCross)
{
	if (crosses[idCross].roadID[dir] != -1)
	{
		if (int idNextLane = isCanEnter(crosses[idCross].roadID[dir] - 5000, idCross))
		{
			Car car_original= car;//��¼carת��֮ǰ��״̬
			driveCar(car, -1);//�ó�����ת�򣬵��ǲ�����ó�ת��󲻻���Ϊ�г��赲Ȼ��WAITTING
			//ʵ����driverCar������������������
			//1.�ɹ���ʻ���¸�·��
			//2.��Ϊ�¸�·���г�waitting,�˳��޷���ʻ
			//3.��Ϊ��ʻ��̲�����ֻ�ܼ�������·��
			if (car_original.idCurRoad != car.idCurRoad || car_original.location != car.location)//road�����仯����location�����仯����Ϊת��ɹ�
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
			addCar(cars[i]);
	}
}

void Scheduler::getPath()
{
	for (int i = 0; i < num_Cars; ++i)
	{
		if (i == 0)//Ϊ�˱��ڲ��Ե������߼�������һ�����滮һ����·��
		{
			cars[i].path = { 5029,5040,5051,5057,5058,5059 };
		}

	}
}
