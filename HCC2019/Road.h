#pragma once
#include "define.h"

class Car
{
private:
	int id;
	int location;
	int status;
	int speed;
	int plantime;
	int idCurRoad;//��ǰ�����ڵĵ�·ID
	int idCurLane;//��ǰ�����ڵĳ���ID
	int dirCross;//��־��·�ڵ�״̬ 
	int idCrossFrom;//���ĳ���·��
	int idCrossTo;//������ֹ·��
	std::vector<int> path;

public:
	//Ĭ�Ϲ��캯��
	Car() {};
	//���ع��캯����ֱ�ӽ�6����������
	Car(int id, int idCrossFrom, int idCrossTo, int speed, int plantime, int status) :
		id(id), idCrossFrom(idCrossFrom), idCrossTo(idCrossTo), speed(speed), plantime(plantime), status(status) {}
	~Car() {};
	//���car������
	int getID() { return id; };
	int getIdForm() { return idCrossFrom; };
	int getIdTo() { return idCrossTo; };
	int getPlaneTime() { return plantime; };
	int getLocation() { return location; };
	int getStatus() { return status; };
	int getSpeed() { return speed; };
	std::vector<int> getPath() { return path; };
	int getIdCurRoad() { return idCurRoad; };
	int getIdCurLane() { return idCurLane; };
	int getDirCross() { return dirCross; };
	int getPathSize() { return path.size(); };
	//�޸�car������
	void putID(int n_id) { id = n_id; };
	void putIdForm(int n_idfrom) { idCrossFrom = n_idfrom; };
	void putIdTo(int n_idto) { idCrossTo = n_idto; };
	void putPlaneTime(int n_plantime) { plantime = n_plantime; };
	void putLocation(int n_location) { location = n_location; };
	void putStatus(int n_status) { status = n_status; };
	void putSpeed(int n_speed) { speed = n_speed; };
	void putPath(std::vector<int> n_path) { path = n_path; };
	void putIdCurRoad(int n_idcurroad) { idCurRoad = n_idcurroad; };
	void putIdCurLane(int n_idcurlane) { idCurLane = n_idcurlane; };
	void putDirCross(int n_dircross) { dirCross = n_dircross;  };
};

class Cross
{
public:
	int id;
	int roadID_T; //˳ʱ���1�� Top
	int roadID_R; //˳ʱ���2�� Right
	int roadID_D; //˳ʱ���3�� Down
	int roadID_L; //˳ʱ���4�� Left
	std::vector<int> roadID;//��˳��洢�����ĸ����򣬱��ڱ���

};

class Lane
{
public:
	bool dir;//��������
	int idLane;//����ID
	std::vector<Car> laneCar;//��¼ÿ�������ϵĳ�����Ϣ

};
class Road
{
public:
	Road();
	~Road();

	//����������Ϣ
	void CreateLane();

	int id;
	int length;
	int speed;
	int channel;
	int idFrom;
	int idTo;
	int isDuplex;
	Lane *lane;
};