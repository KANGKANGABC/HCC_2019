#include "Road.h"



Road::Road()
{

}


Road::~Road()
{
	delete[] this->lane;
}

void Road::CreateLane()
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
	this->lane = new Lane[(1 + this->isDuplex) * this->channel];
	for (int i = 0; i < this->channel; ++i)
	{
		lane[i].dir = FORWARD;//����
		lane[i].idLane = i;//id���������������ԭ��������ϵ��ȵ�˳��
	}
	if (this->isDuplex)
	{
		for (int i = this->channel; i < 2 * this->channel; ++i)
		{
			lane[i].dir = BACKWARD;//����
			lane[i].idLane = i;//id���������������ԭ��������ϵ��ȵ�˳��
		}
	}
}
