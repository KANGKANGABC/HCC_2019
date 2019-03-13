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
	this->lane = new Lane[(1 + this->isDuplex) * this->channel];
	for (int i = 0; i < this->channel; ++i)
	{
		lane[i].dir = FORWARD;//����
		lane[i].idLane = i;//id���������������ԭ��������ϵ��ȵ�˳��
		//lane[i].laneCar.resize(this->length);
	}
	if (this->isDuplex)
	{
		for (int i = this->channel; i < 2 * this->channel; ++i)
		{
			lane[i].dir = BACKWARD;//����
			lane[i].idLane = i - this->channel;//id���������������ԭ��������ϵ��ȵ�˳��
			//lane[i].laneCar.resize(this->length);
		}
	}
}
