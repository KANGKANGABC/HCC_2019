#include "Road.h"



Road::Road()
{
	/*
	if (this->isDuplex == 1)
	{
		this->lane = new Lane[2 * this->channel];
		for (int i = 0; i < this->channel; ++i)
		{
			lane[i].dir = 0;//正向
			lane[i].laneCarId.resize(this->length);
		}
		for (int i = this->channel; i < 2*this->channel; ++i)
		{
			lane[i].dir = 1;//逆向
			lane[i].laneCarId.resize(this->length);
		}
	}
	else
	{
		this->lane = new Lane[this->channel];
		for (int i = 0; i < this->channel; ++i)
		{
			lane[i].dir = 0;//正向
			lane[i].laneCarId.resize(this->length);
		}
	}
	*/
}


Road::~Road()
{
	delete[] this->lane;
}

void Road::CreateLane()
{
	if (this->isDuplex == 1)
	{
		this->lane = new Lane[2 * this->channel];
		for (int i = 0; i < this->channel; ++i)
		{
			lane[i].dir = 0;//正向
			lane[i].laneCarId.resize(this->length);
		}
		for (int i = this->channel; i < 2 * this->channel; ++i)
		{
			lane[i].dir = 1;//逆向
			lane[i].laneCarId.resize(this->length);
		}
	}
	else
	{
		this->lane = new Lane[this->channel];
		for (int i = 0; i < this->channel; ++i)
		{
			lane[i].dir = 0;//正向
			lane[i].laneCarId.resize(this->length);
		}
	}
}
