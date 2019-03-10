#pragma once
#ifndef TOOLS_H_
#define TOOLS_H_

#include "define.h"

class Tools
{
public:
	static std::vector<std::string> split(std::string str, std::string pattern);
};

#endif