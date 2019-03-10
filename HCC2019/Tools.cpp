#include "Tools.h"

//×Ö·û´®·Ö¸îº¯Êý
std::vector <std::string> Tools::split(std::string str, std::string pattern)
{
	std::string::size_type pos;
	std::vector<std::string> result;

	str += pattern;//À©Õ¹×Ö·û´®ÒÔ·½±ã²Ù×÷  
	int size = str.size();

	for (int i = 0; i < size; i++)
	{
		pos = str.find(pattern, i);
		if (pos < size)
		{
			std::string s = str.substr(i, pos - i);
			result.push_back(s);
			i = pos + pattern.size() - 1;
		}
	}
	return result;
}