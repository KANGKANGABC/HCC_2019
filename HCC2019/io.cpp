#include "pch.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <time.h>
#include <sys/timeb.h>
#include <errno.h>
#include <signal.h>
#include <signal.h>
#include "define.h"

#define MAX_LINE_LEN 500

#define INLINE  static __inline

INLINE void write_file(const bool cover, const char * const buff, const char * const filename);

int read_file(char ** const buff, const unsigned int spec, const char * const filename)
{
	FILE *fp = fopen(filename, "r");
	if (fp == NULL)
	{
		PRINT("Fail to open file %s, %s.\n", filename, strerror(errno));
		return 0;
	}
	PRINT("Open file %s OK.\n", filename);

	char line[MAX_LINE_LEN + 2];
	unsigned int cnt = 0;
	while ((cnt < spec) && !feof(fp))
	{
		line[0] = 0;
		if (fgets(line, MAX_LINE_LEN + 2, fp) == NULL)  continue;
		if (line[0] == 0)   continue;
		buff[cnt] = (char *)malloc(MAX_LINE_LEN + 2);
		strncpy(buff[cnt], line, MAX_LINE_LEN + 2 - 1);
		buff[cnt][MAX_LINE_LEN + 1] = 0;
		cnt++;
	}
	fclose(fp);
	PRINT("There are %d lines in file %s.\n", cnt, filename);

	return cnt;
}

void write_result(const char * const buff, const char * const filename)
{
	// 以覆盖的方式写入
	write_file(1, buff, filename);

}

void release_buff(char ** const buff, const int valid_item_num)
{
	for (int i = 0; i < valid_item_num; i++)
		free(buff[i]);
}

INLINE void write_file(const bool cover, const char * const buff, const char * const filename)
{
	if (buff == NULL)
		return;

	const char *write_type = cover ? "w" : "a";//1:覆盖写文件，0:追加写文件
	FILE *fp = fopen(filename, write_type);
	if (fp == NULL)
	{
		PRINT("Fail to open file %s, %s.\n", filename, strerror(errno));
		return;
	}
	PRINT("Open file %s OK.\n", filename);
	fputs(buff, fp);
	fputs("\n", fp);
	fclose(fp);
}