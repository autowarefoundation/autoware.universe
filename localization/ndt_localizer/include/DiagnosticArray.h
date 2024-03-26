#pragma once
#include <vector>
#include <chrono>
#include "_DiagnosticStatus.h"
using namespace std;
struct Header
{
	int32_t seq;		  // 消息序列号
	std::string frame_id; // 参考坐标系ID

	// 使用C++11的chrono库来表示时间戳
	std::chrono::time_point<std::chrono::system_clock> stamp;
};
struct DiagnosticArray
{
	Header header;
	std::vector<DiagnosticStatus> status;
};