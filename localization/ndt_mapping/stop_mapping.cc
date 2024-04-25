#include<fstream>
#include<iostream>
using namespace std;
int main()
{
	fstream f;
	f.open("./stop_single.txt",ios::out);
	//输入你想写入的内容 
	f<<"s";
	f.close();

    std::cout << "stop mapping!" << std::endl;

	return 0;
}
