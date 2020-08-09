#include "PathFinding.h"


/*
�����ߣ������
ѧУ���������պ����ѧ
�绰��15910922135
���䣺515143675@qq.com
������ڣ�2020��8��4��
visual studio�汾��Visual Studio 2019��v142�� Debug x86
*/

// start �� end ���ɴ�ʱ����Ӧ�õ���IWaypointList::Add��
// ��֮����Ҫ��start��ʼ����������������ڵ�waypoint��ֱ��end������Ҫ����start��end��
// �����ڡ� ָ�������Ӵ��ڹ����߻򹫹�����


//��ʼ��NodeFactory�����ڼ��䱣��ľ�̬����
	//�洢���������
unordered_map<int, unordered_map<int, unordered_map<int, unordered_map<int, vector<Position>>>>> PathBuilder::routing_memory;
	//�������Ŀ
int PathBuilder::memory_nums = 0;
	//ָ������������
int PathBuilder::max_mem_nums = 999;
	
extern "C" __declspec(dllexport) void __cdecl FindPath(Grid const & grid,
  Position const & start, Position const & end,
  IWaypointList & waypointList) 
{
	//����յ��غ�ʱ,������
	if (start.column == end.column && start.row == end.row)
		return;

	//��㲻�ɴ�ʱ��������
	if (grid.IsWalkable(start) == false)
		return;

	/*��Ѱ·�㷨����˫��A*Ѱ·����ϸ˵������PathFinding.h�ļ��е����ע��*/

	//�������·������·�㼯�ϵ�·�������߶���
	PathBuilder builder(grid,start,end);
		//����·�����书�ܣ����������ڴ滻ȡ�ٶȵ�����
	builder.use_memory = true;
	/*
	��������ʽ����ϵ��ѡֵ�����ۣ�
		f=g*g_d+h*h_d
		����gΪ·�����ۣ�hΪ���յ�Ĵ��۹��ƣ�ȡ�б�ѩ�����
		�������ϸ�ڲο�make_node����
		����ʽ������������A*�㷨����Ϊ��
		1.��һ�ּ�������£����h_d(�յ������ϵ��)Ϊ0����ôֻ��g(·������)��A*�ͱ����Dijkstra�㷨����ʱ�㷨�ܻ��ҵ�һ�����·�������Ǵ�ʱ�㷨�ٶ�������
		2.���h_dȡС��1��ֵ����ʱg*g_d������ֵ�б���������ôA*���ҵ�һ���϶�·�����ϵ͵�h_d*h�ᵼ��ÿһ��A*�ڵ��н϶�Ĵ�ѡ��չ�㣬ʹ�㷨��ý�����
		3.����һ����������£����h*h_d�����g*g_d�ǳ��ĸߣ��൱��ֻ��h*h_d��A*�ͱ���������������,��ʱ�㷨���Կ���ѡ����һ����չ�㣬�㷨�ٶ���죬����·���ϳ���
		Ϊ��׷�����·�����ҽ�h_d��Ϊ��С����ֵ����ʱ�Ľ���ӽ����������е�·��������������̡�
	*/
		//��������ʽ����ϵ��
		//��ǰ����ȡ���˲���Ľ��
	builder.g_d = 1.0;	//·��������ϵ��
	builder.h_d = 0.1;	//�����յ����Ԥ����ϵ��

	//���waypointList
	builder.fill_path(waypointList);
}
