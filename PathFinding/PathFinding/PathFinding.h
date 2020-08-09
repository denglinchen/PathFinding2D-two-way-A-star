#ifndef IKPGAJDGHOFEKABFDAKEFLIFBAEBBKAK
#define IKPGAJDGHOFEKABFDAKEFLIFBAEBBKAK

#include <cstdlib>
#include <cstdint>
#include<math.h>
#include<vector>
#include<unordered_map>
#include<algorithm>
#include<queue>
using namespace std;

class Position
{
public:
  std::size_t row;
  std::size_t column;
};

class Grid
{
public:
  std::uint8_t const * walkability;
  std::size_t numRows;
  std::size_t numColumns;

  bool IsWalkable(Position const & pos) const
  {
    return 0 == walkability[pos.row * numColumns + pos.column];
  }
};

class IWaypointList
{
public:
  virtual ~IWaypointList()
  {

  }

  virtual void Add(Position const &) = 0;
};

/////////////////////////////////////////////////////////////////////////////////////////
/*                                  ***�����ܽ�***
�����ߣ������
ѧУ���������պ����ѧ
�绰��15910922135
���䣺515143675@qq.com
������ڣ�2020��8��4��
visual studio�汾��Visual Studio 2019��v142�� Debug x86
	
	��Ѱ·�㷨����˫��A*Ѱ·�㷨���ȴ�ͳ��A*�㷨�ٶȸ��죬����ջ�ռ��С�������������ԡ�
���һ�������·�����书�ܣ���һ������㷨�ٶȡ�
	����Ҫ�����������(Node��PathBuilder)������ʵ��Ѱ·�㷨�����Ҹ������Լ�����������һ
Щ�Ż�����չ��ƣ�ʹ�ø��㷨ʹ�����������������Ը���ʵ��ҵ��������е�����
	�������Ҫ���ֽ���һЩ���ܡ�
	
	Node�ࣺ
	������Ҫ���һ���ܱ�������λ��Position�ͼ���A*����ʽ�����Ϣ��·���ࡪ��Node��������
Ҫ�������Ϣ������·���λ��(Position position)����¼�Ƿ�����������յ�࿪���Ĳ���ֵ��
ָ��ǰһ��·���ָ��(Node* parent)���������յ�����ֵ(float f)��
	
	PathBuilder�ࣺ
	�������㷨�ĺ��ģ��㷨��ʵ���Լ��������еĸ������ܶ��ɸô�������Ķ���ȥʵ�ֵġ���ô
FindPath�������о�ֻ�м��м򵥵Ĵ����ˡ��ô�������Ҫ�������֡���Ʒ����·��Node�����·����
��vector<Position>�����е�·����������һ���������ֿ⡱(static unoredered_map)����ȥ���档

	·����make_node��Ա����������make_node���������ж�Ӧ�ý���·�������һ��Ŀ��������һ�
��֤��ÿһ��Position�ڶ��ڴ���ֻ��һ��·������Ӧ������·��ļ���״̬������ֵҲ�����ڸú�
���м���õ��ġ�

	add_serround��Ա����ͨ������make_node��������ǰ·�����Χ·������Ӧ�Ŀ����ϡ����Ҽ��
�ڽ���·����뿪����ʱ�Ƿ������࿪���ϲ����˳�ͻ����������˳�ͻ��˵������·�����յ��·
���Ѿ���ϣ����Խ���·�����ӣ������ٽ��к��������ˡ�

	get_next��������ѡ����һ��ǰ����·�㣬���ҽ���ѡ���·��Ӷ�Ӧ�Ŀ�������ȥ�������ڿ�����
������multiset<Node*, heuristic>���������ݽṹ����˲����ɾ��·��ĸ��Ӷȶ���logN����Ҳ��һ
��СС���Ż���

	make_path�������Խ�������յ�����ɵ������������·����������γ���ɵ�·�����������
���书����Ὣ��������ֿ⣬���´β�ѯʹ�á�

	fill_path������Ϊ�㷨����Ҫ��ܺ�ʵ�֡�

	
	����ר�Ž���һ��·�����书�ܵ�ʵ�֣�
	һ��·������Ϊһ��vector<Position>���󣬴����һ����ά��unordered_map����routing_memory�У�
����start��end��row��columnֵ����Ϊ�ĸ�key��ȡ��routing_memory��NodeFactory��һ��static����
���������ڶ�����NodeFactory���󣬰������������������ڣ������ÿ��FindPath�����õ�ʱ����ǰ��
���䶼���ڣ���find_memory��ѯ������·����ʱ�򣬻�򵥵Ķ�·������Ч����һ���жϣ����Ƕ�·����
ÿһ��Position����һ��IsWalkable�������������������һ��Position���ɴ��˵������Ŀͻ�������
�µĵ�ͼ���ɵ�ͼ�ĵ�·�������Ѿ�������ʹ�ã���ʱ�����clear_memory���������м��������
	������뿪�����书������ʡ�ռ�����Խ�PathBuilder��bool use_memory��Ϊfalse�����㷨�в���
�����κ��й�routing_memory����Ч������
	ͬʱ��������ͨ������PathBuilder��static int memory_nums��static int max_mem_nums�����ü���
�����������make_path��ÿ�����Ӽ���·��֮�󶼻���������ֵ�Ĵ�С������ﵽ����������void clear_memory
��������ռ��䡣
	
	���ϣ�����ͨ��PathBuilder�����ÿ��Ƹ�Ѱ·�㷨�����ܺ���Ϊ���������ù��̲ο�PathFindingDll.cpp
�е����ע�͡�

	ʵ����˫��A*Ѱ·�Ƿ���������ȵ�������������ȡ���ھ���ĵ�ͼ��������������£�˫��A*��
���ܿ��ܲ��絥��A*��
	��󣬸�л���θ�У��Ϸ�������������췽�ٰ�˴ξ��������ڽ���������Ĺ�����ѧϰ���˺ܶ౦
���֪ʶ��
	�ٴθ�л��ף����Խ��Խ�ã�
*/

//·��
class Node
{
public:
	//��·�������
	Position position;
	//����ֵ
	float f;
	//��һ��·��
	Node* parent;
	//�Ƿ񱻼�������Ŀ�����
	bool start_set;
	//�Ƿ񱻼����յ��Ŀ�����
	bool end_set;
};

//��������·�����Ĺ�����
class PathBuilder
{
public:
	//·������
		//��������յ��ظ��������ֱ�ӻ�ȡ��һ�μ���õ�·��,�ռ任ʱ��
		//�ͳ�����һ�����������ڣ���NodeFactory����������ֻ��FindPath�������ڡ�
	static unordered_map<int, unordered_map<int, unordered_map<int, unordered_map<int, vector<Position>>>>> routing_memory;
	static int memory_nums;
	static int max_mem_nums;
	//�Ƿ�����·�����书��
		//ȡ�����ڴ��Ƿ��㹻
	bool use_memory;

	//����ʽϵ���趨
		//·���������ϵ��
	float h_d;
		//�յ������ϵ��
	float g_d;

	//����Ѱ·��Ҫ����Ϣ
	Grid const& grid;
	Position const start;
	Position const end;
	//��¼���������ĵ�
		//�ڸõ㼯���д��ڲ����ڿ�����nodes�в����ڵ�·������Ϊ�ռ�
	unordered_map<int,unordered_map<int,Node*>> flag;

	struct heuristic
	{
		bool operator()(Node* const& node1, const Node* const& node2)const
		{
			//��Node������ֵ��С��������
			return node1->f > node2->f;
		}
	};
	//���࿪����  ʹ�ö�����Ż�
	priority_queue<Node*,vector<Node*>,heuristic> open_set_start;//����ȡ�����ӶȾ�Ϊlogn������vector��n���Ӷ�
	//�յ�࿪����
	priority_queue<Node*, vector<Node*>, heuristic> open_set_end;

	//------------------------------------------------------------------------------------------------
	PathBuilder(Grid const& grid_, Position const& start_, Position const& end_, bool use_mem = true);

	//����·��
	Node* make_node(Position const p, Position const target, Node* parent);
	//�������·�㣬���Ҽ��뿪����
	Node* add_serround(Node* parent, Position const target);
	//�ӿ������л�ȡ��һ��������·�㲢�ҽ���ӿ�������ɾ������Ϊ����ռ���
	Node* get_next(priority_queue<Node*, vector<Node*>, heuristic>& open_set);
	//��ȡ����
	bool find_memory(IWaypointList& waypointList);
	//���·���ͼ���·��
	void make_path(IWaypointList& waypointList, Node* crossPoint, Node* keyPoint);
	//�������
	void clear_memory();
	//��ȡ�յ�·��
	Node* get_end_node();
	//�㷨�����
	void fill_path(IWaypointList& waypointList);
	//�ͷ����д�������·��
	~PathBuilder();
private:
};


PathBuilder::PathBuilder(Grid const& grid_, Position const& start_, Position const& end_, bool use_mem)
	:grid(grid_), start(start_), end(end_), use_memory(use_mem)
{
	if (use_memory == false)
		clear_memory();
	//������ϵ��Ĭ��Ϊ1
	h_d = 1.0;
	g_d = 1.0;
}

Node* PathBuilder::make_node(Position const p, Position const target, Node* parent)
{
	/*
	Debug��־��
		FindPath�ӿ�ò�ƻᴫ��һЩ����grid��Χ��position
		������ʹ��һ��if�жϽ������Ļ�����˵����Խ�����Bug
	*/
	//��黵��
	if (p.column < grid.numColumns && p.row < grid.numRows)
	{
		//ÿ��·��ֻ�ܱ�����һ��
		Node* this_node = flag[p.row][p.column];
		if (this_node != nullptr)
		{
			if (target.row == start.row && target.column == start.column)
			{
				if (this_node->end_set == false)
				{
					this_node->end_set = true;
					open_set_end.push(this_node);
				}
			}
			else
			{
				if (this_node->start_set == false)
				{
					this_node->start_set = true;
					open_set_start.push(this_node);
				}
			}
			return this_node;
		}

		this_node = new Node();
		this_node->position = p;
		this_node->parent = parent;


		/****����ʽ�����ش���****/
		//·������g�ļ���
		float g = 0;
		Node* iter = this_node;
		while (iter->parent != nullptr)
		{
			//б��·������
			if (abs(int(iter->position.column - iter->parent->position.column))
				+ abs(int(iter->position.row - iter->parent->position.row)) == 2)
				g += sqrtf(2);
			//ֱ�Ǳ�·������
			else
			{
				g += 1;
			}
			iter = iter->parent;
		}

		//������h�ļ���
		float h = 0;
		//����ʹ�ð˷�������g�����б�ѩ��ȷ�������Ҽٶ�б�߾���Ϊֱ�Ǳߵĸ���2��
		float dx = abs(float(float(this_node->position.column) - float(target.column)));
		float dy = abs(float(float(this_node->position.row) - float(target.row)));
		h = (dx + dy) + (sqrtf(2) - 2) * min(dx, dy);

		//����ֵ
		this_node->f = g * g_d + h * h_d;
		/****����ʽ�����ش���****/

		flag[p.row][p.column] = this_node;
		//�����Ӧ�Ŀ�����
		if (target.row == start.row && target.column == start.column)
		{
			this_node->end_set = true;
			this_node->start_set = false;
			open_set_end.push(this_node);
		}
		else
		{
			this_node->start_set = true;
			this_node->end_set = false;
			open_set_start.push(this_node);
		}
		return this_node;
	}
	else
		return nullptr;
}

Node* PathBuilder::add_serround(Node* parent, Position const target)
{
	bool up_flag = false, down_flag = false, right_flag = false, left_flag = false;
	//�������ҵ����
	//�����·��
	Position tmp = parent->position;
	tmp.row -= 1;
	if (tmp.row < grid.numRows && tmp.column < grid.numColumns && grid.IsWalkable(tmp))
	{
		up_flag = true;
		Node* this_node = make_node(tmp, target, parent);

		//��⿪���Ƿ��ཻ
		if (this_node != nullptr && this_node->end_set && this_node->start_set)
			return this_node;
	}
	//�����·��
	tmp = parent->position;
	tmp.row += 1;
	if (tmp.row < grid.numRows && grid.IsWalkable(tmp))
	{
		down_flag = true;
		Node* this_node = make_node(tmp, target, parent);

		//��⿪���Ƿ��ཻ
		if (this_node != nullptr && this_node->end_set && this_node->start_set)
			return this_node;
	}
	//�����·��
	tmp = parent->position;
	tmp.column -= 1;
	if (tmp.row < grid.numRows && tmp.column < grid.numColumns && grid.IsWalkable(tmp))
	{
		left_flag = true;
		Node* this_node = make_node(tmp, target, parent);

		//��⿪���Ƿ��ཻ
		if (this_node != nullptr && this_node->end_set && this_node->start_set)
			return this_node;
	}
	//�����·��
	tmp = parent->position;
	tmp.column += 1;
	if (tmp.column < grid.numColumns && grid.IsWalkable(tmp))
	{
		right_flag = true;
		Node* this_node = make_node(tmp, target, parent);

		//��⿪���Ƿ��ཻ
		if (this_node != nullptr && this_node->end_set && this_node->start_set)
			return this_node;
	}

	//б�ߵ�����
	//�������
	tmp = parent->position;
	tmp.row -= 1;
	tmp.column -= 1;
	if (up_flag && left_flag && grid.IsWalkable(tmp))
	{
		Node* this_node = make_node(tmp, target, parent);

		//��⿪���Ƿ��ཻ
		if (this_node != nullptr && this_node->end_set && this_node->start_set)
			return this_node;
	}
	//�������
	tmp = parent->position;
	tmp.row -= 1;
	tmp.column += 1;
	if (right_flag && up_flag && grid.IsWalkable(tmp))
	{
		Node* this_node = make_node(tmp, target, parent);

		//��⿪���Ƿ��ཻ
		if (this_node != nullptr && this_node->end_set && this_node->start_set)
			return this_node;
	}
	//�������
	tmp = parent->position;
	tmp.column += 1;
	tmp.row += 1;
	if (right_flag && down_flag && grid.IsWalkable(tmp))
	{
		Node* this_node = make_node(tmp, target, parent);

		//��⿪���Ƿ��ཻ
		if (this_node != nullptr && this_node->end_set && this_node->start_set)
			return this_node;
	}
	//�������
	tmp = parent->position;
	tmp.column -= 1;
	tmp.row += 1;
	if (left_flag && down_flag && grid.IsWalkable(tmp))
	{
		Node* this_node = make_node(tmp, target, parent);

		//��⿪���Ƿ��ཻ
		if (this_node != nullptr && this_node->end_set && this_node->start_set)
			return this_node;
	}
	//û�м�⵽�ཻ
	return nullptr;
}

Node* PathBuilder::get_next(priority_queue<Node*, vector<Node*>, heuristic>& open_set)
{
	//�ڿ�������Ѱ�Ҵ�ɾ����·��
	Node* next = nullptr;
	//�Ӷ������ȡ����С����ֵ��Node��Ϊnext����
	if (open_set.size() > 0)
	{
		next = open_set.top();
		open_set.pop();
	}

	//����ɾ���ĵ㷵��,�����µ�parent
	return next;
}

bool PathBuilder::find_memory(IWaypointList& waypointList)
{
	vector<Position> mem;
	//��ѯ���м���
	if (routing_memory[start.row][start.column][end.row][end.column].size() > 0)
	{
		mem = routing_memory[start.row][start.column][end.row][end.column];
		//����ͼ�Ƿ���¹�
		for (Position& p : mem)
		{
			if (grid.IsWalkable(p) == false)
			{
				//��⵽���£�����������¼���
				clear_memory();
				return false;
			}
		}
		//û�и��¹���ͼ,�������
		for (Position& p : mem)
			waypointList.Add(p);
		return true;
	}
	//û�м���
	return false;
}

void PathBuilder::make_path(IWaypointList& waypointList, Node* crossPoint, Node* keyPoint)
{
	vector<Node*> nodes_1;
	while (crossPoint != nullptr)
	{
		nodes_1.push_back(crossPoint);
		crossPoint = crossPoint->parent;
	}
	vector<Node*> nodes_2;
	while (keyPoint != nullptr)
	{
		nodes_2.push_back(keyPoint);
		keyPoint = keyPoint->parent;
	}
	//����
	vector<Node*> nodes;
	if (nodes_1.back()->position.row == end.row && nodes_1.back()->position.column == end.column)
	{
		for (int i = nodes_2.size() - 1; i >= 0; --i)nodes.push_back(nodes_2[i]);
		for (int i = 0; i < nodes_1.size(); ++i)nodes.push_back(nodes_1[i]);
	}
	else
	{
		for (int i = nodes_1.size() - 1; i >= 0; --i)nodes.push_back(nodes_1[i]);
		for (int i = 0; i < nodes_2.size(); ++i)nodes.push_back(nodes_2[i]);
	}

	vector<Position> new_routing;
	for (int i = 0; i < nodes.size(); ++i)
	{
		waypointList.Add(nodes[i]->position);
		if (use_memory)
			new_routing.push_back(nodes[i]->position);
	}
	if (use_memory)
	{
		routing_memory[start.row][start.column][end.row][end.column] = new_routing;
		++memory_nums;
		//���������� �������ƾ����
		if (memory_nums > max_mem_nums)
			clear_memory();
	}

}

void PathBuilder::clear_memory()
{
	memory_nums = 0;
	routing_memory.clear();
}

Node* PathBuilder::get_end_node()
{
	return flag[end.row][end.column];
}

void PathBuilder::fill_path(IWaypointList& waypointList)
{
	//�Ȳ��Ҽ���
	if (use_memory && find_memory(waypointList))
		return;

	//��ʼ·��
	Node* start_node = make_node(start, end, nullptr);
	//�յ�·��
	Node* end_node = make_node(end, start, nullptr);

	start_node = get_next(open_set_start);
	end_node = get_next(open_set_end);

	//�޳���ֵ��Ӱ��
	if (start_node == nullptr || end_node == nullptr)return;

	//�����ཻ��·��
	Node* crossPoint = nullptr;
	//���ཻ·�����ӵ�����һ���ؼ�·��
	Node* keyPoint = nullptr;

	while (start_node && end_node)
	{
		//���������·����Χ·��
		crossPoint = add_serround(start_node, end);
		//����ཻ
		if (crossPoint != nullptr)
		{
			keyPoint = start_node;
			break;
		}
		//������յ��·����Χ·��
		crossPoint = add_serround(end_node, start);
		//����ཻ
		if (crossPoint != nullptr)
		{
			keyPoint = end_node;
			break;
		}

		start_node = get_next(open_set_start);
		end_node = get_next(open_set_end);
	}

	//���û��·��
	if (start_node == nullptr || end_node == nullptr)
		return;

	make_path(waypointList, crossPoint, keyPoint);
}

PathBuilder::~PathBuilder()
{
	for (auto it_1 = flag.begin(); it_1 != flag.end(); ++it_1)
	{
		for (auto it_2 = (*it_1).second.begin(); it_2 != (*it_1).second.end(); ++it_2)
			delete (*it_2).second;
	}
}

#endif
