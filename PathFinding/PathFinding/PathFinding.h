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
/*                                  ***开发总结***
参赛者：邓琳琛
学校：北京航空航天大学
电话：15910922135
邮箱：515143675@qq.com
完成日期：2020年8月4日
visual studio版本：Visual Studio 2019（v142） Debug x86
	
	该寻路算法采用双向A*寻路算法，比传统的A*算法速度更快，消耗栈空间更小，性能提升明显。
并且还加入了路径记忆功能，进一步提高算法速度。
	我主要设计了两个类(Node和PathBuilder)来辅助实现寻路算法，并且根据我自己的理解进行了一
些优化和扩展设计，使得该算法使用起来更方便灵活，可以根据实际业务情况进行调整。
	下面对主要部分进行一些介绍。
	
	Node类：
	首先需要设计一个能保存网格位置Position和计算A*启发式相关信息的路点类——Node，其中需
要保存的信息包括该路点的位置(Position position)、记录是否在起点侧或者终点侧开集的布尔值、
指向前一个路点的指针(Node* parent)、还有最终的启发值(float f)。
	
	PathBuilder类：
	该类是算法的核心，算法的实现以及其他所有的辅助功能都由该创建者类的对象去实现的。那么
FindPath函数体中就只有几行简单的代码了。该创建者主要生产两种“产品”，路点Node对象和路径记
忆vector<Position>，所有的路径记忆则由一个“公共仓库”(static unoredered_map)对象去保存。

	路点由make_node成员函数构建，make_node函数可以判断应该将该路点加入哪一侧的开集，并且还
保证了每一个Position在堆内存上只有一个路点跟其对应。各个路点的集合状态和启发值也都是在该函
数中计算得到的。

	add_serround成员函数通过调用make_node函数将当前路点的周围路点加入对应的开集合。并且检测
在将新路点加入开集合时是否在两侧开集合产生了冲突，如果发生了冲突则说明起点侧路径和终点侧路
径已经汇合，可以进行路径连接，不必再进行后续计算了。

	get_next函数用于选择下一个前往的路点，并且将被选择的路点从对应的开集合中去除。由于开集合
都采用multiset<Node*, heuristic>这样的数据结构，因此插入和删除路点的复杂度都是logN，这也是一
个小小的优化。

	make_path函数可以将起点侧和终点侧生成的两个“半个”路径缝合起来形成完成的路径，如果开启
记忆功能则会将其存入记忆仓库，供下次查询使用。

	fill_path函数则为算法的主要框架和实现。

	
	下面专门介绍一下路径记忆功能的实现：
	一条路径记忆为一个vector<Position>对象，存放在一个四维的unordered_map容器routing_memory中，
根据start和end的row和column值来作为四个key获取。routing_memory是NodeFactory的一个static对象，
其生命周期独立于NodeFactory对象，伴随程序的整个生命周期，因此在每次FindPath被调用的时候以前的
记忆都存在，在find_memory查询到记忆路径的时候，会简单的对路径的有效性做一个判断，就是对路径上
每一个Position调用一次IsWalkable函数，如果发现其中有一个Position不可达，则说明程序的客户加载了
新的地图，旧地图的的路径记忆已经不能再使用，此时会调用clear_memory方法将所有记忆清除。
	如果不想开启记忆功能来节省空间则可以将PathBuilder的bool use_memory设为false，则算法中不会
产生任何有关routing_memory的有效操作。
	同时，还可以通过设置PathBuilder中static int memory_nums和static int max_mem_nums来设置记忆
的最大容量，make_path在每次增加记忆路径之后都会检查这两个值的大小，如果达到上限则会调用void clear_memory
函数来清空记忆。
	
	综上，可以通过PathBuilder的配置控制该寻路算法的性能和行为，具体配置过程参考PathFindingDll.cpp
中的相关注释。

	实际上双向A*寻路是否会带来大幅度的性能提升还是取决于具体的地图，在最糟糕的情况下，双向A*的
性能可能不如单向A*。
	最后，感谢畅游高校游戏技术竞赛的主办方举办此次竞赛，我在解决这个赛题的过程中学习到了很多宝
贵的知识。
	再次感谢，祝畅游越办越好！
*/

//路点
class Node
{
public:
	//该路点的坐标
	Position position;
	//启发值
	float f;
	//上一个路点
	Node* parent;
	//是否被加入起点侧的开集合
	bool start_set;
	//是否被加入终点侧的开集合
	bool end_set;
};

//用来创建路点对象的工厂类
class PathBuilder
{
public:
	//路径记忆
		//如果起点和终点重复输入可以直接获取上一次计算好的路径,空间换时间
		//和程序有一样的生命周期，而NodeFactory的生命周期只在FindPath函数体内。
	static unordered_map<int, unordered_map<int, unordered_map<int, unordered_map<int, vector<Position>>>>> routing_memory;
	static int memory_nums;
	static int max_mem_nums;
	//是否启用路径记忆功能
		//取决于内存是否足够
	bool use_memory;

	//启发式系数设定
		//路径代价项的系数
	float h_d;
		//终点距离项系数
	float g_d;

	//保存寻路需要的信息
	Grid const& grid;
	Position const start;
	Position const end;
	//记录被创建过的点
		//在该点集合中存在并且在开集合nodes中不存在的路点则视为闭集
	unordered_map<int,unordered_map<int,Node*>> flag;

	struct heuristic
	{
		bool operator()(Node* const& node1, const Node* const& node2)const
		{
			//将Node按启发值从小到大排序
			return node1->f > node2->f;
		}
	};
	//起点侧开集合  使用二叉堆优化
	priority_queue<Node*,vector<Node*>,heuristic> open_set_start;//插入取出复杂度均为logn，优于vector的n复杂度
	//终点侧开集合
	priority_queue<Node*, vector<Node*>, heuristic> open_set_end;

	//------------------------------------------------------------------------------------------------
	PathBuilder(Grid const& grid_, Position const& start_, Position const& end_, bool use_mem = true);

	//创建路点
	Node* make_node(Position const p, Position const target, Node* parent);
	//添加相邻路点，并且加入开集合
	Node* add_serround(Node* parent, Position const target);
	//从开集合中获取下一个遍历的路点并且将其从开集合中删除（视为加入闭集）
	Node* get_next(priority_queue<Node*, vector<Node*>, heuristic>& open_set);
	//获取记忆
	bool find_memory(IWaypointList& waypointList);
	//填充路径和记忆路径
	void make_path(IWaypointList& waypointList, Node* crossPoint, Node* keyPoint);
	//清除记忆
	void clear_memory();
	//获取终点路点
	Node* get_end_node();
	//算法主框架
	void fill_path(IWaypointList& waypointList);
	//释放所有创建过的路点
	~PathBuilder();
private:
};


PathBuilder::PathBuilder(Grid const& grid_, Position const& start_, Position const& end_, bool use_mem)
	:grid(grid_), start(start_), end(end_), use_memory(use_mem)
{
	if (use_memory == false)
		clear_memory();
	//启发项系数默认为1
	h_d = 1.0;
	g_d = 1.0;
}

Node* PathBuilder::make_node(Position const p, Position const target, Node* parent)
{
	/*
	Debug日志：
		FindPath接口貌似会传入一些超出grid范围的position
		在这里使用一个if判断将这样的坏点过滤掉可以解决这个Bug
	*/
	//检查坏点
	if (p.column < grid.numColumns && p.row < grid.numRows)
	{
		//每个路点只能被创建一次
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


		/****启发式设计相关代码****/
		//路径代价g的计算
		float g = 0;
		Node* iter = this_node;
		while (iter->parent != nullptr)
		{
			//斜边路径代价
			if (abs(int(iter->position.column - iter->parent->position.column))
				+ abs(int(iter->position.row - iter->parent->position.row)) == 2)
				g += sqrtf(2);
			//直角边路径代价
			else
			{
				g += 1;
			}
			iter = iter->parent;
		}

		//距离项h的计算
		float h = 0;
		//由于使用八方向网格，g采用切比雪夫确定，并且假定斜边距离为直角边的根号2倍
		float dx = abs(float(float(this_node->position.column) - float(target.column)));
		float dy = abs(float(float(this_node->position.row) - float(target.row)));
		h = (dx + dy) + (sqrtf(2) - 2) * min(dx, dy);

		//启发值
		this_node->f = g * g_d + h * h_d;
		/****启发式设计相关代码****/

		flag[p.row][p.column] = this_node;
		//加入对应的开集合
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
	//上下左右的添加
	//添加上路点
	Position tmp = parent->position;
	tmp.row -= 1;
	if (tmp.row < grid.numRows && tmp.column < grid.numColumns && grid.IsWalkable(tmp))
	{
		up_flag = true;
		Node* this_node = make_node(tmp, target, parent);

		//检测开集是否相交
		if (this_node != nullptr && this_node->end_set && this_node->start_set)
			return this_node;
	}
	//添加下路点
	tmp = parent->position;
	tmp.row += 1;
	if (tmp.row < grid.numRows && grid.IsWalkable(tmp))
	{
		down_flag = true;
		Node* this_node = make_node(tmp, target, parent);

		//检测开集是否相交
		if (this_node != nullptr && this_node->end_set && this_node->start_set)
			return this_node;
	}
	//添加左路点
	tmp = parent->position;
	tmp.column -= 1;
	if (tmp.row < grid.numRows && tmp.column < grid.numColumns && grid.IsWalkable(tmp))
	{
		left_flag = true;
		Node* this_node = make_node(tmp, target, parent);

		//检测开集是否相交
		if (this_node != nullptr && this_node->end_set && this_node->start_set)
			return this_node;
	}
	//添加右路点
	tmp = parent->position;
	tmp.column += 1;
	if (tmp.column < grid.numColumns && grid.IsWalkable(tmp))
	{
		right_flag = true;
		Node* this_node = make_node(tmp, target, parent);

		//检测开集是否相交
		if (this_node != nullptr && this_node->end_set && this_node->start_set)
			return this_node;
	}

	//斜边点的添加
	//添加左上
	tmp = parent->position;
	tmp.row -= 1;
	tmp.column -= 1;
	if (up_flag && left_flag && grid.IsWalkable(tmp))
	{
		Node* this_node = make_node(tmp, target, parent);

		//检测开集是否相交
		if (this_node != nullptr && this_node->end_set && this_node->start_set)
			return this_node;
	}
	//添加右上
	tmp = parent->position;
	tmp.row -= 1;
	tmp.column += 1;
	if (right_flag && up_flag && grid.IsWalkable(tmp))
	{
		Node* this_node = make_node(tmp, target, parent);

		//检测开集是否相交
		if (this_node != nullptr && this_node->end_set && this_node->start_set)
			return this_node;
	}
	//添加右下
	tmp = parent->position;
	tmp.column += 1;
	tmp.row += 1;
	if (right_flag && down_flag && grid.IsWalkable(tmp))
	{
		Node* this_node = make_node(tmp, target, parent);

		//检测开集是否相交
		if (this_node != nullptr && this_node->end_set && this_node->start_set)
			return this_node;
	}
	//添加左下
	tmp = parent->position;
	tmp.column -= 1;
	tmp.row += 1;
	if (left_flag && down_flag && grid.IsWalkable(tmp))
	{
		Node* this_node = make_node(tmp, target, parent);

		//检测开集是否相交
		if (this_node != nullptr && this_node->end_set && this_node->start_set)
			return this_node;
	}
	//没有检测到相交
	return nullptr;
}

Node* PathBuilder::get_next(priority_queue<Node*, vector<Node*>, heuristic>& open_set)
{
	//在开集合中寻找待删除的路点
	Node* next = nullptr;
	//从二叉堆中取出最小启发值的Node作为next返回
	if (open_set.size() > 0)
	{
		next = open_set.top();
		open_set.pop();
	}

	//将被删除的点返回,当成新的parent
	return next;
}

bool PathBuilder::find_memory(IWaypointList& waypointList)
{
	vector<Position> mem;
	//查询到有记忆
	if (routing_memory[start.row][start.column][end.row][end.column].size() > 0)
	{
		mem = routing_memory[start.row][start.column][end.row][end.column];
		//检测地图是否更新过
		for (Position& p : mem)
		{
			if (grid.IsWalkable(p) == false)
			{
				//检测到更新，清除记忆重新计算
				clear_memory();
				return false;
			}
		}
		//没有更新过地图,正常填充
		for (Position& p : mem)
			waypointList.Add(p);
		return true;
	}
	//没有记忆
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
	//连接
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
		//检查记忆数量 超过限制就清除
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
	//先查找记忆
	if (use_memory && find_memory(waypointList))
		return;

	//起始路点
	Node* start_node = make_node(start, end, nullptr);
	//终点路点
	Node* end_node = make_node(end, start, nullptr);

	start_node = get_next(open_set_start);
	end_node = get_next(open_set_end);

	//剔除坏值的影响
	if (start_node == nullptr || end_node == nullptr)return;

	//开集相交的路点
	Node* crossPoint = nullptr;
	//与相交路点连接的另外一个关键路点
	Node* keyPoint = nullptr;

	while (start_node && end_node)
	{
		//先添加起点侧路点周围路点
		crossPoint = add_serround(start_node, end);
		//检测相交
		if (crossPoint != nullptr)
		{
			keyPoint = start_node;
			break;
		}
		//再添加终点侧路点周围路点
		crossPoint = add_serround(end_node, start);
		//检测相交
		if (crossPoint != nullptr)
		{
			keyPoint = end_node;
			break;
		}

		start_node = get_next(open_set_start);
		end_node = get_next(open_set_end);
	}

	//如果没有路径
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
