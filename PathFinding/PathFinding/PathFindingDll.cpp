#include "PathFinding.h"


/*
参赛者：邓琳琛
学校：北京航空航天大学
电话：15910922135
邮箱：515143675@qq.com
完成日期：2020年8月4日
visual studio版本：Visual Studio 2019（v142） Debug x86
*/

// start 到 end 不可达时，不应该调用IWaypointList::Add；
// 反之，需要从start开始，依次添加所有相邻的waypoint，直到end，（需要包含start和end）
// “相邻” 指两个格子存在公共边或公共顶点


//初始化NodeFactory中用于记忆保存的静态变量
	//存储记忆的容器
unordered_map<int, unordered_map<int, unordered_map<int, unordered_map<int, vector<Position>>>>> PathBuilder::routing_memory;
	//记忆的数目
int PathBuilder::memory_nums = 0;
	//指定最大记忆数量
int PathBuilder::max_mem_nums = 999;
	
extern "C" __declspec(dllexport) void __cdecl FindPath(Grid const & grid,
  Position const & start, Position const & end,
  IWaypointList & waypointList) 
{
	//起点终点重合时,不操作
	if (start.column == end.column && start.row == end.row)
		return;

	//起点不可达时，不操作
	if (grid.IsWalkable(start) == false)
		return;

	/*本寻路算法基于双向A*寻路，详细说明查阅PathFinding.h文件中的相关注释*/

	//定义操作路点对象和路点集合的路径创建者对象
	PathBuilder builder(grid,start,end);
		//开启路径记忆功能，消耗少量内存换取速度的提升
	builder.use_memory = true;
	/*
	关于启发式各项系数选值的讨论：
		f=g*g_d+h*h_d
		其中g为路径代价；h为到终点的代价估计，取切比雪夫距离
		具体计算细节参考make_node函数
		启发式可以用来控制A*算法的行为：
		1.在一种极端情况下，如果h_d(终点距离项系数)为0，那么只有g(路径代价)，A*就变成了Dijkstra算法，此时算法总会找到一条最短路径，但是此时算法速度最慢。
		2.如果h_d取小于1的值，此时g*g_d在启发值中比重增大，那么A*会找到一条较短路径。较低的h_d*h会导致每一个A*节点有较多的待选扩展点，使算法变得较慢。
		3.在另一个极端情况下，如果h*h_d相对于g*g_d非常的高，相当于只有h*h_d，A*就变成了最佳优先搜索,此时算法可以快速选出下一个扩展点，算法速度最快，但是路径较长。
		为了追求最短路径，我将h_d设为较小的数值，此时的结果接近赛题样例中的路径，甚至比其更短。
	*/
		//配置启发式各项系数
		//当前配置取得了不错的结果
	builder.g_d = 1.0;	//路径代价项系数
	builder.h_d = 0.1;	//距离终点代价预估项系数

	//填充waypointList
	builder.fill_path(waypointList);
}
