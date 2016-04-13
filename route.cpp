// route.cpp
#include "route.h"
#include "lib_record.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/timeb.h>
#include <vector>

#define _OUTPUT
#ifdef _OUTPUT
#include <iostream>
#endif
using namespace std;

vector<vector<int> > topo_int;
vector<int> node_name;
vector<vector<int> > D;
vector<vector<int> > P;
vector<vector<int> > D_cir;
vector<vector<int> > P_cir;
vector<vector<int> > D_init;
vector<vector<int> > P_init;
vector<int> demand_sparse;

vector<vector<int> > best_way;
vector<vector<int> > all_way;
vector<vector<int> > Router;
vector<int> Cost;
vector<int> mask;

vector<vector<int> > g_Router_v;
vector<vector<int> > g_Router_vp;
vector<int> g_Cost;
vector<int> g_mask;

vector<int> demand_table;
vector<int> demand_table_orign;
vector<vector<int> > ResultAllRout;
vector<vector<int> > adjInfor;
vector<int > curRout;

unsigned int V_point[2];//重复点V点头尾
int node_num;
int node_start;
int node_end;
int len;

int init_best_way(int num_way)
{
    best_way.clear();

    //产生随机的num_way条路径(每条路径包括头尾) -> best_way[num_way]
    for (int way_num=0; way_num<num_way; way_num++)
    {
        //获取demand_sparse的某种随机排列temp_way
        vector<int> temp_way = demand_sparse;
        int temp_point;
        int k = len;
        for(int i=0; i<len-1; i++)
        {
            int tmp = rand() % k;
            //exchange tmp<-->k-1
            temp_point = temp_way[tmp];
            temp_way[tmp] = temp_way[k-1];
            temp_way[k-1] = temp_point;
            k--;
        }

        //加上开头和结尾
        temp_way.insert(temp_way.begin(),node_start);
        temp_way.push_back(node_end);

        best_way.push_back(temp_way); //best_way包含头尾
    }

#if 0
    for (int way_num=0; way_num<num_way; way_num++) //检查是否开始随机产生路径有问题
    {
        cout << way_num << "---";
        for (int i=0; i<len+2; i++)
        {
            printf("%3d -",best_way[way_num][i]);
        }
        cout << endl;
    }
#endif // 1

    return 0;
}

int init_all_way(int num_way)
{
    all_way.clear();
    all_way.resize(2*num_way);

    //对这num_way条路径进行相应的变化并产生后代
    //best_way[num_way] --> all_way[2*num_way]
    for (int i=0; i<num_way; i++)
    {
        all_way[i] = best_way[i];

        all_way[i+num_way] = best_way[i];

        int m = (rand() % len) + 1;
        int n = (rand() % len) + 1;
        int temp_point = all_way[i+num_way][n];
        all_way[i+num_way][n] = all_way[i+num_way][m];
        all_way[i+num_way][m] = temp_point;
    }

    return 0;
}

int search2node(vector<int>& r, int& cost, int nodeStart, int nodeEnd)
{
    if (65535 == D[nodeStart][nodeEnd])
    {
        cost = 65535;
        return -1;
    }
    else
    {
        cost += D[nodeStart][nodeEnd];
    }

    int node = P[nodeStart][nodeEnd];

    while (node != nodeEnd)
    {
        r.push_back(node);
        node = P[node][nodeEnd];
    }

    r.push_back(nodeEnd);

    return 0;
}

int judge_router(int index)
{
    if (Cost[index] >= 65535) //不通，非解，视为有环通路
    {
        return 0;
    }

    static int _STATISTIC[600];
    for (int i=0; i<600; i++)
    {
        _STATISTIC[i] = 0;
    }

    for (unsigned int i=0; i<Router[index].size(); i++)
    {
        int tmp = Router[index][i];
        if (_STATISTIC[tmp] == 0)
        {
            _STATISTIC[tmp]++;
        }
        else if (_STATISTIC[tmp] == 1)
        {
            return 0;
        }
    }

    return 1; //无环路输出1
}

int judge_g_mask(void)
{
    for (unsigned int i=0; i<g_mask.size(); i++)
    {
        if (1 == g_mask[i])
        {
            return 0;
        }
    }

    return 1;
}

int find_line(int nodeOut, int nodeIn)
{
    int line = -1;
    for (unsigned int i=0; i<topo_int.size(); i++)
    {
        if (topo_int[i][1] == node_name[nodeOut] && topo_int[i][2] == node_name[nodeIn])
        {
            if (line == -1)
            {
                line = i;
            }
            else if (topo_int[i][3] < topo_int[line][3])
            {
                line = i;
            }
        }
    }

    return topo_int[line][0];
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int judge_circle(vector<int> r)
{
    static int _STATISTIC[600];
    for (int i=0; i<600; i++)
    {
        _STATISTIC[i] = 0;
    }

    for (unsigned int i=0; i<r.size(); i++)
    {
        int tmp = r[i];
        if (_STATISTIC[tmp] == 0)
        {
            _STATISTIC[tmp]++;
        }
        else if (_STATISTIC[tmp] == 1)
        {
            return 0;
        }
    }

    return 1; //无环路输出1
}

int judge_exist(vector<int> r, int cmp)
{
    for (unsigned int i=0; i<r.size(); ++i)
    {
        if (cmp == r[i])
        {
            return -1;
        }
    }
    return 1;
}

int isContainDemand(vector<int> demand_table)
{
    for (unsigned int i=0; i<demand_table.size(); ++i)
    {
        if (demand_table[i] != 0)
        {
            return -1;
        }
    }
    return 1;
}

int violenceSearch(int curPt)
{
	curRout.push_back(curPt);
	demand_table[curPt] = 0;

	//如果curPt为终点，则退出,并判断此路径是否包含全部demand，若是将这次路径存储
	//同时，还需要返回上一级
	if (curPt == node_end)
	{
		if (judge_circle( curRout ) == 1 && isContainDemand(demand_table) == 1)
        {
            ResultAllRout.push_back(curRout);
        }

        //wheather its true to earse the tail???
		curRout.erase(curRout.end()-1);
		if (demand_table_orign[curPt] == 1)
        {
            demand_table[curPt] = 1;
        }
		return 0;
	}
	else
	{
		for (unsigned int i=0; i<adjInfor[curPt].size(); ++i)
		{
			int nextPt = adjInfor[curPt][i];

			//if dont exsit in the current rout,ten violenceReach
			if (judge_exist(curRout,nextPt) != -1)
			{
                //否则寻找ptcur的邻接点,将邻接点置为curPt
                violenceSearch(nextPt);
			}
			if (i == adjInfor[curPt].size()-1)
			{
                curRout.erase(curRout.end()-1);
                if (demand_table_orign[curPt] == 1)
                {
                    demand_table[curPt] = 1;
                }
			}
		}
	}

	return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int search2node_cir(vector<int>& r, int nodeStart, int nodeEnd)//找到两个V点之间所经过的点
{
    if (65535 == D_cir[nodeStart][nodeEnd])
    {
        return -1;
    }
    int node = P_cir[nodeStart][nodeEnd];
    while (node != nodeEnd)
    {
        r.push_back(node);
        node = P_cir[node][nodeEnd];
    }
    r.push_back(nodeEnd);
    return 0;
}

void calc_cost(int& cost,int nodeStart, int nodeEnd)//计算两点之间的成本
{
    if (65535 == D_cir[nodeStart][nodeEnd])
    {
        cost = 65535;
    }

    cost += D_cir[nodeStart][nodeEnd];
}

int g_vpoint(int best_id)//找g_Router_vp[best_id]中的重复点，返回该重复点所在demand段的头尾位置V_point[0],[1]
{
    V_point[0] = 0;
    V_point[1] = 0;
    static int mask_point[600];
    for (int i=0; i<600; i++)
    {
        mask_point[i] = 0;
    }
    int index_v = 0;
    vector<int> v_num; //用于记录v点在 g_Router_vp中的坐标

    for (unsigned int i=0; i<g_Router_vp[best_id].size(); i++)
    {
        int tmp = g_Router_vp[best_id][i];
        if (tmp == g_Router_v[best_id][index_v]) //如果点到了V点
        {
            index_v ++;
            v_num.push_back(i);
        }

        if (mask_point[tmp] == 0)
        {
            mask_point[tmp]++;
        }
        else if (mask_point[tmp] == 1)
        {
            V_point[0] = v_num[index_v-1];//第二个重复点的V区间开始点。
            for (unsigned int j = i; j<g_Router_vp[best_id].size(); j++)
            {
                if( g_Router_vp[best_id][j] == g_Router_v[best_id][index_v] ) //寻找重复点V区间的终止点
                {
                    V_point[1] = j;
                    return 1;
                }
            }
        }
    }

    return 0;
}

int remove_repeat(int best_id)//对有重复点的区间进行消除重复点，生成新的g_Router_vp[best_id]，并且得到它的成本
{
    D_cir = D_init;
    P_cir = P_init;

    if (V_point[0] > 0)
    {
        for (unsigned int i=0; i<V_point[0]; i++) //屏蔽目标区间前面的点
        {
            int temp_point = g_Router_vp[best_id][i];
            for (int j=0; j<node_num; j++)
            {
                D_cir[j][temp_point] = 65535;
            }
        }
    }
    if (V_point[1] < g_Router_vp[best_id].size()-1)
    {
        for (unsigned int i=V_point[1]+1; i<g_Router_vp[best_id].size(); i++) //屏蔽目标区间后面的点
        {
            int temp_point = g_Router_vp[best_id][i];
            for (int j=0; j<node_num; j++)
            {
                D_cir[j][temp_point] = 65535;
            }
        }
    }

    for (int k=0; k<node_num; k++)  //对目标区间进行算法求路计算。
    {
        for (int i=0; i<node_num; i++)
        {
            for (int j=0; j<node_num; j++)
            {
                int tmp = D_cir[i][k] + D_cir[k][j];
                if (D_cir[i][j] > tmp)
                {
                    D_cir[i][j] = tmp;
                    P_cir[i][j] = P_cir[i][k];
                }
            }
        }
    }

    vector<int> cir_temp;
    cir_temp.push_back(g_Router_vp[best_id][V_point[0]]);
    if (search2node_cir(cir_temp, g_Router_vp[best_id][V_point[0]], g_Router_vp[best_id][V_point[1]]) == -1) //去重复点失败
    {
        return 1;
    }

    g_Router_vp[best_id].erase(g_Router_vp[best_id].begin() + V_point[0], g_Router_vp[best_id].begin() + V_point[1]+1);//将原来中间那段去除，并且替换成新的。
    for(unsigned int i=0; i<cir_temp.size(); i++)
    {
        g_Router_vp[best_id].insert(g_Router_vp[best_id].begin()+V_point[0]+i, cir_temp[i]);
    }

    int cost = 0;
    for (unsigned int i=0; i<g_Router_vp[best_id].size()-1; i++)//计算新的路径的成本
    {
        calc_cost(cost, g_Router_vp[best_id][i], g_Router_vp[best_id][i+1]);
    }
    Cost[best_id] = cost;

    return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//你要完成的功能总入口
void search_route(char *topo[5000], int edge_num, char *demand)
{
    srand(time(0)); //随机种子

    struct timeb rawtime;
    ftime(&rawtime);
    int init_ms = rawtime.millitm;
    unsigned long init_s = rawtime.time;

    //转化成topo_int
    for (int i=0;i<edge_num;i++) //edge行
    {
        vector<int> tmp(4);
        int index = 0;
        int num = 0;
        for (int j=0;index<4;j++)
        {
            if ('0' <= topo[i][j] && topo[i][j] <= '9')
            {
                num *= 10;
                num += topo[i][j] - '0';
            }
            else
            {
                tmp[index] = num;
                num = 0;
                index++;
            }
        }
        topo_int.push_back(tmp);
    }

    //找到节点数量
    vector<int> _node(600);
    for (int i=0; i<600; i++) //Initialize
    {
        _node[i] = 0;
    }
    for (int i=0; i<edge_num; i++) //statistic
    {
        _node[topo_int[i][1]]++;
        _node[topo_int[i][2]]++;
    }
    node_num = 0;
    for (int i=0; i<600; i++) //
    {
        if (_node[i])
        {
            _node[i] = node_num;
            node_num++;
            node_name.push_back(i);
        }
    }

    //解析demand到demand_sparse
    int num = 0;
    for (int i=0; ;i++)
    {
        if ('0' <= demand[i] && demand[i] <= '9')
        {
            num *= 10;
            num += demand[i] - '0';
        }
        else
        {
            if (demand[i] == ',' || demand[i] == '|' || demand[i] == 0x0a)
            {
                demand_sparse.push_back(_node[num]);
                num = 0;
                if (demand[i] == 0x0a)
                {
                    break;
                }
            }
        }
    }
    //分配点
    node_start = demand_sparse[0]; //起点
    node_end = demand_sparse[1];   //终点
    demand_sparse.erase(demand_sparse.begin(),demand_sparse.begin()+2);
    len = int(demand_sparse.size()); //中间PASS节点数

    //初始化D和P内存
    D.resize(node_num);
    P.resize(node_num);
    for (int i=0;i<node_num;i++)
    {
        for (int j=0;j<node_num;j++)
        {
            if (i == j)
            {
                D[i].push_back(0);
            }
            else
            {
                D[i].push_back(65535);
            }
            P[i].push_back(j);
        }
    }

    //根据topo初始化D
    for (int i=0;i<edge_num;i++)
    {
        int tmpD = topo_int[i][3];
        if ( D[ _node[topo_int[i][1]] ][ _node[topo_int[i][2]] ] > tmpD )
        {
            D[ _node[topo_int[i][1]] ][ _node[topo_int[i][2]] ] = tmpD;
        }
    }
    D_init = D;
    P_init = P;

    /////////3号算法
    if (node_num <= 20)
    {
        for (int i=0; i<node_num; ++i)
        {
            demand_table_orign.push_back(0);
            demand_table.push_back(0);
        }
        for (int i=0; i<len; ++i)
        {
            demand_table_orign[ demand_sparse[i] ] = 1;
            demand_table[ demand_sparse[i] ] = 1;
        }
        demand_table[ node_start ] = 1;
        demand_table[ node_end ] = 1;
        demand_table_orign[ node_start ] = 1;
        demand_table_orign[ node_end ] = 1;

        //循环结束的条件：起始点node_start的邻接点全部遍历过
        //每次循环需要重新赋初值
        for (int i=0; i<node_num; ++i)
        {
            vector<int> tmp;
            tmp.clear();
            for (int j=0; j<node_num; ++j)
            {
                //i到j有边
                if (D[i][j] != 65535 && D[i][j] != 0)
                {
                    tmp.push_back(j);
                }
            }
            adjInfor.push_back(tmp);
        }

        violenceSearch(node_start);

        //find the minimue cost
        int minCost = 65535;
        int ind_rout = -1;
        for(unsigned int i=0; i<ResultAllRout.size(); i++)
        {
             int tmpCost = 0;
             for (unsigned int j=0; j<ResultAllRout[i].size()-1; j++)
             {
                tmpCost += D[ ResultAllRout[i][j] ][ ResultAllRout[i][j+1] ];
             }
             if (tmpCost < minCost)
             {
                minCost = tmpCost;
                ind_rout = i;
             }
        }
        #ifdef _OUTPUT
        cout << "Min:" << minCost << endl;
        for (unsigned int j=0; j<ResultAllRout[ind_rout].size(); j++)
        {
            cout << ResultAllRout[ind_rout][j];
            if (j != ResultAllRout[ind_rout].size()-1)
            {
                cout << "->";
            }
        }
        cout << endl;
        #endif

        //输出文件
        for (unsigned int i=0; i<ResultAllRout[ind_rout].size()-1; i++)
        {
            int line = find_line(ResultAllRout[ind_rout][i], ResultAllRout[ind_rout][i+1]);
            record_result(line);
        }

        return ;
    }

/////////////////////////////////////////////////////////////////////
    //弗洛伊德算法init_all_way
    for (int k=0;k<node_num;k++)
    {
        for (int i=0;i<node_num;i++)
        {
            for (int j=0;j<node_num;j++)
            {
                int tmp = D[i][k] + D[k][j];
                if (D[i][j] > tmp)
                {
                    D[i][j] = tmp;
                    P[i][j] = P[i][k];
                }
            }
        }
    }

/////////////////////////////////////////////////////////////////////
    //初步判断
    //如果起点终点不通，退出
    if (65535 == D[node_start][node_end])
    {
        #ifdef _OUTPUT
        cout << "NANA" << endl;
        #endif // _OUTPUT
        return ;
    }
    //如果中间节点不通，退出
    for (int i=0; i<len; i++)
    {
        int tmp = demand_sparse[i];
        if (65535 == D[node_start][tmp])
        {
            #ifdef _OUTPUT
            cout << "NANA" << endl;
            #endif // _OUTPUT
            return ;
        }
        if (65535 == D[tmp][node_end])
        {
            #ifdef _OUTPUT
            cout << "NANA" << endl;
            #endif // _OUTPUT
            return ;
        }
    }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (node_num > 20)
    {
        // num_test
        int num_test = 1000;  //实验次数//30 10 100 50 20
        if (node_num <= 300)
        {
            num_test = 300;
            if (node_num <= 200)
            {
                num_test = 150;
                if (node_num == 150)
                {
                    num_test = 100;
                }
            }
        }

        int num_way = 50; //路径多少
        int _STOP = 20; //终止条件 <=20

        for (int test_num=0; test_num<num_test; test_num++)//进行test_num次实验
        {
            //cout << test_num << endl;
            //根据demand产生随机的num_way条路径->best_way[num_way]
            init_best_way(num_way);

            vector<int> best_cost;
            best_cost.clear();
            int best_index = 0;
            for (int num_iter=0; num_iter<1000; num_iter++)//设置迭代次数
            {
                //printf("%2d | ", num_iter);

                init_all_way(num_way);

                //计算all_way[2*num_way]中每条路径的Router[2*num_way]和Cost[2*num_way]
                Router.clear();
                Cost.resize(2*num_way);
                mask.resize(2*num_way);//无环路标示
                vector<int> r;
                int cost = 0;
                for (int i=0; i<(2*num_way); i++)
                {
                    r.clear();
                    r.push_back(node_start);
                    cost = 0;

                    for (int j=0; j<len+1; j++)
                    {
                        if (search2node(r,cost,all_way[i][j],all_way[i][j+1]) == -1)
                            break;
                    }
                    Router.push_back(r);
                    Cost[i] = cost;
                    mask[i] = judge_router(i);//计算环路(mask=1表示无环通路)
                    //cout << mask[i];
                }
                //cout << endl;

                //一级排序[无环路->有环路]
                int num_solver = 0; //无环路的个数
                for (int i=0; i<(2*num_way); i++)
                {
                    if (mask[i]) //无环路，优先
                    {
                        num_solver++;
                        all_way.insert(all_way.begin(), all_way[i]);
                        all_way.erase(all_way.begin() + (i+1));
                        Router.insert(Router.begin(), Router[i]);
                        Router.erase(Router.begin() + (i+1));
                        Cost.insert(Cost.begin(), Cost[i]);
                        Cost.erase(Cost.begin() + (i+1));
                        mask.insert(mask.begin(), mask[i]);
                        mask.erase(mask.begin() + (i+1));
                    }
                }

                //二级排序(初始化)[Cost优先]
                vector<int> INDEX;
                INDEX.resize(2*num_way);
                for (int i=0; i<(2*num_way); i++)
                {
                    INDEX[i] = i;
                }

                //二级排序(无环路排序)[0到num_solver-1]
                for (int i=0; i<num_solver; i++)
                {
                    for (int j=i+1; j<num_solver; j++)
                    {
                        if (Cost[i] > Cost[j])
                        {
                            Cost[i] = Cost[i]^Cost[j];
                            Cost[j] = Cost[i]^Cost[j];
                            Cost[i] = Cost[i]^Cost[j];
                            INDEX[i] = INDEX[i]^INDEX[j];
                            INDEX[j] = INDEX[i]^INDEX[j];
                            INDEX[i] = INDEX[i]^INDEX[j];
                        }
                    }
                }

                //二级排序(有环路排序)[num_solver到(2*num_way)]
                for (int i=num_solver; i<(2*num_way); i++)
                {
                    for (int j=i+1; j<(2*num_way); j++)
                    {
                        if (Cost[i] > Cost[j])
                        {
                            Cost[i] = Cost[i]^Cost[j];
                            Cost[j] = Cost[i]^Cost[j];
                            Cost[i] = Cost[i]^Cost[j];
                            INDEX[i] = INDEX[i]^INDEX[j];
                            INDEX[j] = INDEX[i]^INDEX[j];
                            INDEX[i] = INDEX[i]^INDEX[j];
                        }
                    }
                }

                //归位(仅有all_way和Router需要INDEX索引)
                for (int i=0; i<num_way; i++)
                {
                    int index = INDEX[i];
                    best_way[i] = all_way[index];
                }

                best_cost.push_back(Cost[0]); //push本次迭代的最优Cost
                best_index = INDEX[0];

                if (num_iter > 20) //终止迭代的一个条件
                {
                    if (best_cost[num_iter] == best_cost[num_iter-_STOP])
                    {
                        break;
                    }
                }//终止迭代条件
            }//1000次迭代的尾
            g_Cost.push_back(Cost[0]);
            g_Router_vp.push_back(Router[best_index]);
            g_Router_v.push_back(all_way[best_index]);
            g_mask.push_back(mask[0]);

            //时间判断
            struct timeb now_time;
            ftime(&now_time);
            int out_ms = now_time.millitm - init_ms;
            unsigned long out_s = now_time.time - init_s;
            if (out_ms < 0)
            {
                out_ms += 1000;
                out_s -= 1;
            }
            if (out_s >= 19 && out_ms >= 500) //如果达到9.5s
            {
                break;
            }
        } //num_test次实验的尾

    /////////////////////////////////////////////////////////////////////
        //判断是否无解
        if (judge_g_mask())
        {
            #ifdef _OUTPUT
            cout << "NA" << endl;
            #endif // _OUTPUT

            //找到最小Cost所对应的best_id
            vector<int> sort_cost;
            sort_cost = g_Cost;
            vector<int> sort_mask;
            sort_mask.resize(g_Cost.size());
            for (unsigned int i=0; i<sort_mask.size(); i++)
            {
                sort_mask[i] = i;
            }
            for (unsigned int i=0; i<sort_mask.size(); i++)
            {
                for (unsigned int j=i+1; j<sort_mask.size(); j++)
                {
                    if (sort_cost[i] > sort_cost[j])
                    {
                        sort_cost[i] = sort_cost[i]^sort_cost[j];
                        sort_cost[j] = sort_cost[i]^sort_cost[j];
                        sort_cost[i] = sort_cost[i]^sort_cost[j];
                        sort_mask[i] = sort_mask[i]^sort_mask[j];
                        sort_mask[j] = sort_mask[i]^sort_mask[j];
                        sort_mask[i] = sort_mask[i]^sort_mask[j];
                    }
                }
            }
            sort_cost.clear();
            int best_id = sort_mask[0];


            #ifdef _OUTPUT
            cout << "-------------------------------------------------------" << endl;
            //输出对应best_id的最优v点：
            cout << "best_id:" << best_id << endl;
            cout << "Best Point V: [Cost " << g_Cost[best_id] << " ]" << endl;
            for (unsigned int i=0; i<g_Router_v[best_id].size(); i++)
            {
                cout << g_Router_v[best_id][i];
                if (i < g_Router_v[best_id].size()-1)
                {
                    cout << " -> ";
                }
            }
            cout << endl;
            //输出对应best_id的最优vp点：
            cout << "Best Point VP: [Cost " << g_Cost[best_id] << " ]" << endl;
            for (unsigned int i=0; i<g_Router_vp[best_id].size(); i++)
            {
                cout << g_Router_vp[best_id][i];
                if (i < g_Router_vp[best_id].size()-1)
                {
                    cout << " -> ";
                }
            }
            cout << endl;
            cout << "-------------------------------------------------------" << endl;
            #endif // _OUTPUT

            int num_remove = 0;//找重复点实验次数
            int id_cir = 0;
            while (1 == g_vpoint(best_id)) //如果检测不到有重复点那么就退出循环
            {
                #ifdef _OUTPUT
                for (unsigned int i=0; i<g_Router_vp[best_id].size(); i++)
                {
                    cout << g_Router_vp[best_id][i];
                    if (i < g_Router_vp[best_id].size()-1)
                    {
                        cout << " -> ";
                    }
                }
                cout << endl;
                cout << V_point[0] << ',' << V_point[1] << endl;
                #endif

                if (1 == remove_repeat(best_id))    //如果best_id的最优路径依旧是成环的，那么就g_rout这个全局的路径中从头开始寻找，当然是否有其他的找其他路方式？
                {
                    #ifdef _OUTPUT
                    cout << "Fail to Search Router" << endl;
                    #endif
                    id_cir++;
                    best_id = sort_mask[id_cir];
                }
                num_remove ++;
                if (num_remove == 20)//超过20次找不到就不找了。就说明就是真的成环了。
                {
                    #ifdef _OUTPUT
                    cout<<"cannot find the route"<<endl;
                    #endif
                    break;
                }
                //cout << "-------------------------------------------------------" << endl;
            }

            //输出文件
            for (unsigned int i=0; i<g_Router_vp[best_id].size()-1; i++)
            {
                int line = find_line(g_Router_vp[best_id][i], g_Router_vp[best_id][i+1]);
                record_result(line);
            }

            return ;
        }//////////////////////////////////////////////////////
        //如果有解，继续执行

        //找到第一个mask为1的标号
        int best_id = 0;
        for (unsigned int i=0; i<g_mask.size(); i++)
        {
            if (g_mask[i] == 1)
            {
                best_id = i;
                break;
            }
        }
        //找到剩余mask==1中最小Cost的标号
        for (unsigned int i=best_id+1; i<g_mask.size(); i++)
        {
            if ((g_mask[i] == 1) && (g_Cost[i] < g_Cost[best_id]))
            {
                best_id = i;
            }
        }

        //若损耗无穷大，退出
        if (g_Cost[best_id] >= 65535)
        {
            #ifdef _OUTPUT
            cout << "NA" << endl;
            #endif // _OUTPUT
            return ;
        }

    #ifdef _OUTPUT
        cout << "-------------------------------------------------------" << endl;
        //输出对应best_id的最优点：
        cout << "Best Point: [Cost " << g_Cost[best_id] << " ]" << endl;
        for (unsigned int i=0; i<g_Router_vp[best_id].size(); i++)
        {
            cout << g_Router_vp[best_id][i];
            if (i < g_Router_vp[best_id].size()-1)
            {
                cout << " -> ";
            }
        }
        cout << endl;

        //输出对应的best_id的最优路径
        cout << "Best Line:" << endl;
        for (unsigned int i=0; i<g_Router_vp[best_id].size()-1; i++)
        {
            cout << find_line(g_Router_vp[best_id][i],g_Router_vp[best_id][i+1]);
            if (i < g_Router_vp[best_id].size()-2)
            {
                cout << '|';
            }
        }
        cout << endl << "-------------------------------------------------------" << endl;
    #endif

        //输出文件
        for (unsigned int i=0; i<g_Router_vp[best_id].size()-1; i++)
        {
            int line = find_line(g_Router_vp[best_id][i], g_Router_vp[best_id][i+1]);
            record_result(line);
        }
        return ;
    }
}
