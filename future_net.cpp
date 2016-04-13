#include "route.h"
#include "lib/lib_io.h"
#include "lib/lib_time.h"
#include "stdio.h"

int main(int argc, char *argv[])
{
    print_time("Begin");
    char *topo[5000];
    int edge_num;
    char *demand;
    int demand_num;

    //char *topo_file = argv[1];
    char topo_file[] = "test-case/case0/topo.csv";
    edge_num = read_file(topo, 5000, topo_file);
    if (edge_num == 0)
    {
        printf("Please input valid topo file.\n");
        return -1;
    }

    //char *demand_file = argv[2];
    char demand_file[] = "test-case/case0/demand.csv";
    demand_num = read_file(&demand, 1, demand_file);
    if (demand_num != 1)
    {
        printf("Please input valid demand file.\n");
        return -1;
    }

    search_route(topo, edge_num, demand);

    //char *result_file = argv[3];
    char result_file[] = "test-case/case0/result.csv";
    write_result(result_file);
    release_buff(topo, edge_num);
    release_buff(&demand, 1);

    print_time("End");

	return 0;
}

