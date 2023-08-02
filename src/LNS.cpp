#include "LNS.h"
#include "ECBS.h"
#include <queue>

LNS::LNS(const Instance &instance, double time_limit, const string &init_algo_name, const string &replan_algo_name,
         const string &destory_name, int neighbor_size, int num_of_iterations, bool use_init_lns,
         const string &init_destory_name, bool use_sipp, int screen, PIBTPPS_option pipp_option) : BasicLNS(instance, time_limit, neighbor_size, screen),
                                                                                                   init_algo_name(init_algo_name), replan_algo_name(replan_algo_name), num_of_iterations(num_of_iterations),
                                                                                                   use_init_lns(use_init_lns), init_destory_name(init_destory_name),
                                                                                                   path_table(instance.map_size), pipp_option(pipp_option)
{
    start_time = Time::now();
    replan_time_limit = time_limit / 100;
    if (destory_name == "Adaptive")
    {
        ALNS = true;
        destroy_weights.assign(DESTORY_COUNT, 1);
        decay_factor = 0.01;
        reaction_factor = 0.01;
    }
    else if (destory_name == "RandomWalk")
        destroy_strategy = RANDOMWALK;
    else if (destory_name == "Intersection")
        destroy_strategy = INTERSECTION;
    else if (destory_name == "Random")
        destroy_strategy = RANDOMAGENTS;
    else
    {
        cerr << "Destroy heuristic " << destory_name << " does not exists. " << endl;
        exit(-1);
    }

    int N = instance.getDefaultNumberOfAgents();
    agents.reserve(N);
    for (int i = 0; i < N; i++)
        agents.emplace_back(instance, i, use_sipp);
    preprocessing_time = ((fsec)(Time::now() - start_time)).count();
    if (screen >= 2)
        cout << "Pre-processing time = " << preprocessing_time << " seconds." << endl;
}

bool LNS::run()
{
    // only for statistic analysis, and thus is not included in runtime
    sum_of_distances = 0;
    for (const auto &agent : agents)
    {
        sum_of_distances += agent.path_planner->my_heuristic[agent.path_planner->start_location];
    }

    initial_solution_runtime = 0;
    start_time = Time::now();
    bool succ = getInitialSolution();
    initial_solution_runtime = ((fsec)(Time::now() - start_time)).count();
    runtime = initial_solution_runtime;
    return succ;
    if (!succ && initial_solution_runtime < time_limit)
    {
        if (use_init_lns)
        {
            init_lns = new InitLNS(instance, agents, time_limit - initial_solution_runtime,
                                   replan_algo_name, init_destory_name, neighbor_size, screen);
            succ = init_lns->run();
            if (succ) // accept new paths
            {
                path_table.reset();
                for (const auto &agent : agents)
                {
                    path_table.insertPath(agent.id, agent.path);
                }
                init_lns->clear();
                initial_sum_of_costs = init_lns->sum_of_costs;
                sum_of_costs = initial_sum_of_costs;
            }
            initial_solution_runtime = ((fsec)(Time::now() - start_time)).count();
        }
        else // use random restart
        {
            while (!succ && initial_solution_runtime < time_limit)
            {
                succ = getInitialSolution();
                initial_solution_runtime = ((fsec)(Time::now() - start_time)).count();
                restart_times++;
            }
        }
    }

    iteration_stats.emplace_back(neighbor.agents.size(),
                                 initial_sum_of_costs, initial_solution_runtime, init_algo_name);
    runtime = initial_solution_runtime;
    if (succ)
    {
        if (screen >= 1)
            cout << "Initial solution cost = " << initial_sum_of_costs << ", "
                 << "runtime = " << initial_solution_runtime << endl;
    }
    else
    {
        cout << "Failed to find an initial solution in "
             << runtime << " seconds after  " << restart_times << " restarts" << endl;
        return false; // terminate because no initial solution is found
    }

    while (runtime < time_limit && iteration_stats.size() <= num_of_iterations)
    {
        runtime = ((fsec)(Time::now() - start_time)).count();
        if (screen >= 1)
            validateSolution();
        if (ALNS)
            chooseDestroyHeuristicbyALNS();

        switch (destroy_strategy)
        {
        case RANDOMWALK:
            succ = generateNeighborByRandomWalk();
            break;
        case INTERSECTION:
            succ = generateNeighborByIntersection();
            break;
        case RANDOMAGENTS:
            neighbor.agents.resize(agents.size());
            for (int i = 0; i < (int)agents.size(); i++)
                neighbor.agents[i] = i;
            if (neighbor.agents.size() > neighbor_size)
            {
                std::random_shuffle(neighbor.agents.begin(), neighbor.agents.end());
                neighbor.agents.resize(neighbor_size);
            }
            succ = true;
            break;
        default:
            cerr << "Wrong neighbor generation strategy" << endl;
            exit(-1);
        }
        if (!succ)
            continue;

        // store the neighbor information
        neighbor.old_paths.resize(neighbor.agents.size());
        neighbor.old_sum_of_costs = 0;
        for (int i = 0; i < (int)neighbor.agents.size(); i++)
        {
            if (replan_algo_name == "PP")
                neighbor.old_paths[i] = agents[neighbor.agents[i]].path;
            path_table.deletePath(neighbor.agents[i], agents[neighbor.agents[i]].path);
            neighbor.old_sum_of_costs += agents[neighbor.agents[i]].path.size() - 1;
        }

        if (replan_algo_name == "EECBS")
            succ = runEECBS();
        else if (replan_algo_name == "CBS")
            succ = runCBS();
        else if (replan_algo_name == "PP")
            succ = runPP();
        else
        {
            cerr << "Wrong replanning strategy" << endl;
            exit(-1);
        }

        if (ALNS) // update destroy heuristics
        {
            if (neighbor.old_sum_of_costs > neighbor.sum_of_costs)
                destroy_weights[selected_neighbor] =
                    reaction_factor * (neighbor.old_sum_of_costs - neighbor.sum_of_costs) / neighbor.agents.size() + (1 - reaction_factor) * destroy_weights[selected_neighbor];
            else
                destroy_weights[selected_neighbor] =
                    (1 - decay_factor) * destroy_weights[selected_neighbor];
        }
        runtime = ((fsec)(Time::now() - start_time)).count();
        sum_of_costs += neighbor.sum_of_costs - neighbor.old_sum_of_costs;
        if (screen >= 1)
            cout << "Iteration " << iteration_stats.size() << ", "
                 << "group size = " << neighbor.agents.size() << ", "
                 << "solution cost = " << sum_of_costs << ", "
                 << "remaining time = " << time_limit - runtime << endl;
        iteration_stats.emplace_back(neighbor.agents.size(), sum_of_costs, runtime, replan_algo_name);
    }

    average_group_size = -iteration_stats.front().num_of_agents;
    for (const auto &data : iteration_stats)
        average_group_size += data.num_of_agents;
    if (average_group_size > 0)
        average_group_size /= (double)(iteration_stats.size() - 1);

    cout << getSolverName() << ": "
         << "runtime = " << runtime << ", "
         << "iterations = " << iteration_stats.size() << ", "
         << "solution cost = " << sum_of_costs << ", "
         << "initial solution cost = " << initial_sum_of_costs << ", "
         << "failed iterations = " << num_of_failures << endl;
    return true;
}

bool LNS::getInitialSolution()
{
    neighbor.agents.resize(agents.size());
    for (int i = 0; i < (int)agents.size(); i++)
        neighbor.agents[i] = i;
    neighbor.old_sum_of_costs = MAX_COST;
    neighbor.sum_of_costs = 0;
    bool succ = false;
    if (init_algo_name == "EECBS")
        succ = runEECBS();
    else if (init_algo_name == "PP")
        succ = runPP(agent_selection_mode);
    else if (init_algo_name == "PIBT")
        succ = runPIBT();
    else if (init_algo_name == "PPS")
        succ = runPPS();
    else if (init_algo_name == "winPIBT")
        succ = runWinPIBT();
    else if (init_algo_name == "CBS")
        succ = runCBS();
    else
    {
        cerr << "Initial MAPF solver " << init_algo_name << " does not exist!" << endl;
        exit(-1);
    }
    if (succ)
    {
        initial_sum_of_costs = neighbor.sum_of_costs;
        sum_of_costs = neighbor.sum_of_costs;
        return true;
    }
    else
    {
        return false;
    }
}

bool LNS::runEECBS()
{
    vector<SingleAgentSolver *> search_engines;
    search_engines.reserve(neighbor.agents.size());
    for (int i : neighbor.agents)
    {
        search_engines.push_back(agents[i].path_planner);
    }

    ECBS ecbs(search_engines, screen - 1, &path_table);
    ecbs.setPrioritizeConflicts(true);
    ecbs.setDisjointSplitting(false);
    ecbs.setBypass(true);
    ecbs.setRectangleReasoning(true);
    ecbs.setCorridorReasoning(true);
    ecbs.setHeuristicType(heuristics_type::WDG, heuristics_type::GLOBAL);
    ecbs.setTargetReasoning(true);
    ecbs.setMutexReasoning(false);
    ecbs.setConflictSelectionRule(conflict_selection::EARLIEST);
    ecbs.setNodeSelectionRule(node_selection::NODE_CONFLICTPAIRS);
    ecbs.setSavingStats(false);
    double w;
    if (iteration_stats.empty())
        w = 5; // initial run
    else
        w = 1.1; // replan
    ecbs.setHighLevelSolver(high_level_solver_type::EES, w);
    runtime = ((fsec)(Time::now() - start_time)).count();
    double T = time_limit - runtime;
    if (!iteration_stats.empty()) // replan
        T = min(T, replan_time_limit);
    bool succ = ecbs.solve(T, 0);
    if (succ && ecbs.solution_cost < neighbor.old_sum_of_costs) // accept new paths
    {
        auto id = neighbor.agents.begin();
        for (size_t i = 0; i < neighbor.agents.size(); i++)
        {
            agents[*id].path = *ecbs.paths[i];
            path_table.insertPath(agents[*id].id, agents[*id].path);
            ++id;
        }
        neighbor.sum_of_costs = ecbs.solution_cost;
        if (sum_of_costs_lowerbound < 0)
            sum_of_costs_lowerbound = ecbs.getLowerBound();
    }
    else // stick to old paths
    {
        if (!neighbor.old_paths.empty())
        {
            for (int id : neighbor.agents)
            {
                path_table.insertPath(agents[id].id, agents[id].path);
            }
            neighbor.sum_of_costs = neighbor.old_sum_of_costs;
        }
        if (!succ)
            num_of_failures++;
    }
    return succ;
}
bool LNS::runCBS()
{
    if (screen >= 2)
        cout << "old sum of costs = " << neighbor.old_sum_of_costs << endl;
    vector<SingleAgentSolver *> search_engines;
    search_engines.reserve(neighbor.agents.size());
    for (int i : neighbor.agents)
    {
        search_engines.push_back(agents[i].path_planner);
    }

    CBS cbs(search_engines, screen - 1, &path_table);
    cbs.setPrioritizeConflicts(true);
    cbs.setDisjointSplitting(false);
    cbs.setBypass(true);
    cbs.setRectangleReasoning(true);
    cbs.setCorridorReasoning(true);
    cbs.setHeuristicType(heuristics_type::WDG, heuristics_type::ZERO);
    cbs.setTargetReasoning(true);
    cbs.setMutexReasoning(false);
    cbs.setConflictSelectionRule(conflict_selection::EARLIEST);
    cbs.setNodeSelectionRule(node_selection::NODE_CONFLICTPAIRS);
    cbs.setSavingStats(false);
    cbs.setHighLevelSolver(high_level_solver_type::ASTAR, 1);
    runtime = ((fsec)(Time::now() - start_time)).count();
    double T = time_limit - runtime; // time limit
    if (!iteration_stats.empty())    // replan
        T = min(T, replan_time_limit);
    bool succ = cbs.solve(T, 0);
    if (succ && cbs.solution_cost <= neighbor.old_sum_of_costs) // accept new paths
    {
        auto id = neighbor.agents.begin();
        for (size_t i = 0; i < neighbor.agents.size(); i++)
        {
            agents[*id].path = *cbs.paths[i];
            path_table.insertPath(agents[*id].id, agents[*id].path);
            ++id;
        }
        neighbor.sum_of_costs = cbs.solution_cost;
        if (sum_of_costs_lowerbound < 0)
            sum_of_costs_lowerbound = cbs.getLowerBound();
    }
    else // stick to old paths
    {
        if (!neighbor.old_paths.empty())
        {
            for (int id : neighbor.agents)
            {
                path_table.insertPath(agents[id].id, agents[id].path);
            }
            neighbor.sum_of_costs = neighbor.old_sum_of_costs;
        }
        if (!succ)
            num_of_failures++;
    }
    return succ;
}

LL LNS::getNumPathChoice(int x, vector<TMatrix> &adj_matrix)
{
    int map_size = instance.map_size;
    TMatrix b = TMatrix(map_size, 1);
    LL ans = 0;
    b.val[instance.getStart(x)][0] = 1;
    int k = -1;

    // cerr<<x<<" ";
    for (int i = 0; i < adj_matrix.size() && k < 5; i++)
    {
        b = adj_matrix[i] * b;
        if (b.val[instance.getGoal(x)][0] > 0)
        {
            if (shortest_path_length[x] == -1)
                shortest_path_length[x] = i + 1;
            if (k < 0)
                k = 0;
            ans += b.val[instance.getGoal(x)][0];
            // cerr<<i<<" "<<b.val[instance.getGoal(x)][0]<<endl;
        }
        if (k >= 0)
            k++;
    }
    return ans;
}

int LNS::selectAgent(set<int> &remaining_agents_set, vector<TMatrix> &adj_matrix)
{
    int selected_agent = -1;
    LL min_path_choice = -1;
    shortest_path_length.clear();
    shortest_path_length.resize(instance.map_size, -1);
    int agent_shortest_path_length = 1e8;
    // cerr<<"=================== "<<remaining_agents_set.size()<<endl;
    for (auto _ : remaining_agents_set)
    {
        int x = agents[_].id;
        LL num_path_choice = getNumPathChoice(x, adj_matrix);
        if (min_path_choice == -1 || min_path_choice > num_path_choice)
            min_path_choice = num_path_choice,
            selected_agent = _;
        agent_shortest_path_length = min(agent_shortest_path_length, shortest_path_length[x]);
    }
    // cerr<<agent_shortest_path_length <<" "<<shortest_path_length[selected_agent]<<endl;
    if (agent_shortest_path_length == shortest_path_length[selected_agent])
        count_pick_shortest_path++;
    /*for (auto _:remaining_agents_set)
    {
        int x = agents[_].id;
        if (shortest_path_length[x] == agent_shortest_path_length)
            return _;
    }*/
    return selected_agent;
}

int LNS::selectAgentBasedonShortestPath(set<int> &remaining_agents_set, ConstraintTable &constraint_table, int mode)
{
    int agent_shortest_path_length = mode == 2 ? 1e8 : 0, selected_agent = -1;
    for (auto _ : remaining_agents_set)
    {
        int id = agents[_].id;
        agents[id].path = agents[id].path_planner->findPath(constraint_table);
        if (agents[id].path.empty())
            return -1;
        int path_length = (int)agents[id].path.size() - 1;
        if ((path_length < agent_shortest_path_length && mode == 2) or (path_length > agent_shortest_path_length && mode == 3))
        {
            agent_shortest_path_length = path_length;
            selected_agent = _;
        }
    }
    return selected_agent;
}

int LNS::selectAgentBasedonMDD(set<int> &remaining_agents_set, ConstraintTable &constraint_table, int mode)
{
    // int selected_agent = -1;
    // vector<int> id_list;
    // id_list.clear();
    // for (auto id : remaining_agents_set)
    //     id_list.push_back(id);
    // // printf("\n");

    // // for(auto i = remaining_agents_set.begin(); i != remaining_agents_set.end(); i++)
    // // random_shuffle(id_list.begin(),id_list.end());
    // // for (auto id:id_list)
    // int num_remaining_agents = int(id_list.size());
    // int min_mdd_node = 1000000000;
    // for (int i = 0; i < num_remaining_agents; i++)
    // {
    //     //int id = *i;
    //     printf("Iteration %d ", i);
    //     int id = id_list[i];
    //     printf("start agent %d of %d;", id, remaining_agents_set.size());
    //     agents[id].path = agents[id].path_planner->findPath(constraint_table);
    //     if (agents[id].path.empty()) // Check if path is valid and non-empty
    //     {
    //         printf("No valid path found for agent %d. Exiting.\n", id);
    //         return -1;
    //     }
    //     bool successfully_built_mdd = false;
    //     if (agents[id].mdd.levels.empty() || mode == 5)
    //         successfully_built_mdd = agents[id].mdd.buildMDD(constraint_table, agents[id].path_planner, min_mdd_node);
    //     // selected_agent = id;
    //     if (!successfully_built_mdd)
    //         continue;
    //     int mdd_size = 0;
    //     for (auto &level : agents[id].mdd.levels)
    //         mdd_size += int(level.size());
    //     int mdd_depth = agents[id].mdd.levels.size();
    //     if (selected_agent == -1 || min_mdd_node > mdd_size) // 5
    //     {
    //         selected_agent = id;
    //         min_mdd_node = mdd_size;
    //     }
    //     printf("built mdd for agesnt %d (%d) with depth %d and size %d\n", id, agents[id].id, mdd_depth, mdd_size);
    //     // if (i + 1 < num_remaining_agents) printf("next agent is %d; ", id_list[i+1]);
    //     if (mode != 6)
    //         agents[id].mdd.clear();
    //     printf("%d ", i);
    //     if (i + 1 < num_remaining_agents)
    //         printf("next agent is %d; ", id_list[i + 1]);
    // }
    // printf("selection done: selected agent %d\n", selected_agent);
    // return selected_agent;
    // int selected_agent = -1;
    // vector<int> id_list;
    // id_list.clear();
    // for (auto id : remaining_agents_set)
    //     id_list.push_back(id);

    // int num_remaining_agents = int(id_list.size());
    // int max_overlap_count = -1; // store maximum overlap count

    // for (int i = 0; i < num_remaining_agents; i++)
    // {
    //     int id = id_list[i];
    //     printf("start agent %d of %d;", id, remaining_agents_set.size());
    //     agents[id].path = agents[id].path_planner->findPath(constraint_table);
    //     if (agents[id].path.empty()) // Check if path is valid and non-empty
    //     {
    //         printf("No valid path found for agent %d. Exiting.\n", id);
    //         return -1;
    //     }
    //     bool successfully_built_mdd = false;
    //     if (agents[id].mdd.levels.empty() || mode == 5)
    //         successfully_built_mdd = agents[id].mdd.buildMDD(constraint_table, agents[id].path_planner, max_overlap_count); // replace 'min_mdd_node' with 'max_overlap_count'

    //     if (!successfully_built_mdd)
    //         continue;

    //     // Calculate overlap count
    //     int overlap_count = 0;
    //     for (auto &level : agents[id].mdd.levels)
    //     {
    //         if (level.size() == 1)
    //         {                                             // this is a unit length level
    //             auto level_loc = level.front()->location; // use list's front() method to access the first element
    //             for (int j = 0; j < num_remaining_agents; j++)
    //             {
    //                 if (i == j)
    //                     continue; // skip comparing with itself
    //                 for (auto &other_level : agents[id_list[j]].mdd.levels)
    //                 {
    //                     if (other_level.size() != 1)
    //                         continue;                                         // only compare with unit length levels
    //                     auto other_level_loc = other_level.front()->location; // use list's front() method to access the first element
    //                     if (level_loc == other_level_loc)
    //                     {
    //                         overlap_count++;
    //                         break; // no need to keep checking once we found an overlap
    //                     }
    //                 }
    //             }
    //         }
    //     }

    //     if (max_overlap_count < overlap_count)
    //     {
    //         selected_agent = id;
    //         max_overlap_count = overlap_count;
    //     }

    //     printf("built mdd for agent %d (%d) with overlap count %d\n", id, agents[id].id, overlap_count);

    //     if (mode != 6)
    //         agents[id].mdd.clear();
    // }

    // printf("selection done: selected agent %d\n", selected_agent);
    // return selected_agent;
    // int selected_agent = -1;
    // vector<int> id_list;
    // for (auto id : remaining_agents_set)
    //     id_list.push_back(id);

    // int num_remaining_agents = int(id_list.size());
    // int conflict_count = 0;

    // for (int i = 0; i < num_remaining_agents; i++)
    // {
    //     int id = id_list[i];
    //     agents[id].path = agents[id].path_planner->findPath(constraint_table);
    //     if (agents[id].path.empty()) // Check if path is valid and non-empty
    //     {
    //         printf("No valid path found for agent %d. Exiting.\n", id);
    //         return -1;
    //     }
    //     if (agents[id].mdd.levels.empty() || mode == 5)
    //         agents[id].mdd.buildMDD(constraint_table, agents[id].path_planner, num_remaining_agents);

    //     int agent_conflict_count = 0; // Variable to store the conflict count for the current agent

    //     for (int j = 0; j < num_remaining_agents; j++)
    //     {
    //         if (i == j)
    //             continue; // Skip comparing with self

    //         if (agents[id_list[j]].mdd.levels.empty()) // If the other agent's MDD is not built yet
    //             continue;

    //         // For each level in the MDD, check if the locations in the level conflict with the locations in the other agent's MDD
    //         for (auto &level : agents[id].mdd.levels)
    //         {
    //             for (auto &node : level)
    //             {
    //                 // Make sure the other agent's MDD has the same level
    //                 if (agents[id_list[j]].mdd.levels.size() > level.size())
    //                 {
    //                     for (auto &other_node : agents[id_list[j]].mdd.levels[level.size()])
    //                     {
    //                         if (node->location == other_node->location)
    //                             agent_conflict_count++;
    //                     }
    //                 }
    //             }
    //         }
    //     }

    //     if (selected_agent == -1 || agent_conflict_count > conflict_count) // Choose the agent with the highest conflict count
    //     {
    //         selected_agent = id;
    //         conflict_count = agent_conflict_count;
    //     }
    // }

    // return selected_agent;
    // int selected_agent = -1;
    // vector<int> id_list;
    // id_list.clear();
    // for (auto id : remaining_agents_set)
    //     id_list.push_back(id);

    // int num_remaining_agents = int(id_list.size());
    // double min_mdd_product = std::numeric_limits<double>::max(); // Start with max double
    // for (int i = 0; i < num_remaining_agents; i++)
    // {
    //     printf("Iteration %d ", i);
    //     int id = id_list[i];
    //     printf("start agent %d of %d;", id, remaining_agents_set.size());
    //     agents[id].path = agents[id].path_planner->findPath(constraint_table);
    //     if (agents[id].path.empty()) // Check if path is valid and non-empty
    //     {
    //         printf("No valid path found for agent %d. Exiting.\n", id);
    //         return -1;
    //     }
    //     bool successfully_built_mdd = false;
    //     if (agents[id].mdd.levels.empty() || mode == 5)
    //         successfully_built_mdd = agents[id].mdd.buildMDD(constraint_table, agents[id].path_planner, min_mdd_product);
    //     if (!successfully_built_mdd)
    //         continue;

    //     double mdd_product = 1;
    //     for (auto &level : agents[id].mdd.levels)
    //         mdd_product *= level.size();

    //     int mdd_depth = agents[id].mdd.levels.size();
    //     if (selected_agent == -1 || min_mdd_product < mdd_product)
    //     {
    //         selected_agent = id;
    //         min_mdd_product = mdd_product;
    //     }
    //     printf("built mdd for agent %d (%d) with depth %d and size %lf\n", id, agents[id].id, mdd_depth, mdd_product);
    //     if (mode != 6)
    //         agents[id].mdd.clear();
    //     printf("%d ", i);
    //     if (i + 1 < num_remaining_agents)
    //         printf("next agent is %d; ", id_list[i + 1]);
    // }
    // printf("selection done: selected agent %d\n", selected_agent);
    // return selected_agent;
    int selected_agent = -1;
    vector<int> id_list;
    for (auto id : remaining_agents_set)
        id_list.push_back(id);

    int num_remaining_agents = int(id_list.size());
    int max_unit_width_levels = -1; // Initialize to a value that will be smaller than any possible count

    for (int i = 0; i < num_remaining_agents; i++)
    {
        printf("Iteration %d ", i);
        int id = id_list[i];
        printf("start agent %d of %d;", id, remaining_agents_set.size());
        agents[id].path = agents[id].path_planner->findPath(constraint_table);
        if (agents[id].path.empty()) // Check if path is valid and non-empty
        {
            printf("No valid path found for agent %d. Exiting.\n", id);
            return -1;
        }
        if (agents[id].mdd.levels.empty() || mode == 5)
            agents[id].mdd.buildMDD(constraint_table, agents[id].path_planner, max_unit_width_levels);

        int unit_width_levels = 0;
        for (auto &level : agents[id].mdd.levels)
            if (level.size() == 1) // This level is of unit-width
                unit_width_levels += 1;

        if (max_unit_width_levels < unit_width_levels)
        {
            selected_agent = id;
            max_unit_width_levels = unit_width_levels;
        }

        printf("built mdd for agesnt %d (%d) with unit-width levels %d\n", id, agents[id].id, unit_width_levels);

        if (mode != 6)
            agents[id].mdd.clear();
    }

    printf("selection done: selected agent %d\n", selected_agent);
    return selected_agent;
    // int selected_agent = -1;
    // vector<int> id_list;
    // for (auto id : remaining_agents_set)
    //     id_list.push_back(id);

    // int num_remaining_agents = int(id_list.size());
    // double max_ratio = -1.0; // Initialize to a value that will be smaller than any possible ratio

    // for (int i = 0; i < num_remaining_agents; i++)
    // {
    //     printf("Iteration %d ", i);
    //     int id = id_list[i];
    //     printf("start agent %d of %d;", id, remaining_agents_set.size());
    //     agents[id].path = agents[id].path_planner->findPath(constraint_table);
    //     if (agents[id].path.empty()) // Check if path is valid and non-empty
    //     {
    //         printf("No valid path found for agent %d. Exiting.\n", id);
    //         return -1;
    //     }
    //     if (agents[id].mdd.levels.empty() || mode == 5)
    //         agents[id].mdd.buildMDD(constraint_table, agents[id].path_planner, max_ratio);

    //     int unit_width_levels = 0;
    //     int total_size = 0;
    //     for (auto &level : agents[id].mdd.levels)
    //     {
    //         if (level.size() == 1) // This level is of unit-width
    //             unit_width_levels += 1;
    //         total_size += level.size();
    //     }

    //     double ratio = (double)unit_width_levels / total_size;
    //     if (max_ratio < ratio)
    //     {
    //         selected_agent = id;
    //         max_ratio = ratio;
    //     }

    //     printf("built mdd for agesnt %d (%d) with unit-width levels ratio %f\n", id, agents[id].id, ratio);

    //     if (mode != 6)
    //         agents[id].mdd.clear();
    // }

    // printf("selection done: selected agent %d\n", selected_agent);
    // return selected_agent;
}

void LNS::modifyAdjMatrix(const Path &path, vector<TMatrix> &adj_matrix)
{
    for (int t = 0; t < min((int)path.size(), int(adj_matrix.size())); t++)
    {
        int i = path[t].location;
        auto i_neighbors = instance.getNeighbors(i);
        i_neighbors.emplace_back(i);
        if (t + 1 < (int)path.size())
            adj_matrix[t].val[path[t + 1].location][i] = 0;
        for (auto j : i_neighbors)
        {
            adj_matrix[t].val[i][j] = 0;
            if (t)
            {
                adj_matrix[t - 1].val[j][i] = 0;
            }
        }
        if (t + 1 == (int)path.size())
        {
            for (t++; t < (int)adj_matrix.size(); t++)
            {
                for (auto j : i_neighbors)
                {
                    adj_matrix[t].val[i][j] = 0;
                    if (t)
                        adj_matrix[t - 1].val[j][i] = 0;
                }
            }
        }
    }
}

bool LNS::runPP(int agent_selection_mode) // agent_selection_mode: 0 random 1 MatrixMAPF 2: shortest shortest path 3: longest shortest path 4: longest path fixed
{
    auto shuffled_agents = neighbor.agents;

    srand(pseed);
    if (pshuffle > 1e-8 && agent_selection_mode == 0)
    {
        int n_partial_agents = pshuffle < 1 ? int(pshuffle * shuffled_agents.size()) : pshuffle;
        // for (auto x:shuffled_agents)
        //     cout<<x<<" ";cout<<endl;
        int tp = int(shuffled_agents.size()) - n_partial_agents;
        random_shuffle(shuffled_agents.begin() + tp, shuffled_agents.end());
        // for (auto x:shuffled_agents)
        //     cout<<x<<" ";cout<<endl;
    }
    else if (pshuffle < -1e-8 && agent_selection_mode == 0)
    {
        int n_partial_agents = pshuffle > -1 ? int(-pshuffle * shuffled_agents.size()) : -pshuffle;
        for (auto x : shuffled_agents)
            cout << x << " ";
        cout << endl;
        // int tp = int(shuffled_agents.size()) - n_partial_agents;
        random_shuffle(shuffled_agents.begin(), shuffled_agents.begin() + n_partial_agents);
        for (auto x : shuffled_agents)
            cout << x << " ";
        cout << endl;
    }

    set<int> remaining_agents_set;
    remaining_agents_set.clear();
    for (auto x : shuffled_agents)
        remaining_agents_set.insert(x);

    std::random_shuffle(shuffled_agents.begin(), shuffled_agents.end());
    if (screen >= 2)
    {
        for (auto id : shuffled_agents)
            cout << id << "(" << agents[id].path_planner->my_heuristic[agents[id].path_planner->start_location] << "->" << agents[id].path.size() - 1 << "), ";
        cout << endl;
    }
    int remaining_agents = (int)shuffled_agents.size();

    if (agent_selection_mode == 4)
    {
        vector<pair<int, int>> tp_sort;
        tp_sort.clear();
        for (auto id : shuffled_agents)
        {
            tp_sort.push_back(make_pair(-agents[id].path_planner->my_heuristic[agents[id].path_planner->start_location], id));
        }
        sort(tp_sort.begin(), tp_sort.end());
        for (int i = 0; i < (int)shuffled_agents.size(); i++)
            shuffled_agents[i] = tp_sort[i].second;
    }

    auto p = shuffled_agents.begin();
    neighbor.sum_of_costs = 0;
    runtime = ((fsec)(Time::now() - start_time)).count();
    double T = time_limit - runtime; // time limit
    if (!iteration_stats.empty())    // replan
        T = min(T, replan_time_limit);
    auto time = Time::now();
    ConstraintTable constraint_table(instance.num_of_cols, instance.map_size, &path_table);

    // for agent_selection_mode = 1
    int N = instance.getDefaultNumberOfAgents();
    int make_span = instance.num_of_cols + instance.num_of_rows + N / 4;
    int map_size = instance.map_size;
    vector<TMatrix> adj_matrix;
    TMatrix adj0;
    if (agent_selection_mode == 1)
    {
        adj_matrix.resize(make_span, TMatrix(map_size, map_size));
        adj0 = TMatrix(map_size, map_size);
        for (int i = 0; i < map_size; i++)
        {
            auto i_neighbors = instance.getNeighbors(i);
            adj0.val[i][i] = 1;
            for (auto j : i_neighbors)
                adj0.val[i][j] = 1;
        }
        for (int i = 0; i < make_span; i++)
            adj_matrix[i] = adj0;
    }

    int i_shuffled = 0;
    while (remaining_agents > 0 && ((fsec)(Time::now() - time)).count() < T)
    {
        int id = *p;
        if (agent_selection_mode == 1)
            id = selectAgent(remaining_agents_set, adj_matrix);
        else if (agent_selection_mode == 2 || agent_selection_mode == 3)
            id = selectAgentBasedonShortestPath(remaining_agents_set, constraint_table, agent_selection_mode);
        else if (agent_selection_mode >= 5)
            id = selectAgentBasedonMDD(remaining_agents_set, constraint_table, agent_selection_mode);
        // cerr<<"_+";
        if (id == -1)
            break;
        assert(remaining_agents_set.find(id) != remaining_agents_set.end());
        remaining_agents_set.erase(id);
        if (screen >= 3)
            cout << "Remaining agents = " << remaining_agents << ", remaining time = " << T - ((fsec)(Time::now() - time)).count() << " seconds. " << endl
                 << "Agent " << agents[id].id << endl;
        // cerr<<"_";
        agents[id].path = agents[id].path_planner->findPath(constraint_table);
        if (agents[id].path.empty())
        {
            cerr << -1 << endl;
            break;
        }
        neighbor.sum_of_costs += (int)agents[id].path.size() - 1;
        // cerr<<(int)agents[id].path.size() - 1;
        if (neighbor.sum_of_costs >= neighbor.old_sum_of_costs)
            break;
        remaining_agents--;
        if (agent_selection_mode == 1)
            modifyAdjMatrix(agents[id].path, adj_matrix);
        // cerr<<"_";
        path_table.insertPath(agents[id].id, agents[id].path);
        // cerr<<"_"<<endl;
        ++p;
    }
    // cerr<<count_pick_shortest_path<<endl;
    if (remaining_agents == 0 && neighbor.sum_of_costs <= neighbor.old_sum_of_costs) // accept new paths
    {
        return true;
    }
    else // stick to old paths
    {
        return false;
        if (p != shuffled_agents.end())
            num_of_failures++;
        auto p2 = shuffled_agents.begin();
        while (p2 != p)
        {
            int a = *p2;
            path_table.deletePath(agents[a].id, agents[a].path);
            ++p2;
        }
        if (!neighbor.old_paths.empty())
        {
            p2 = neighbor.agents.begin();
            for (int i = 0; i < (int)neighbor.agents.size(); i++)
            {
                int a = *p2;
                agents[a].path = neighbor.old_paths[i];
                path_table.insertPath(agents[a].id, agents[a].path);
                ++p2;
            }
            neighbor.sum_of_costs = neighbor.old_sum_of_costs;
        }
        return false;
    }
}
bool LNS::runPPS()
{
    auto shuffled_agents = neighbor.agents;
    std::random_shuffle(shuffled_agents.begin(), shuffled_agents.end());

    MAPF P = preparePIBTProblem(shuffled_agents);
    P.setTimestepLimit(pipp_option.timestepLimit);

    // seed for solver
    auto *MT_S = new std::mt19937(0);
    PPS solver(&P, MT_S);
    solver.setTimeLimit(time_limit);
    //    solver.WarshallFloyd();
    bool result = solver.solve();
    if (result)
        updatePIBTResult(P.getA(), shuffled_agents);
    return result;
}
bool LNS::runPIBT()
{
    auto shuffled_agents = neighbor.agents;
    std::random_shuffle(shuffled_agents.begin(), shuffled_agents.end());

    MAPF P = preparePIBTProblem(shuffled_agents);

    // seed for solver
    auto MT_S = new std::mt19937(0);
    PIBT solver(&P, MT_S);
    solver.setTimeLimit(time_limit);
    bool result = solver.solve();
    if (result)
        updatePIBTResult(P.getA(), shuffled_agents);
    return result;
}
bool LNS::runWinPIBT()
{
    auto shuffled_agents = neighbor.agents;
    std::random_shuffle(shuffled_agents.begin(), shuffled_agents.end());

    MAPF P = preparePIBTProblem(shuffled_agents);
    P.setTimestepLimit(pipp_option.timestepLimit);

    // seed for solver
    auto MT_S = new std::mt19937(0);
    winPIBT solver(&P, pipp_option.windowSize, pipp_option.winPIBTSoft, MT_S);
    solver.setTimeLimit(time_limit);
    bool result = solver.solve();
    if (result)
        updatePIBTResult(P.getA(), shuffled_agents);
    return result;
}

MAPF LNS::preparePIBTProblem(vector<int> &shuffled_agents)
{

    // seed for problem and graph
    auto MT_PG = new std::mt19937(0);

    //    Graph* G = new SimpleGrid(instance);
    Graph *G = new SimpleGrid(instance.getMapFile());

    std::vector<Task *> T;
    PIBT_Agents A;

    for (int i : shuffled_agents)
    {
        assert(G->existNode(agents[i].path_planner->start_location));
        assert(G->existNode(agents[i].path_planner->goal_location));
        auto a = new PIBT_Agent(G->getNode(agents[i].path_planner->start_location));

        //        PIBT_Agent* a = new PIBT_Agent(G->getNode( agents[i].path_planner.start_location));
        A.push_back(a);
        Task *tau = new Task(G->getNode(agents[i].path_planner->goal_location));

        T.push_back(tau);
        if (screen >= 5)
        {
            cout << "Agent " << i << " start: " << a->getNode()->getPos() << " goal: " << tau->getG().front()->getPos() << endl;
        }
    }

    return MAPF(G, A, T, MT_PG);
}

void LNS::updatePIBTResult(const PIBT_Agents &A, vector<int> &shuffled_agents)
{
    int soc = 0;
    for (int i = 0; i < A.size(); i++)
    {
        int a_id = shuffled_agents[i];

        agents[a_id].path.resize(A[i]->getHist().size());
        int last_goal_visit = 0;
        if (screen >= 2)
            std::cout << A[i]->logStr() << std::endl;
        for (int n_index = 0; n_index < A[i]->getHist().size(); n_index++)
        {
            auto n = A[i]->getHist()[n_index];
            agents[a_id].path[n_index] = PathEntry(n->v->getId());

            // record the last time agent reach the goal from a non-goal vertex.
            if (agents[a_id].path_planner->goal_location == n->v->getId() && n_index - 1 >= 0 && agents[a_id].path_planner->goal_location != agents[a_id].path[n_index - 1].location)
                last_goal_visit = n_index;
        }
        // resize to last goal visit time
        agents[a_id].path.resize(last_goal_visit + 1);
        if (screen >= 2)
            std::cout << " Length: " << agents[a_id].path.size() << std::endl;
        if (screen >= 5)
        {
            cout << "Agent " << a_id << ":";
            for (auto loc : agents[a_id].path)
            {
                cout << loc.location << ",";
            }
            cout << endl;
        }
        path_table.insertPath(agents[a_id].id, agents[a_id].path);
        soc += (int)agents[a_id].path.size() - 1;
    }

    neighbor.sum_of_costs = soc;
}

void LNS::chooseDestroyHeuristicbyALNS()
{
    rouletteWheel();
    switch (selected_neighbor)
    {
    case 0:
        destroy_strategy = RANDOMWALK;
        break;
    case 1:
        destroy_strategy = INTERSECTION;
        break;
    case 2:
        destroy_strategy = RANDOMAGENTS;
        break;
    default:
        cerr << "ERROR" << endl;
        exit(-1);
    }
}

bool LNS::generateNeighborByIntersection()
{
    if (intersections.empty())
    {
        for (int i = 0; i < instance.map_size; i++)
        {
            if (!instance.isObstacle(i) && instance.getDegree(i) > 2)
                intersections.push_back(i);
        }
    }

    set<int> neighbors_set;
    auto pt = intersections.begin();
    std::advance(pt, rand() % intersections.size());
    int location = *pt;
    path_table.get_agents(neighbors_set, neighbor_size, location);
    if (neighbors_set.size() < neighbor_size)
    {
        set<int> closed;
        closed.insert(location);
        std::queue<int> open;
        open.push(location);
        while (!open.empty() && (int)neighbors_set.size() < neighbor_size)
        {
            int curr = open.front();
            open.pop();
            for (auto next : instance.getNeighbors(curr))
            {
                if (closed.count(next) > 0)
                    continue;
                open.push(next);
                closed.insert(next);
                if (instance.getDegree(next) >= 3)
                {
                    path_table.get_agents(neighbors_set, neighbor_size, next);
                    if ((int)neighbors_set.size() == neighbor_size)
                        break;
                }
            }
        }
    }
    neighbor.agents.assign(neighbors_set.begin(), neighbors_set.end());
    if (neighbor.agents.size() > neighbor_size)
    {
        std::random_shuffle(neighbor.agents.begin(), neighbor.agents.end());
        neighbor.agents.resize(neighbor_size);
    }
    if (screen >= 2)
        cout << "Generate " << neighbor.agents.size() << " neighbors by intersection " << location << endl;
    return true;
}
bool LNS::generateNeighborByRandomWalk()
{
    if (neighbor_size >= (int)agents.size())
    {
        neighbor.agents.resize(agents.size());
        for (int i = 0; i < (int)agents.size(); i++)
            neighbor.agents[i] = i;
        return true;
    }

    int a = findMostDelayedAgent();
    if (a < 0)
        return false;

    set<int> neighbors_set;
    neighbors_set.insert(a);
    randomWalk(a, agents[a].path[0].location, 0, neighbors_set, neighbor_size, (int)agents[a].path.size() - 1);
    int count = 0;
    while (neighbors_set.size() < neighbor_size && count < 10)
    {
        int t = rand() % agents[a].path.size();
        randomWalk(a, agents[a].path[t].location, t, neighbors_set, neighbor_size, (int)agents[a].path.size() - 1);
        count++;
        // select the next agent randomly
        int idx = rand() % neighbors_set.size();
        int i = 0;
        for (auto n : neighbors_set)
        {
            if (i == idx)
            {
                a = i;
                break;
            }
            i++;
        }
    }
    if (neighbors_set.size() < 2)
        return false;
    neighbor.agents.assign(neighbors_set.begin(), neighbors_set.end());
    if (screen >= 2)
        cout << "Generate " << neighbor.agents.size() << " neighbors by random walks of agent " << a
             << "(" << agents[a].path_planner->my_heuristic[agents[a].path_planner->start_location]
             << "->" << agents[a].path.size() - 1 << ")" << endl;

    return true;
}

int LNS::findMostDelayedAgent()
{
    int a = -1;
    int max_delays = -1;
    for (int i = 0; i < agents.size(); i++)
    {
        if (tabu_list.find(i) != tabu_list.end())
            continue;
        int delays = agents[i].getNumOfDelays();
        if (max_delays < delays)
        {
            a = i;
            max_delays = delays;
        }
    }
    if (max_delays == 0)
    {
        tabu_list.clear();
        return -1;
    }
    tabu_list.insert(a);
    if (tabu_list.size() == agents.size())
        tabu_list.clear();
    return a;
}

int LNS::findRandomAgent() const
{
    int a = 0;
    int pt = rand() % (sum_of_costs - sum_of_distances) + 1;
    int sum = 0;
    for (; a < (int)agents.size(); a++)
    {
        sum += agents[a].getNumOfDelays();
        if (sum >= pt)
            break;
    }
    assert(sum >= pt);
    return a;
}

// a random walk with path that is shorter than upperbound and has conflicting with neighbor_size agents
void LNS::randomWalk(int agent_id, int start_location, int start_timestep,
                     set<int> &conflicting_agents, int neighbor_size, int upperbound)
{
    int loc = start_location;
    for (int t = start_timestep; t < upperbound; t++)
    {
        auto next_locs = instance.getNeighbors(loc);
        next_locs.push_back(loc);
        while (!next_locs.empty())
        {
            int step = rand() % next_locs.size();
            auto it = next_locs.begin();
            advance(it, step);
            int next_h_val = agents[agent_id].path_planner->my_heuristic[*it];
            if (t + 1 + next_h_val < upperbound) // move to this location
            {
                path_table.getConflictingAgents(agent_id, conflicting_agents, loc, *it, t + 1);
                loc = *it;
                break;
            }
            next_locs.erase(it);
        }
        if (next_locs.empty() || conflicting_agents.size() >= neighbor_size)
            break;
    }
}

void LNS::validateSolution() const
{
    int sum = 0;
    for (const auto &a1_ : agents)
    {
        if (a1_.path.empty())
        {
            cerr << "No solution for agent " << a1_.id << endl;
            exit(-1);
        }
        else if (a1_.path_planner->start_location != a1_.path.front().location)
        {
            cerr << "The path of agent " << a1_.id << " starts from location " << a1_.path.front().location
                 << ", which is different from its start location " << a1_.path_planner->start_location << endl;
            exit(-1);
        }
        else if (a1_.path_planner->goal_location != a1_.path.back().location)
        {
            cerr << "The path of agent " << a1_.id << " ends at location " << a1_.path.back().location
                 << ", which is different from its goal location " << a1_.path_planner->goal_location << endl;
            exit(-1);
        }
        for (int t = 1; t < (int)a1_.path.size(); t++)
        {
            if (!instance.validMove(a1_.path[t - 1].location, a1_.path[t].location))
            {
                cerr << "The path of agent " << a1_.id << " jump from "
                     << a1_.path[t - 1].location << " to " << a1_.path[t].location
                     << " between timesteps " << t - 1 << " and " << t << endl;
                exit(-1);
            }
        }
        sum += (int)a1_.path.size() - 1;
        for (const auto &a2_ : agents)
        {
            if (a1_.id >= a2_.id || a2_.path.empty())
                continue;
            const auto &a1 = a1_.path.size() <= a2_.path.size() ? a1_ : a2_;
            const auto &a2 = a1_.path.size() <= a2_.path.size() ? a2_ : a1_;
            int t = 1;
            for (; t < (int)a1.path.size(); t++)
            {
                if (a1.path[t].location == a2.path[t].location) // vertex conflict
                {
                    cerr << "Find a vertex conflict between agents " << a1.id << " and " << a2.id << " at location " << a1.path[t].location << " at timestep " << t << endl;
                    exit(-1);
                }
                else if (a1.path[t].location == a2.path[t - 1].location &&
                         a1.path[t - 1].location == a2.path[t].location) // edge conflict
                {
                    cerr << "Find an edge conflict between agents " << a1.id << " and " << a2.id << " at edge (" << a1.path[t - 1].location << "," << a1.path[t].location << ") at timestep " << t << endl;
                    exit(-1);
                }
            }
            int target = a1.path.back().location;
            for (; t < (int)a2.path.size(); t++)
            {
                if (a2.path[t].location == target) // target conflict
                {
                    cerr << "Find a target conflict where agent " << a2.id << " (of length " << a2.path.size() - 1 << ") traverses agent " << a1.id << " (of length " << a1.path.size() - 1 << ")'s target location " << target << " at timestep " << t << endl;
                    exit(-1);
                }
            }
        }
    }
    if (sum_of_costs != sum)
    {
        cerr << "The computed sum of costs " << sum_of_costs << " is different from the sum of the paths in the solution " << sum << endl;
        exit(-1);
    }
}

void LNS::writeIterStatsToFile(const string &file_name) const
{
    if (init_lns != nullptr)
    {
        init_lns->writeIterStatsToFile(file_name + "-initLNS.csv");
    }
    if (iteration_stats.size() <= 1)
        return;
    string name = file_name;
    if (use_init_lns or num_of_iterations > 0)
        name += "-LNS.csv";
    else
        name += "-" + init_algo_name + ".csv";
    std::ofstream output;
    output.open(name);
    // header
    output << "num of agents,"
           << "sum of costs,"
           << "runtime,"
           << "cost lowerbound,"
           << "sum of distances,"
           << "MAPF algorithm" << endl;

    for (const auto &data : iteration_stats)
    {
        output << data.num_of_agents << "," << data.sum_of_costs << "," << data.runtime << "," << max(sum_of_costs_lowerbound, sum_of_distances) << "," << sum_of_distances << "," << data.algorithm << endl;
    }
    output.close();
}

void LNS::writeResultToFile(const string &file_name) const
{
    if (init_lns != nullptr)
    {
        init_lns->writeResultToFile(file_name + "-initLNS.csv", sum_of_distances, preprocessing_time);
    }
    string name = file_name;
    if (use_init_lns or num_of_iterations > 0)
        name += "-LNS.csv";
    else
        name += "-" + init_algo_name + ".csv";
    std::ifstream infile(name);
    bool exist = infile.good();
    infile.close();
    if (!exist)
    {
        ofstream addHeads(name);
        addHeads << "runtime,solution cost,initial solution cost,lower bound,sum of distance,"
                 << "iterations,"
                 << "group size,"
                 << "runtime of initial solution,restart times,area under curve,"
                 << "LL expanded nodes,LL generated,LL reopened,LL runs,"
                 << "preprocessing runtime,solver name,instance name" << endl;
        addHeads.close();
    }
    uint64_t num_LL_expanded = 0, num_LL_generated = 0, num_LL_reopened = 0, num_LL_runs = 0;
    for (auto &agent : agents)
    {
        agent.path_planner->reset();
        num_LL_expanded += agent.path_planner->accumulated_num_expanded;
        num_LL_generated += agent.path_planner->accumulated_num_generated;
        num_LL_reopened += agent.path_planner->accumulated_num_reopened;
        num_LL_runs += agent.path_planner->num_runs;
    }
    double auc = 0;
    if (!iteration_stats.empty())
    {
        auto prev = iteration_stats.begin();
        auto curr = prev;
        ++curr;
        while (curr != iteration_stats.end() && curr->runtime < time_limit)
        {
            auc += (prev->sum_of_costs - sum_of_distances) * (curr->runtime - prev->runtime);
            prev = curr;
            ++curr;
        }
        auc += (prev->sum_of_costs - sum_of_distances) * (time_limit - prev->runtime);
    }
    ofstream stats(name, std::ios::app);
    stats << runtime << "," << sum_of_costs << "," << initial_sum_of_costs << "," << max(sum_of_distances, sum_of_costs_lowerbound) << "," << sum_of_distances << "," << iteration_stats.size() << "," << average_group_size << "," << initial_solution_runtime << "," << restart_times << "," << auc << "," << num_LL_expanded << "," << num_LL_generated << "," << num_LL_reopened << "," << num_LL_runs << "," << preprocessing_time << "," << getSolverName() << "," << instance.getInstanceName() << endl;
    stats.close();
}

void LNS::writePathsToFile(const string &file_name) const
{
    std::ofstream output;
    output.open(file_name);
    // header
    // output << agents.size() << endl;

    for (const auto &agent : agents)
    {
        output << "Agent " << agent.id << ":";
        for (const auto &state : agent.path)
            output << "(" << instance.getRowCoordinate(state.location) << "," << instance.getColCoordinate(state.location) << ")->";
        output << endl;
    }
    output.close();
}
