#include "maze.h"

#ifndef ASTARH
#define ASTARH

void initialize_astar();
int agent_acupation(cell1 *auxcell);
int goal_acupation(cell1 *auxcell);
void Multi_print_grid();
void initialize_state(cell1 *tmpcell);
int compare_path(int a, int j, int myConflictStep, int otherConflictStep, int lookahead);
void determine_constraints(int a, int lookahead, int formula, cell1 *currentCell, int initialState);
void determine_role(int *roleij, int maxInfo, int a, int j, int *cell_role);
int compute_shortestpath_astar(int a, int lookahead);
int compute_constraintpath(int a, int lookahead);
void observe_new_agents(int a, int i, int lookahead);
void observe_agent2(int a, int i, int lookahead, cell1 *previous);
void observe_agent(int a, int i, int lookahead, cell1 *previous);
void updateHistory(int a, int i, int lookahead, cell1 *previous);
void evalPrevPrediction(int a, int i, int correct);
void updateProbabilities(int a, int i);
void computePrediction2(int a, int i, int lookahead);
void computePrediction(int a, int i, int lookahead);
void randommove(int a);
void test_rtaastar(int lookahead, int prunning);
void call_rtaastar();

#endif
