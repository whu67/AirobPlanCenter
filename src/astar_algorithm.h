/*
 * file: astar_algorithm.h
 * author: MulinB@HUST
 * date: 2010-10-10
 * modified: 2012-05-09
 * A-star algorithm implemented in C. Only for study.
 */
#ifndef _ASTAR_ALGORITHM_H
#define _ASTAR_ALGORITHM_H
#include <math.h>
#include <stdlib.h>
#include <malloc.h>
#include <string.h>
#define M  160
#define N  160
//map marks
#define  AVAIL     0
#define  UNAVAIL   1
#define  START     7
#define  END       9
#define  ROAD      8
#define	 GET_F(X)  (X->G + X->H)
typedef struct Node
{
	//for node itself
	int type; //node type
	int i; //i index
	int j; //j index
	//for A star algorithm
	double G; //past road cost
	double H; //heuristic, F = G + H
	struct Node* parent; //parent node, used for trace road
	struct Node* next; //only used for open and close list
}Node;

void destroy_openlist();
void destroy_closelist();
Node* Road_Plan(int map[][N], int starti, int startj, int endi, int endj);
//==========================open close list operation================
#endif /* file end */
