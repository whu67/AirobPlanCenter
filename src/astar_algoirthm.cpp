/*
 * astar_algoirthm.cpp
 *
 *  Created on: May 18, 2016
 *      Author: root
 */

#include "astar_algorithm.h"

Node* open_list;
Node* close_list;
//int blocktag;
//int XxStart;
//int YyStart;
int XxEnd;
int YyEnd;

void init_openlist()
{
	open_list = NULL;
}

void init_closelist()
{
	close_list = NULL;
}
void destroy_openlist()
{
	Node* q;
	Node* p = open_list;
	while (p != NULL)
	{
		q = p->next;
		free(p);
		p = q;
	}
}
void destroy_closelist()
{
	Node* q;
	Node* p = close_list;
	while (p != NULL)
	{
		q = p->next;
		free(p);
		p = q;
	}
}
void insert_into_openlist(Node* new_node) //insert and sort by F
{
	Node* p;
	Node* q;
	if (open_list == NULL)
	{
		open_list = new_node; //insert as the first
		return;
	}
	p = open_list;
	while (p != NULL)
	{
		q = p;
		p = p->next;
		if (p == NULL)
		{
			q->next = new_node; //insert as the last
			return;
		}
		else if (GET_F(new_node) < GET_F(p))
		{
			q->next = new_node; //insert before p, sorted
			new_node->next = p;
			return;
		}
	}

}

void insert_into_closelist(Node* new_node) //just insert before head
{
	if (close_list == NULL)
	{
		close_list = new_node; //insert as the first
		return;
	}
	else
	{
		new_node->next = close_list; //insert before head
		close_list = new_node;
		return;
	}
}

Node* find_node_in_list_by_ij(Node* node_list, int di, int dj)
{
	Node* p = node_list;
	while (p)
	{
		if (p->i == di && p->j == dj)
			return p;
		p = p->next;
	}
	return NULL;
}

Node* pop_firstnode_from_openlist() //get the minimum node sorted by F
{
	Node* p = open_list;
	if (p == NULL)
	{
		return NULL;
	}
	else
	{
		open_list = p->next;
		p->next = NULL;
		return p;
	}
}

void remove_node_from_openlist(Node* nd) //just remove it, do not destroy it
{
	Node* q;
	Node* p = open_list;
	if (open_list == nd)
	{
		open_list = open_list->next;
		return;
	}
	while (p)
	{
		q = p;
		p = p->next;
		if (p == nd) //found
		{
			q->next = p->next;
			p->next = NULL;
			return;
		}
	}
}

void remove_node_from_closelist(Node* nd) //just remove it, do not destroy it
{
	Node* q;
	Node* p = close_list;
	if (close_list == nd)
	{
		close_list = close_list->next;
		return;
	}
	while (p)
	{
		q = p;
		p = p->next;
		if (p == nd) //found
		{
			q->next = p->next;
			p->next = NULL;
			return;
		}
	}
}
//===================================================================
//=======================calculate H, G =============================
//calculate Heuristic value
//(reimplemented when porting a star to another application)
double calc_H(int cur_i, int cur_j, int end_i, int end_j)
{
	return (abs(end_j - cur_j) + abs(end_i - cur_i)) * 10.0; //the heuristic
}
//calculate G value
//(reimplemented when porting a star to another application)
double calc_G(Node* cur_node)
{
	Node* p = cur_node->parent;
	if (abs(p->i - cur_node->i) + abs(p->j - cur_node->j) > 1)
		return 14.0 + p->G; //the diagonal cost is 14
	else
		return 10.0 + p->G; //the adjacent cost is 10
}

void init_start_node(Node* st, int si, int sj, int ei, int ej)
{
	memset(st, 0, sizeof(Node));
	st->type = START;
	st->i = si;
	st->j = sj;
	st->H = calc_H(si, sj, ei, ej);
	st->G = 0;
}

void init_end_node(Node* ed, int ei, int ej)
{
	memset(ed, 0, sizeof(Node));
	ed->type = END;
	ed->i = ei;
	ed->j = ej;
	ed->H = 0;
	ed->G = 9999; //temp value
}

void init_pass_node(Node* pd, int pi, int pj)
{
	memset(pd, 0, sizeof(Node));
	pd->type = AVAIL;
	pd->i = pi;
	pd->j = pj;
}

//check the candidate node (i,j) when extending parent_node
int check_neighbor(int map[][N], int width, int height,
	int di, int dj, Node* parent_node, Node* end_node)
{
	Node* p;
	Node* temp;
	double new_G;
	if (di < 0 || dj < 0 || di > height-1 || dj > width-1)
		return UNAVAIL;
	//1. check available
	if (map[di][dj] == UNAVAIL)
		return UNAVAIL;
	//2. check if existed in close list
	p = find_node_in_list_by_ij(close_list, di, dj);
	if (p != NULL)
	{
		//found in the closed list, check if the new G is better, added 2012-05-09
		temp = p->parent;
		p->parent = parent_node;
		new_G = calc_G(p);
		if (new_G >= p->G)
		{
			p->parent = temp; //if new_G is worse, recover the parent
		}
		else
		{
			//the new_G is better, remove it from close list, insert it into open list
			p->G = new_G;
			remove_node_from_closelist(p); //remove it
			insert_into_openlist(p); //insert it, sorted
		}
		return AVAIL;
	}
	//3. check if existed in open list
	p = find_node_in_list_by_ij(open_list, di, dj); //in open list
	if (p != NULL)
	{
		//found in the open list, check if the new G is better
		temp = p->parent;
		p->parent = parent_node;
		new_G = calc_G(p);
		if (new_G >= p->G)
		{
			p->parent = temp; //if new_G is worse, recover the parent
		}
		else
		{
			//the new_G is better, resort the list
			p->G = new_G;
			remove_node_from_openlist(p); //remove it
			insert_into_openlist(p); //insert it, sorted
		}
		return AVAIL;
	}

	//4. none of above, insert a new node into open list
	if ((di == XxEnd)&&(dj == YyEnd))//map[di][dj] == END)
	{
		//4~. check if it is end node
		end_node->parent = parent_node;
		end_node->G = calc_G(end_node);
		insert_into_openlist(end_node); //insert into openlist
		return AVAIL;
	}
	else
	{
		//4~~. create a new node
		p = (Node *)malloc(sizeof(Node));
		init_pass_node(p, di, dj);
		p->parent = parent_node;
		p->H = calc_H(di, dj, end_node->i, end_node->j);
		p->G = calc_G(p);
		insert_into_openlist(p); //insert into openlist
		return AVAIL;
	}
}
//extend the current node on the map
//(reimplemented when porting a star to another application)
void extend_node(Node* cd, int map[][N], int width, int height, Node* end_node)
{
	int up_status, down_status, left_status, right_status;
	int ci, cj; //cur node i, j
	int ti, tj; //temp i, j
	ci = cd->i;
	cj = cd->j;
	//1. up
	ti = ci - 1;
	tj = cj;
	up_status = check_neighbor(map, width, height, ti, tj, cd, end_node);
	//2. down
	ti = ci + 1;
	tj = cj;
	down_status = check_neighbor(map, width, height, ti, tj, cd, end_node);
	//3. left
	ti = ci;
	tj = cj - 1;
	left_status = check_neighbor(map, width, height, ti, tj, cd, end_node);
	//4. right
	ti = ci;
	tj = cj + 1;
	right_status = check_neighbor(map, width, height, ti, tj, cd, end_node);
	//5. leftup
	ti = ci - 1;
	tj = cj - 1;
	if (up_status == AVAIL && left_status == AVAIL)
		check_neighbor(map, width, height, ti, tj, cd, end_node);
	//6. rightup
	ti = ci - 1;
	tj = cj + 1;
	if (up_status == AVAIL && right_status == AVAIL)
		check_neighbor(map, width, height, ti, tj, cd, end_node);
	//7. leftdown
	ti = ci + 1;
	tj = cj - 1;
	if (down_status == AVAIL && left_status == AVAIL)
		check_neighbor(map, width, height, ti, tj, cd, end_node);
	//8. rightdown
	ti = ci + 1;
	tj = cj + 1;
	if (down_status == AVAIL && right_status == AVAIL)
		check_neighbor(map, width, height, ti, tj, cd, end_node);

}

//=======================search algorithm======================================
Node* a_star_search(int map[M][N], int width, int height, int start_i, int start_j, int end_i, int end_j)
{
	Node* cur_node;
	Node* start_node;
	Node* end_node;
	//create start and end node
	start_node = (Node *)malloc(sizeof(Node));
	init_start_node(start_node, start_i, start_j, end_i, end_j);
	end_node = (Node *)malloc(sizeof(Node));
	init_end_node(end_node, end_i, end_j);

	//init open and close list
	init_openlist();
	init_closelist();
	//put start_node into open list
	insert_into_openlist(start_node);

	//start searching
	while (1)
	{
		cur_node = pop_firstnode_from_openlist(); //it has the minimum F value
		if (cur_node == NULL || cur_node->type == END)
		{
			break; //found the road or no road found
		}

		extend_node(cur_node, map, width, height, end_node); //the key step!!
		insert_into_closelist(cur_node);
	}
	//you can track the road by the node->parent
	return cur_node;
}

Node* Road_Plan(int map[][N], int starti, int startj, int endi, int endj)
{
	XxEnd = endi;
	YyEnd = endj;
	Node* node_list = a_star_search(map, N, M, starti, startj, endi, endj);
	if(NULL == node_list)
	{
		fprintf(stdout,"No road found! \n");
	}
	else
	{
		fprintf(stdout,"The cost of this road is %f\n", node_list->G);
	}
	return node_list;
}

