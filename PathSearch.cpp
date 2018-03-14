#include "PathSearch.h"
#include <iostream>


namespace fullsail_ai { namespace algorithms {
	
	class PathSearch;

	PathSearch::SearchNode::SearchNode(Tile* element)
	{
		currentTile = element;
	}
	PathSearch::Edge::Edge(SearchNode* n_node, float cost)
	{
		End_point = n_node;
		Coast = cost;
	}
	PathSearch::PlannerNode::PlannerNode(SearchNode* m_node)
	{
		parnet = nullptr;
		vertex = m_node;
	}
	bool is_Greater(PathSearch::PlannerNode*const&lhs, PathSearch::PlannerNode*const&rhs)
	{
		return (lhs->f_cost > rhs->f_cost);
	}

	PathSearch::PathSearch() : new_queue(is_Greater)
	{
		goal = nullptr;
		found = nullptr;
		tilemap = nullptr;
	}

	PathSearch::~PathSearch()
	{
	}
	// getting the distance between 2points
	float PathSearch::distancefrom(double startx, double starty, double endx, double endy)
	{
		double updated_x = endx - startx;
		double updated_y = endy - starty;

		updated_x *= updated_x;
		updated_y *= updated_y;

		return sqrt(updated_x + updated_y);

	}


	void PathSearch::initialize(TileMap* _tileMap)
	{
		tilemap = _tileMap;
		Node_Map.resize(tilemap->getRowCount());
		for (int i = 0; i < tilemap->getRowCount(); i++)
		{
			Node_Map[i].resize(tilemap->getColumnCount());
		}
		for (int j = 0; j < tilemap->getRowCount(); j++)
		{
			for (int k = 0; k < tilemap->getColumnCount(); k++)
			{
				if (tilemap->getTile(j,k)!=NULL && tilemap->getTile(j,k)->getWeight() !=0)
				{
					Node_Map[j][k] = new SearchNode(tilemap->getTile(j,k));
				}
			}
		}

		for (int  n_row = 0; n_row <Node_Map.size(); n_row++)
		{
			for (int n_col = 0; n_col < Node_Map[n_row].size(); n_col++)
			{
				if (Node_Map[n_row][n_col] == nullptr)
				{
					continue;
				}
				//fill the edges for the tiles 
				for (int tile_row = Node_Map[n_row][n_col]->currentTile->getRow() - 1; tile_row <= Node_Map[n_row][n_col]->currentTile->getRow()+1; tile_row++)
				{
					for (int tile_col = Node_Map[n_row][n_col]->currentTile->getColumn() - 1; tile_col <= Node_Map[n_row][n_col]->currentTile->getColumn() + 1; tile_col++)
					{
						if (tilemap->getTile(tile_row,tile_col) != NULL && tilemap->getTile(tile_row, tile_col)->getWeight()!=0)
						{
							if (areAdjacent(Node_Map[n_row][n_col]->currentTile, tilemap->getTile(tile_row, tile_col)))
							{
								Tile* temp = tilemap->getTile(tile_row, tile_col);
								float dist = distancefrom(Node_Map[n_row][n_col]->currentTile->getXCoordinate(), Node_Map[n_row][n_col]->currentTile->getYCoordinate(), temp->getXCoordinate(), temp->getYCoordinate());
								float cost = dist*temp->getWeight();
								Edge* n_edge = new Edge(new SearchNode(temp), cost);
								Node_Map[n_row][n_col]->Edges_onTile.push_back(n_edge);
							}
						}
					}
				}
			}
		}
	}

	void PathSearch::enter(int startRow, int startColumn, int goalRow, int goalColumn)
	{
		new_queue.push(new PlannerNode(Node_Map[startRow][startColumn]));
		goal = new SearchNode(tilemap->getTile(goalRow, goalColumn));
		goal->Edges_onTile = Node_Map[goalRow][goalColumn]->Edges_onTile;
		visited_Node[Node_Map[startRow][startColumn]]= new_queue.front();
		visited_Node[Node_Map[startRow][startColumn]]->h_cost = distancefrom(new_queue.front()->vertex->currentTile->getXCoordinate(), new_queue.front()->vertex->currentTile->getYCoordinate(), 
			                                                                  goal->currentTile->getXCoordinate(), goal->currentTile->getYCoordinate());
		visited_Node[Node_Map[startRow][startColumn]]->g_cost = 0;
		visited_Node[Node_Map[startRow][startColumn]]->f_cost = visited_Node[Node_Map[startRow][startColumn]]->g_cost + visited_Node[Node_Map[startRow][startColumn]]->h_cost * H_weight;
		time = 0;
		found_solution = false;
	}

	void PathSearch::update(long timeslice)
	{
		DWORD s_time = GetTickCount();
		time = 0;
		while (!new_queue.empty())
		{
			if (time > timeslice)
			{
				return;
			}
			PlannerNode* current_node = new_queue.front();
			new_queue.pop();
			if (current_node->vertex->currentTile == goal->currentTile)
			{
				found_solution = true;
				found = current_node;
				return;
			}
			for (int i = 0; i < current_node->vertex->Edges_onTile.size(); i++)
			{
				SearchNode* S_node = Node_Map[current_node->vertex->Edges_onTile[i]->End_point->currentTile->getRow()][current_node->vertex->Edges_onTile[i]->End_point->currentTile->getColumn()];
				float give_cost = current_node->g_cost+ current_node->vertex->Edges_onTile[i]->Coast;
				if (visited_Node[S_node]!=NULL)
				{
					PlannerNode * S_planer = visited_Node[S_node];
					if (give_cost < S_planer->g_cost)
					{
						//S_node->currentTile->setFill(0xff0000ff);
						new_queue.remove(S_planer);
						S_planer->g_cost = give_cost;
						S_planer->f_cost = S_planer->g_cost + S_planer->h_cost*H_weight;
						S_planer->parnet = current_node;
						new_queue.push(S_planer);
					}

				}
				else
				{
					//S_node->currentTile->setFill(0xff0000ff);
					PlannerNode * N_planer = new PlannerNode(S_node);
					N_planer->g_cost = give_cost;
					N_planer->h_cost = distancefrom(S_node->currentTile->getXCoordinate(), S_node->currentTile->getYCoordinate(), goal->currentTile->getXCoordinate(), goal->currentTile->getYCoordinate());
					N_planer->f_cost = N_planer->g_cost + N_planer->h_cost*H_weight;
					N_planer->parnet = current_node;
					visited_Node[S_node] = N_planer;
					new_queue.push(N_planer);
				}
			}
			time = GetTickCount()-s_time;
			if (timeslice == 0)
			{
				return;
			}
		}
	}

	void PathSearch::exit()
	{
		// fast delete for visited node nodes that were created
		for (auto iter = visited_Node.begin(); iter != visited_Node.end(); iter++)
		{
			delete iter->second;
		}
		visited_Node.clear();
		new_queue.clear();
		if (goal!= nullptr)
		{
			delete goal;
			goal = nullptr;
		}
		time = 0;

	}

	void PathSearch::shutdown()
	{
		for (int Row = 0; Row < Node_Map.size(); Row++)
		{
			for (int col = 0; col < Node_Map[Row].size(); col++)
			{
				if (Node_Map[Row][col] == nullptr)
				{
					continue;
				}
				for (int edge = 0; edge < Node_Map[Row][col]->Edges_onTile.size(); edge++)
				{
					delete Node_Map[Row][col]->Edges_onTile[edge]->End_point;
					delete Node_Map[Row][col]->Edges_onTile[edge];
				}
				Node_Map[Row][col]->Edges_onTile.clear();
				delete Node_Map[Row][col];
			}
			Node_Map[Row].clear();
		}
		Node_Map.clear();
		exit();
		found_solution = false;

	}
	
	bool PathSearch::isDone() const
	{
		if (found_solution)
		{
			return true;
		}
		return false;

	}

	std::vector<Tile const*> const PathSearch::getSolution() const
	{
		std::vector<Tile const*> temp;
		PlannerNode* iter = found;
		while (iter!=nullptr)
		{
			Tile* t_tile = iter->vertex->currentTile;
			temp.push_back(t_tile);
			iter = iter->parnet;

		}
		return temp;
	}

	bool PathSearch::areAdjacent(Tile const* lhs, Tile const* rhs)
	{
		bool type_return = false;
		//odd
		if (lhs->getRow() %2 !=0)
		{
			if (rhs->getRow() == lhs->getRow()+1 || rhs->getRow() == lhs->getRow()-1)
			{
				if (rhs->getColumn()== lhs->getColumn()+1 || rhs->getColumn() == lhs->getColumn())
				{
					type_return = true;
				}

			}
			else if (rhs->getRow() == lhs->getRow())
			{
				if (rhs->getColumn() == lhs->getColumn() + 1 || rhs->getColumn() == lhs->getColumn()-1)
				{
					type_return = true;
				}
			}

		}
		//even
		else
		{
			if (rhs->getRow() == lhs->getRow() + 1 || rhs->getRow() == lhs->getRow() - 1)
			{
				if (rhs->getColumn() == lhs->getColumn() - 1 || rhs->getColumn() == lhs->getColumn())
				{
					type_return = true;
				}

			}
			else if (rhs->getRow() == lhs->getRow())
			{
				if (rhs->getColumn() == lhs->getColumn() + 1 || rhs->getColumn() == lhs->getColumn() - 1)
				{
					type_return = true;
				}
			}
		}
		return type_return;
	}
}}  // namespace fullsail_ai::algorithms

