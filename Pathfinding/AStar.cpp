#include "AStar.h"
#include "Map.h"
#include "SDL.h"

//Joseph Ariganello

AStar::AStar(Map* m)
	:map(m),
	isSearching(false)
{
	graph = m->GetGraph();
}

AStar::~AStar()
{
}

bool AStar::IsSearching()
{
	return isSearching;
}

void AStar::Search(Node* start, Node* goal)
{
	isSearching = true;
	startNode = start;
	goalNode = goal;
	
	thread = SDL_CreateThread(SearchThread, "", this);
}

void AStar::OnSearchDone()
{
	isSearching = false;

	// Draw the shortest path
	for (auto p : pathFound)
	{
		map->SetPathMap(p->position, Map::RESULT_PATH_FOUND); // the second param value '2' means that it will draw
	}
	
}

int AStar::SearchThread(void * data)
{
	AStar* astar = static_cast<AStar*>(data);

	if (!astar->startNode || !astar->goalNode)
	{
		astar->OnSearchDone();
		return 0;
	}

	// To do: Complete this function.

	astar->distanceDict[astar->startNode] = 0;

	std::vector<Node> mapNodes = astar->map->GetGraph()->GetAllNodes();

	for (Node node : mapNodes){
		astar->distanceDict[astar->graph->GetNode(node.position)] = std::numeric_limits<float>::max();
		astar->actualDistanceDict[astar->graph->GetNode(node.position)] = std::numeric_limits<float>::max();
	}

	astar->distanceDict[astar->startNode] = 0;
	astar->actualDistanceDict[astar->startNode] = 0;

	astar->visited.clear();
	astar->unvisited.clear();

	for (Node node : mapNodes) {
		astar->unvisited.push_back(astar->graph->GetNode(node.position));
	}

	astar->predecessorDict.clear();

	while (astar->unvisited.size() > 0) {
		Node* u = astar->GetClosestFromUnvisited();

		if (u == astar->goalNode)
			break;

		astar->visited.push_back(u);
		astar->map->SetPathMap(u->position, Map::SEARCH_IN_PROGRESS);

		for (Node* v : astar->graph->GetAdjacentNodes(u)) {
			if (std::find(astar->visited.begin(), astar->visited.end(), v) != astar->visited.end())
				continue;

			if (astar->distanceDict[v] > astar->actualDistanceDict[u] + astar->graph->GetDistance(u, v) + astar->graph->GetDistance(v, astar->goalNode)) {
				astar->distanceDict[v] = astar->actualDistanceDict[u] + astar->graph->GetDistance(u, v) + astar->graph->GetDistance(v, astar->goalNode);

			}

			if (astar->actualDistanceDict[v] > astar->actualDistanceDict[u] + astar->graph->GetDistance(u, v)) {
				astar->actualDistanceDict[v] = astar->actualDistanceDict[u] + astar->graph->GetDistance(u, v);
				astar->predecessorDict[v] = u;
			}
		}
	}

	Node* p = astar->goalNode;

	while (p != astar->startNode) {
		astar->pathFound.push_back(p);
		p = astar->predecessorDict[p];
	}

	astar->pathFound.reserve(astar->pathFound.size());

	astar->OnSearchDone();
	return 0;
}

Node * AStar::GetClosestFromUnvisited()
{
	float shortest = std::numeric_limits<float>::max();
	Node* shortestNode = nullptr;

	// To do: Complete this function.

	for (Node* node : unvisited) {
		if (shortest > distanceDict[node]) {
			shortest = distanceDict[node];
			shortestNode = node;
		}
	}

	std::vector<Node*>::iterator iter = unvisited.begin();
	
	while (iter != unvisited.end()) {
		if (*iter == shortestNode)
			iter = unvisited.erase(iter);
		else
			iter++;
	}

	return shortestNode;
}

void AStar::ValidateDistanceDict(Node * n)
{
	float max = std::numeric_limits<float>::max();
	if (distanceDict.find(n) == distanceDict.end())
	{
		distanceDict[n] = max;
	}
	if (actualDistanceDict.find(n) == actualDistanceDict.end())
	{
		actualDistanceDict[n] = max;
	}
}
