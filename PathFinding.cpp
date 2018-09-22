//Group 67
//Written by Amaar Quadri

#include <iostream>
#include <random>
#include <vector>
#include <cmath>
#include <cfloat>
#include <algorithm>
#include <stack>
#include <fstream>
#include <ctime>

using namespace std;

//AutoHotkey file that will paste the required text into Desmos online graphing calculator
//when later comments say that something is being written to Desmos,
//they mean that the AutoHotkey code required to paste that thing into Desmos is being written to this file
ofstream fout("C:\\Users\\amaar\\Desktop\\Desmos.ahk");

const float POSITIVE_INFINITY = FLT_MAX;
//the tolerance for considering 2 floats to be equal
const float TOL = 0.001;
//half of the largest linear dimension of the rover (used to extend the polygons to account for space for the rover)
const float SLOP = 5;
//the minimum distance between any 2 obstacles (used in the end condition for the clustering algorithm)
const float MIN_OBSTACLE_PROXIMITY = 15;

struct Point {
    float x;
    float y;
};

struct Node {
    Point* p;
    vector<Node*>* visibleNodes;
    float pathLength;
    float distanceToEnd;
    float combinedHeuristic;
    Node* pathVia;
};

struct LineSegment {
    Point* start;
    Point* end;
};

//returns whether or not the two given floats are equal to each other to within TOL
bool equals(const float float1, const float float2) {
    return abs(float1 - float2) <= TOL;
}

//returns the euclidean distance between the two given Points
float dist(const Point* point1, const Point* point2) {
    return hypot(point1->x - point2->x, point1->y - point2->y);
}

//returns whether or not the two given Points equal each other to within a distance of TOL
bool equals(const Point* point1, const Point* point2) {
    return dist(point1, point2) < TOL;
}

//returns 1 if the given Points represent a left turn
//returns -1 if the given Points represent a right turn
//returns 0 if the given Points are collinear (to within  a tolerance based on TOL)
//NOTE: THIS CODE WAS DERIVED FROM ONLINE SOURCES
int ccw(const Point* point1, const Point* point2, const Point* point3) {
    float a = (point2->x - point1->x) * (point3->y - point1->y) - (point2->y - point1->y) * (point3->x - point1->x);
    if (equals(a, 0)) return 0;
    return a > 0 ? 1 : -1;
}

//swaps the two given pointers to Points
void swap(Point* point1, Point* point2) {
    Point temp = *point1;
    *point1 = *point2;
    *point2 = temp;
}

//writes the given LineSegment to Desmos
void toDesmos(LineSegment* lineSegment) {
    if (equals(lineSegment->start->x, lineSegment->end->x)) {
        float minY = min(lineSegment->start->y, lineSegment->end->y);
        float maxY = max(lineSegment->start->y, lineSegment->end->y);
        fout << "Send, x=" << lineSegment->start->x << "{{}" << minY << "<y<" << maxY << "{}}{Enter}" << endl;
        return;
    }
    float m = (lineSegment->end->y - lineSegment->start->y) / (lineSegment->end->x - lineSegment->start->x);
    float b = lineSegment->start->y - m * lineSegment->start->x;
    float minX = min(lineSegment->start->x, lineSegment->end->x);
    float maxX = max(lineSegment->start->x, lineSegment->end->x);
    fout << "Send, y=" << m << "x{+}" << b << "{{}" << minX << "<x<" << maxX << "{}}{Enter}" << endl;
}

//prints the LineSegment connecting the two given nodes to Desmos
void toDesmos(Node* n1, Node* n2) {
    toDesmos(new LineSegment{n1->p, n2->p});
}

//NOTE THAT THIS ALGORITHM WAS DERIVED FROM ONLINE SOURCES
//returns whether or not the two given LineSegments intersect
bool cross(LineSegment *lineSegment1, LineSegment *lineSegment2) {
    //return false for LineSegments that share an edge because
    //we don't want to discount a potential connection because it intersects the edge of the polygon that it originates from
    if (equals(lineSegment1->start, lineSegment2->start) || equals(lineSegment1->start, lineSegment2->end) ||
            equals(lineSegment1->end, lineSegment2->start) || equals(lineSegment1->end, lineSegment2->end))
        return false;

    if (ccw(lineSegment1->start, lineSegment1->end, lineSegment2->start) ==
        ccw(lineSegment1->start, lineSegment1->end, lineSegment2->end))
        return false;

    return ccw(lineSegment2->start, lineSegment2->end, lineSegment1->start) !=
           ccw(lineSegment2->start, lineSegment2->end, lineSegment1->end);
}

//prints the vector of Points to Desmos
void printDesmos(vector<Point*> *polygon) {
    Point* p = polygon->at(0);
    fout << "Send, (" << p->x << ", " << p->y << ")";
    for (unsigned int i = 1; i < polygon->size(); i++) {
        p = polygon->at(i);
        fout << ", (" << p->x << ", " << p->y << ")";
    }
    fout << "{Enter}" << endl;
}

//NOTE THAT ALTHOUGH THIS CLUSTERING ALGORITHM WAS PRODUCED INDEPENDANTLY, IT WAS INSPIRED BY HIGHER LEVEL CONCEPTS LEARNED ONLINE
//clusters the vector of Points into groups that are geometrically close together
vector<vector<Point*>*>* clusterify(const vector<Point*>* obstaclePoints) {
    //create a list of clusters, and fill it with clusters (each obstacle Point is in a cluster by itself initially)
    auto* clusters = new vector<vector<Point*>*>;
    for (Point* p : *obstaclePoints) {
        auto* cluster = new vector<Point*>;
        cluster->push_back(p);
        clusters->push_back(cluster);
    }

    //loop until finished combining clusters
    while (true) {
        //get the minimum distance between all possible pairs of clusters, and the indices of that pair
        float overallMinimumDistance = POSITIVE_INFINITY;
        unsigned int minI, minJ;
        for (unsigned int i = 0; i < clusters->size(); i++) for (unsigned int j = i + 1; j < clusters->size(); j++) {
            //get the minimum distance between any pair of Points from the 2 clusters
            float minimumDistance = POSITIVE_INFINITY;
            for (Point* p : *clusters->at(i)) for (Point* q : *clusters->at(j)) {
                float distance = dist(p, q);
                if (distance < minimumDistance) minimumDistance = distance;
            }

            //if these two clusters are closer together than any previously found pair of clusters
            if (minimumDistance < overallMinimumDistance) {
                //set them as the minimum
                overallMinimumDistance = minimumDistance;
                minI = i;
                minJ = j;
            }
        }
        //if the minimum distance is greater than the specified minimum distance between any 2 obstacles then we are done
        if (overallMinimumDistance > MIN_OBSTACLE_PROXIMITY) return clusters;

        //otherwise, combine the two closest clusters
        vector<Point*>* firstCluster = clusters->at(minI);
        for (Point* p : *clusters->at(minJ)) firstCluster->push_back(p);
        clusters->erase(clusters->begin() + minJ);
    }
}

//NOTE THAT THIS FUNCTION USES JARVIS' ALGORITHM AND WAS TAKEN FROM ONLINE SOURCES
//returns the convex hull of the given vector of Points
vector<Point*>* getConvexHull(vector<Point*> *cluster) {
    auto* convexHull = new vector<Point*>;

    Point* left = cluster->at(0);
    unsigned int l = 0;
    for (unsigned int i = 1; i < cluster->size(); i++) {
        Point* p = cluster->at(i);
        if (p->x < left->x) {
            left = p;
            l = i;
        }
    }

    unsigned int p = l, q;
    do {
        convexHull->push_back(cluster->at(p));
        q = (p + 1) % cluster->size();
        for (unsigned int i = 0; i < cluster->size(); i++)
            if (ccw(cluster->at(p), cluster->at(i), cluster->at(q)) == 1) q = i;
        p = q;
    } while (p != l);
    return convexHull;
}

//creates a visibility graph amongst the polygons and the start and end Points, and returns the starting node
Node* createVisibilityGraph(Point* startPoint, Point* endPoint, vector<vector<Point*>*>* polygons) {
    //create the start and end nodes
    float distance = dist(startPoint, endPoint);
    auto* startNode = new Node{startPoint, new vector<Node*>, 0, distance, distance, nullptr};
    auto* endNode = new Node{endPoint, new vector<Node*>, POSITIVE_INFINITY, 0, POSITIVE_INFINITY, nullptr};

    //convert the polygons of Points to polygons of nodes
    auto* nodePolygons = new vector<vector<Node*>*>;
    for (vector<Point*>* polygon : *polygons) {
        auto* nodePolygon = new vector<Node*>;
        for (Point* p : *polygon) nodePolygon->push_back(
                    new Node{p, new vector<Node*>, POSITIVE_INFINITY, dist(p, endPoint), POSITIVE_INFINITY, nullptr});
        nodePolygons->push_back(nodePolygon);
    }

    fout << "Send, Borders{Enter}" << endl;

    //connect adjacent vertices of polygons, and accumulate those LineSegments
    auto* borders = new vector<LineSegment*>;
    for (vector<Node*>* nodePolygon : *nodePolygons) {
        //join the first and last vertices of the polygon
        Node* firstNode = nodePolygon->at(0);
        Node* lastNode = nodePolygon->at(nodePolygon->size() - 1);
        firstNode->visibleNodes->push_back(lastNode);
        lastNode->visibleNodes->push_back(firstNode);
        toDesmos(firstNode, lastNode);
        borders->push_back(new LineSegment{firstNode->p, lastNode->p});

        //join the rest of the polygon's vertices
        for (unsigned int i = 1; i < nodePolygon->size(); i++) {
            //join the i-1 and ith vertices of the polygon
            Node* node1 = nodePolygon->at(i - 1);
            Node* node2 = nodePolygon->at(i);
            node1->visibleNodes->push_back(node2);
            node2->visibleNodes->push_back(node1);
            toDesmos(node1, node2);
            borders->push_back(new LineSegment{node1->p, node2->p});
        }
    }

    //check if a direct route from the start to end Point is possible
    auto* directRoute = new LineSegment{startPoint, endPoint};
    bool directRoutePossible = true;
    for (LineSegment* border : *borders) if (cross(directRoute, border)) {
        directRoutePossible = false;
        break;
    }
    if (directRoutePossible) {
        //connect the start and end nodes
        startNode->visibleNodes->push_back(endNode);
        endNode->visibleNodes->push_back(startNode);

        //return a visiblity graph consisting of just the start and end nodes so that A Star will finish quickly
        return startNode;
    }

    fout << "Send, {Backspace}{Backspace}{Down}" << endl;
    fout << "Send, Connections Between Extended Polygons{Enter}" << endl;

    //join nodes between the polygons that are visible to each other
    for (unsigned int i = 0; i < nodePolygons->size(); i++) for (unsigned int j = i + 1; j < nodePolygons->size(); j++) {
        vector<Node*>* nodePolygon1 = nodePolygons->at(i);
        vector<Node*>* nodePolygon2 = nodePolygons->at(j);
        //for every pair of Points between the polygons
        for (auto* node1 : *nodePolygon1) for (auto* node2 : *nodePolygon2) {
            auto* lineSegment = new LineSegment{node1->p, node2->p};

            //check if the LineSegment between these two nodes crosses any of the borders of the polygons
            bool isVisible = true;
            for (LineSegment* border : *borders) if (cross(lineSegment, border)) {
                isVisible = false;
                break;
            }

            if (isVisible) {
                //connect the two nodes
                node1->visibleNodes->push_back(node2);
                node2->visibleNodes->push_back(node1);
                toDesmos(node1, node2);
            }
        }
    }

    fout << "Send, {Backspace}{Backspace}{Down}" << endl;
    fout << "Send, Connections From Start and End{Enter}" << endl;

    //join the start and end nodes to other visible nodes
    for (vector<Node*>* nodePolygon : *nodePolygons) for (Node* node : *nodePolygon) {
        auto* startSegment = new LineSegment{startPoint, node->p};
        auto* endSegment = new LineSegment{endPoint, node->p};

        //check if either the start node or the end node are visible from the given node
        bool isStartVisible = true, isEndVisible = true;
        for (LineSegment* border : *borders) {
            if (isStartVisible) {
                if (cross(startSegment, border)) isStartVisible = false;
            }
            if (isEndVisible) {
                if (cross(endSegment, border)) isEndVisible = false;
            }
            //if neither the start nor end node are visible from the given node, then we can skip to the next node
            if (!isStartVisible && !isEndVisible) break;
        }

        if (isStartVisible) {
            //join the start node to the visible node
            startNode->visibleNodes->push_back(node);
            node->visibleNodes->push_back(startNode);
            toDesmos(startNode, node);
        }
        if (isEndVisible) {
            //join the end node to the visible node
            endNode->visibleNodes->push_back(node);
            node->visibleNodes->push_back(endNode);
            toDesmos(endNode, node);
        }
    }
    fout << "Send, {Backspace}{Backspace}{Down}" << endl;
    return startNode;
}

//NOTE THAT THIS FUNCTION USES THE WELL KNOWN A STAR ALGORITHM AND WAS BASED OFF OF ONLINE SOURCES
//runs the A Star algorithm on the visibility graph whose start node is given
vector<Point*>* aStar(Node* startNode, Point* endPoint) {
    //create a priority queue and place the starting node in it
    auto* priorityQueue = new vector<Node*>;
    priorityQueue->push_back(startNode);

    //continue to process the node at the top of the priority queue
    while (true) {
        Node* currentNode = priorityQueue->at(0);

        //if the endNode has reached the top of the priority queue
        if (currentNode->p == endPoint) {
            //create the path
            auto* path = new vector<Point*>;

            //follow the chain of nodes backwards, starting at endNode, through the pathVia pointers
            Node* nextNode = currentNode;
            while (nextNode != nullptr) {
                //insert the node's Point to the beginning of the path list (because we are iterating backwards)
                path->insert(path->begin(), nextNode->p);
                nextNode = nextNode->pathVia;
            }
            return path;
        }

        //for each of the neighbours of this node
        for (Node* neighbour : *currentNode->visibleNodes) {
            //get the path length to the neighbour node through the current node
            float pathLength = currentNode->pathLength + dist(currentNode->p, neighbour->p);

            //if this is less that the the neighbour's current path length
            if (pathLength < neighbour->pathLength) {
                neighbour->pathLength = pathLength;
                neighbour->combinedHeuristic = pathLength + neighbour->distanceToEnd;
                neighbour->pathVia = currentNode;

                //place neighbour in the priority queue
                for (unsigned int j = priorityQueue->size() - 1; j >= 0; j--) {
                    Node* node = priorityQueue->at(j);

                    //if the neighbour node is already in the queue, remove it (it will be placed at a higher priority)
                    if (node == neighbour) {
                        priorityQueue->erase(priorityQueue->begin() + j);
                        continue;
                    }

                    //once you reach a node that has a better (i.e. lower) combined heuristic than neighbour (or equal)
                    if (node->combinedHeuristic <= neighbour->combinedHeuristic) {
                        //insert neighbour after the node
                        priorityQueue->insert(priorityQueue->begin() + j + 1, neighbour);
                        break;
                    }
                }
            }
        }

        //remove the current node from the priority queue
        priorityQueue->erase(priorityQueue->begin());
    }
}

//runs the path-finding algorithm, returning a vector of Points representing the resulting path
vector<Point*>* getPath(Point* startPoint, Point* endPoint, vector<Point*>* obstaclePoints) {
    //add a 5 second wait so that the user has time to switch to Desmos
    fout << "Sleep, 5000" << endl;

    //write the start and end Points to Desmos
    fout << "Send, End Points{Enter}" << endl << "Sleep, 200" << endl;
    fout << "Send, (" << startPoint->x << "," << startPoint->y << "){Tab}{Enter}{Tab}Start+{Tab}+{Tab}{Enter}" << endl << "Sleep, 200" << endl;
    fout << "Send, (" << endPoint->x << "," << endPoint->y << "){Tab}{Enter}{Tab}End{Tab}{Tab}{Tab}{Tab}{Tab}" << endl << "Sleep, 200" << endl;

    vector<vector<Point*>*>* clusters = clusterify(obstaclePoints);

    //write the obstacle clusters to Desmos
    fout << "Send, Obstacle Clusters{Enter}" << endl;
    for (vector<Point*>* cluster : *clusters) printDesmos(cluster);
    fout << "Send, {Backspace}{Backspace}{Down}" << endl;

    //extend each cluster to account for space for the rover, and compute the convex hull
    auto* polygons = new vector<vector<Point*>*>;
    for (auto* cluster : *clusters) {
        auto* extendedCluster = new vector<Point*>;
        for (Point* p : *cluster) {
            extendedCluster->push_back(new Point{p->x + SLOP, p->y + SLOP});
            extendedCluster->push_back(new Point{p->x + SLOP, p->y - SLOP});
            extendedCluster->push_back(new Point{p->x - SLOP, p->y + SLOP});
            extendedCluster->push_back(new Point{p->x - SLOP, p->y - SLOP});
        }
        polygons->push_back(getConvexHull(extendedCluster));
    }

    //write the extended polygons to Desmos
    fout << "Send, Extended Polygons{Enter}" << endl;
    for (vector<Point*>* polygon : *polygons) printDesmos(polygon);
    fout << "Send, {Backspace}{Backspace}{Down}" << endl;

    vector<Point*>* path = aStar(createVisibilityGraph(startPoint, endPoint, polygons), endPoint);

    //write the final path to Desmos
    fout << "Send, Path{Enter}" << endl;
    for (unsigned int i = 0; i < path->size() - 1; i++) toDesmos(new LineSegment{path->at(i), path->at(i + 1)});
    fout << "Send, {Backspace}{Backspace}" << endl;

    return path;
}

//runs the path-finding algorithm on the data file generated by a previous run of the algorithm
vector<Point*>* runFromFile(string fileName) {
    ifstream fin("C:\\Users\\amaar\\Desktop\\Good Luck\\" + fileName + ".txt");
    if (!fin) {
        cout << "File not found!";
        return {};
    }

    float x, y;
    fin >> x >> y;
    auto* start = new Point{x, y};

    fin >> x >> y;
    auto* end = new Point{x, y};

    auto* obstacles = new vector<Point*>;
    while (fin >> x >> y) obstacles->push_back(new Point{x, y});

    shuffle(obstacles->begin(), obstacles->end(), default_random_engine{});

    return getPath(start, end, obstacles);
}

//generates a random float between min and max
float random(float min, float max) {
    return ((float) rand()) / ((float) RAND_MAX) * (max - min) + min;
}

//runs the path-finding algorithm on a randomly generated set of obstacles
//generates a data file that can be used to re-run the exact same test
vector<Point*>* runRandomObstacles(string outputFileName) {
    auto* start = new Point{0, 0};
    auto* end = new Point{100, 100};

    //create list of random obstacles
    auto* obstacles = new vector<Point*>;
    for (int i = 0; i < 20; i++) obstacles->push_back(new Point{random(0, 100), random(0, 100)});

    //write the start Point, end Point, and obstacles to a file so the same test can be reused later
    ofstream fout("C:\\Users\\amaar\\Desktop\\Good Luck\\" + outputFileName + ".txt");
    fout << start->x << " " << start->y << " " << end->x << " " << end->y << " ";
    for (Point* obstacle : *obstacles) fout << obstacle->x << " " << obstacle->y << " ";
    fout.close();

    return getPath(start, end, obstacles);
}

int main() {
    if (MIN_OBSTACLE_PROXIMITY < 2 * sqrt(2) * SLOP) {
        cout << "Obstacles could be placed close enough together that the robot can't move between them! Terminating!";
        return EXIT_FAILURE;
    }
    srand(static_cast<unsigned int>(time(nullptr)));
    for (Point* p : *runRandomObstacles("Test Data")) cout << "(" << p->x << ", " << p->y << "), ";
}