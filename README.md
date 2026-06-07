# Game AI Project

For this assignment I implemented two pathfinding algorithms and one path smoothing algorithm 
inside the provided framework:

## BFS(Breadth-First Search)
Finds a path between two nodes by exploring the graph level by level using a queue.
The path is reconstructed at the end by backtracking through a map of each node's previous node.

## A*
Finds the shortest path between two nodes on a weighted graph using an open and closed list.
Once the goal is reached, the path is reconstructed by backtracking through the closed list.

## Funnel Algorithm (SSFA)
Takes a node path and turns it into a smooth, shorter path over a navigation mesh.
First builds a list of portals (the edges to cross between consecutive nodes) and makes sure 
they are consistently oriented. Then walks through them maintaining a funnel of two legs 
(left and right) from an apex point, tightening it at each portal. When a new portal crosses 
over the opposite leg, the funnel collapses, a waypoint is added, and the funnel resets.

## Extra Assignment
I decided to make Jump Point Search algorithm, which is an extra assignment from Week 5 - Pathfinding Algorithms
