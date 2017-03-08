using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;

using Graph.Grid;
using Priority_Queue;

namespace Graph.Pathing
{
    /// <summary>
    /// Pathfinding algorithm 'AStar'. 
    /// Uses a heuristic to find the optimal path. 
    /// Useful for finding a single target destination for a given source destination or vice versa. 
    /// </summary>
    public abstract class AStar
    {
        /*****************************************************************/
        // Methods
        /*****************************************************************/
        #region Methods

        /// <summary>
        /// Finds a path on the given grid and returns the path, beginning with the given start cell. 
        /// </summary>
        /// <param name="start">A vertex to begin the search at. </param>
        /// <param name="goal">A vertex to end the search at. </param>
        /// <param name="grid">The grid to search on. </param>
        /// <returns></returns>
        public static IEnumerable<T> GetPath<T>(T start, T goal, IGraph<T> grid, float costDiagonal = 1.4F) where T : Vertex
        {
            SimplePriorityQueue<T> frontier = new SimplePriorityQueue<T>();
            List<T> lPath = new List<T>();
            Dictionary<T, T> cameFrom = new Dictionary<T, T>();
            Dictionary<T, float> costSoFar = new Dictionary<T, float>();
            costSoFar.Add(start, 0);

            frontier.Enqueue(start, 0);
            cameFrom.Add(start, null);
            T current = null;

            // Traverse map. 
            while (frontier.Count() != 0)
            {
                current = frontier.Dequeue();

                if (current == goal) // Reached goal destination. 
                    break;

                IEnumerable<T> neighbors = grid.GetNeighbors(current);

                for (int next = 0; next < neighbors.Count(); next++)
                {
                    T neighbor = neighbors.ElementAt(next);

                    if (neighbor.impassable) // Looking at impassable tile. 
                        continue;

                    // Get cost. 
                    float newCost = 0.0F;
                    costSoFar.TryGetValue(current, out newCost);
                    newCost += GraphUtility.GetCost(current, neighbor, costDiagonal);

                    if (!costSoFar.ContainsKey(neighbor) || newCost < costSoFar[neighbor])
                    {
                        if (costSoFar.ContainsKey(neighbor))
                            costSoFar[neighbor] = newCost;
                        else
                            costSoFar.Add(neighbor, newCost);

                        float priority = 0;
                        float D = GraphUtility.GetLowestCost(neighbor, grid, costDiagonal);

                        Point pntGoal = new Point(goal.X, goal.Y);
                        Point pntNeighbor = new Point(neighbor.X, neighbor.Y);

                        if (costDiagonal > 0)
                        {
                            float D2 = GraphUtility.GetCost(current, neighbor, costDiagonal);
                            priority = newCost + GraphUtility.GetHeuristicDiagonal(pntGoal, pntNeighbor, D, D2);
                        }
                        else
                        {
                            priority = newCost + GraphUtility.GetHeuristicManhattan(pntGoal, pntNeighbor, D);
                        }

                        frontier.Enqueue(neighbor, priority);
                        cameFrom.Add(neighbor, current); // TODO: Key can be added twice?
                    }
                }
            }

            return GraphUtility.ConstructPath(cameFrom, goal);
        }

        #endregion Methods
    }
}
