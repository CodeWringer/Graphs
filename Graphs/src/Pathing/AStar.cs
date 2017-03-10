using System;
using System.Collections.Generic;
using System.Linq;

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
                    newCost += grid.GetCost(current, neighbor);

                    if (!costSoFar.ContainsKey(neighbor) || newCost < costSoFar[neighbor])
                    {
                        if (costSoFar.ContainsKey(neighbor))
                            costSoFar[neighbor] = newCost;
                        else
                            costSoFar.Add(neighbor, newCost);

                        float priority = newCost + grid.GetHeuristic(goal, neighbor);

                        frontier.Enqueue(neighbor, priority);

                        if (cameFrom.ContainsKey(neighbor))
                            cameFrom[neighbor] = current;
                        else
                            cameFrom.Add(neighbor, current);
                    }
                }
            }

            return GraphUtility.ConstructPath(cameFrom, goal);
        }

        #endregion Methods
    }
}
