using System;
using System.Collections.Generic;
using System.Linq;

using Priority_Queue;

namespace Graph.Pathing
{
    /// <summary>
    /// Pathfinding algorithm 'Dijkstra'. 
    /// Considers movement costs. 
    /// Useful for finding all source destinations for a given target destination or vice versa. 
    /// </summary>
    public abstract class Dijkstra
    {
        /*****************************************************************/
        // Methods
        /*****************************************************************/
        #region Methods

        /// <summary>
        /// Finds a path on the given grid and returns the path, beginning with the given start cell. 
        /// </summary>
        /// <typeparam name="T">The "Vertex" class or a class inheriting from the "Vertex" class. </typeparam>
        /// <param name="start">A vertex to begin the search at. </param>
        /// <param name="goal">A vertex to end the search at. </param>
        /// <param name="grid">The grid to search on. </param>
        /// <returns></returns>
        public static IEnumerable<T> GetPath<T>(T start, T goal, IGraph<T> grid) where T : Vertex
        {
            Dictionary<T, T> cameFrom = Dijkstra.GetPath<T>(start, goal, grid, true);

            return GraphUtility.ConstructPath(cameFrom, goal);
        }

        /// <summary>
        /// Returns a path based on the given "came from" vertices and a given goal vertex. 
        /// </summary>
        /// <typeparam name="T">The "Vertex" class or a class inheriting from the "Vertex" class. </typeparam>
        /// <param name="cameFrom"></param>
        /// <param name="goal"></param>
        /// <returns></returns>
        public static IEnumerable<T> GetPath<T>(Dictionary<T, T> cameFrom, T goal) where T : Vertex
        {
            return GraphUtility.ConstructPath(cameFrom, goal);
        }

        /// <summary>
        /// Finds all paths on the given grid that lead to the given vertex and returns the paths. 
        /// </summary>
        /// <typeparam name="T">The "Vertex" class or a class inheriting from the "Vertex" class. </typeparam>
        /// <param name="goal">The vertex that all paths should lead to. </param>
        /// <param name="grid">The grid to search on. </param>
        /// <returns></returns>
        public static Dictionary<T, T> GetPaths<T>(T goal, IGraph<T> grid) where T : Vertex
        {
            return Dijkstra.GetPath<T>(goal, null, grid, true);
        }

        /// <summary>
        /// Finds a path on the given grid and returns the path, beginning with the given start cell. 
        /// </summary>
        /// <typeparam name="T">The "Vertex" class or a class inheriting from the "Vertex" class. </typeparam>
        /// <param name="start">A vertex to begin the search at. </param>
        /// <param name="goal">A vertex to end the search at. Will be ignored, if null. </param>
        /// <param name="grid">The grid to search on. </param>
        /// <param name="breakEarly">If true, will stop searching after reaching the goal. </param>
        /// <returns></returns>
        internal static Dictionary<T, T> GetPath<T>(T start, T goal, IGraph<T> grid, bool breakEarly) where T : Vertex
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

                if (goal != null && current == goal && breakEarly) // Reached goal destination. 
                    break;

                IEnumerable<T> neighbors = grid.GetNeighbors(current);

                for (int next = 0; next < neighbors.Count(); next++)
                {
                    T neighborNext = neighbors.ElementAt(next);

                    if (neighborNext.impassable) // Looking at impassable tile. 
                        continue;

                    float newCost = 0.0F;
                    costSoFar.TryGetValue(current, out newCost);
                    newCost += grid.GetCost(current, neighborNext);

                    if (!costSoFar.ContainsKey(neighborNext) || newCost < costSoFar[neighborNext])
                    {
                        if (costSoFar.ContainsKey(neighborNext))
                            costSoFar[neighborNext] = newCost;
                        else
                            costSoFar.Add(neighborNext, newCost);

                        float priority = newCost;
                        frontier.Enqueue(neighborNext, priority);

                        if (cameFrom.ContainsKey(neighborNext))
                            cameFrom[neighborNext] = current;
                        else
                            cameFrom.Add(neighborNext, current);
                    }
                }
            }

            return cameFrom;
        }

        #endregion Methods
    }
}
