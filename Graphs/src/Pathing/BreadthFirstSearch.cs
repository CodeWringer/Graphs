using System;
using System.Collections.Generic;
using System.Linq;

namespace Graph.Pathing
{
    /// <summary>
    /// Pathfinding algorithm 'BreadthFirstSearch'. 
    /// Useful for finding all source destinations for a given target destination or vice versa. 
    /// </summary>
    public abstract class BreadthFirstSearch
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
            Dictionary<T, T> cameFrom = BreadthFirstSearch.GetPath<T>(start, goal, grid, true);

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
            return BreadthFirstSearch.GetPath<T>(goal, null, grid, true);
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
            if (start == null)
                throw new ArgumentNullException("start", "The given start vertex must not be null!");

            if (grid == null)
                throw new ArgumentNullException("grid", "The given grid must not be null!");

            Queue<T> frontier = new Queue<T>();
            List<T> lPath = new List<T>();
            Dictionary<T, T> cameFrom = new Dictionary<T, T>();

            frontier.Enqueue(start);
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

                    if (!cameFrom.ContainsKey(neighborNext)) // vertex not yet looked at. 
                    {
                        frontier.Enqueue(neighborNext);
                        cameFrom.Add(neighborNext, current); // Add trace back for the neighbor to the current cell. 
                    }
                }
            }

            return cameFrom;
        }

        #endregion Methods
    }
}
