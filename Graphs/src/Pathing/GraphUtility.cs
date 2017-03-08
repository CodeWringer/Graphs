using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;

using Graph.Grid;

namespace Graph.Pathing
{
    /// <summary>
    /// An internal utility class for various graph operations. 
    /// </summary>
    internal abstract class GraphUtility
    {
        /*****************************************************************/
        // Methods
        /*****************************************************************/
        #region Methods

        #region GetCost

        /// <summary>
        /// Returns the cost difference between the given vertices. 
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="vertexA"></param>
        /// <param name="vertexB"></param>
        /// <param name="costDiagonal"></param>
        /// <returns></returns>
        internal static float GetCost<T>(T vertexA, T vertexB, float costDiagonal = 1.0F) where T : Vertex
        {
            // TODO: Figure out if diagonal cost can be removed. 
            float cost = Math.Max(vertexA.cost - vertexB.cost, 1.0F);

            return costDiagonal > 0 ? (cost * costDiagonal) : cost;
        }

        #endregion GetCost

        #region GetHeuristic

        /// <summary>
        /// Returns the Manhattan distance. 
        /// </summary>
        /// <param name="goal">A point representing the goal position. </param>
        /// <param name="point">A point to get the heuristic for. </param>
        /// <param name="D">The minimum cost between any adjacent cells. </param>
        /// <returns></returns>
        internal static float GetHeuristicManhattan(Point goal, Point point, float D)
        {
            float dx = Math.Abs(point.X - goal.X);
            float dy = Math.Abs(point.Y - goal.Y);
            return D * (dx + dy);
        }

        /// <summary>
        /// Returns the diagonal distance. 
        /// </summary>
        /// <param name="goal">A point representing the goal position. </param>
        /// <param name="point">A point to get the heuristic for. </param>
        /// <param name="D">The minimum cost between any adjacent cells. </param>
        /// <param name="D2">The cost of moving diagonally. </param>
        /// <returns></returns>
        internal static float GetHeuristicDiagonal(Point goal, Point point, float D, float D2)
        {
            float dx = Math.Abs(point.X - goal.X);
            float dy = Math.Abs(point.Y - goal.Y);
            return D * (dx + dy) + (D2 - 2 * D) * Math.Min(dx, dy);
        }

        /// <summary>
        /// Returns the Euclidean distance. 
        /// </summary>
        /// <param name="goal"></param>
        /// <param name="point"></param>
        /// <param name="D"></param>
        /// <returns></returns>
        internal static float GetHeuristicEuclidean(Point goal, Point point, float D)
        {
            float dx = Math.Abs(point.X - goal.X);
            float dy = Math.Abs(point.Y - goal.Y);
            return D * (float)Math.Sqrt(dx * dx + dy * dy);
        }

        #endregion GetHeuristic

        #region GetLowestCost

        /// <summary>
        /// Returns the lowest cost of all the neighboring cells of the given cell. 
        /// </summary>
        /// <param name="vertex"></param>
        /// <param name="grid"></param>
        /// <param name="costDiagonal"></param>
        /// <returns></returns>
        internal static float GetLowestCost<T>(T vertex, IGraph<T> grid, float costDiagonal) where T : Vertex
        {
            IEnumerable<T> neighbors = grid.GetNeighbors(vertex);
            float costLowest = float.MaxValue;

            // TODO: Clean up. 
            //if (costDiagonal <= 0)
            //    neighbors = oGrid.GetNeighbors(cell.Location, false);
            //else
            //    neighbors = oGrid.GetNeighbors(cell.Location, true);

            foreach (T neighbor in neighbors)
            {
                if (neighbor.cost < costLowest)
                    costLowest = neighbor.cost;
            }
            return costLowest;
        }

        #endregion GetLowestCost

        #region ConstructPath

        /// <summary>
        /// Returns a path constructed from a given dictionary of "came from" vertices and a given start and goal vertex. 
        /// </summary>
        /// <param name="start">A vertex the path originates from. </param>
        /// <param name="goal">A vertex the path leads towards. </param>
        /// <param name="cameFrom">A dictionary of "came from" vertices. Each key vertex is associated 
        /// with a value vertex which represents the vertex that was reached in order to get to the key vertex. </param>
        /// <returns></returns>
        internal static IEnumerable<T> ConstructPath<T>(Dictionary<T, T> cameFrom, T goal) where T : Vertex
        {
            if (goal == null || !cameFrom.ContainsKey(goal)) // No path - can not trace back from goal. 
                return null;

            // A list of vertices representing the traced back path. 
            List<T> path = new List<T>();
            // The currently looked at vertex. 
            T current = goal;

            // Trace path. 
            while (current != null)
            {
                path.Add(current);
                cameFrom.TryGetValue(current, out current);
            }
            path.Reverse();

            return path;
        }

        #endregion ConstructPath

        #endregion Methods
    }
}
