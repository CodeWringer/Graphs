using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

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
