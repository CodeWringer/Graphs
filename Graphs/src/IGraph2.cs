using System.Collections.Generic;

namespace Graph
{
    /// <summary>
    /// An interface for two dimensional graphs to implement. 
    /// </summary>
    interface IGraph2<T>
    {
        /// <summary>
        /// Returns true, if there is an edge from vertex A to vertex B. 
        /// </summary>
        /// <returns></returns>
        bool IsAdjacent(T vertexA, T vertexB);

        /// <summary>
        /// Returns a two dimensional array of vertices that neighbor the given vertex. 
        /// </summary>
        /// <returns></returns>
        IEnumerable<T> GetNeighbors(T vertex);

        /// <summary>
        /// Adds the given vertex, if possible and returns true, if successful. 
        /// </summary>
        /// <param name="vertex"></param>
        /// <returns></returns>
        bool AddVertex(T vertex);

        /// <summary>
        /// Removes the given vertex, if possible and returns true, if successful. 
        /// </summary>
        /// <param name="vertex"></param>
        /// <returns></returns>
        bool RemoveVertex(T vertex);

        /// <summary>
        /// Adds an edge for the given vertices, if possible and returns true, if successful. 
        /// </summary>
        /// <returns></returns>
        bool AddEdge(T vertexA, T vertexB);

        /// <summary>
        /// Adds an edge for the given vertices, if possible and returns true, if successful. 
        /// </summary>
        /// <returns></returns>
        bool RemoveEdge(T vertexA, T vertexB);

    }
}
