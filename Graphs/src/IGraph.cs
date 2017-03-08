using System.Collections.Generic;

namespace Graph
{
    /// <summary>
    /// An interface for graphs to implement. 
    /// </summary>
    /// <remarks>
    /// TODO: Consider adding:
    /// - GetCell(T cell)
    /// - IsOutOfBounds(T cell)
    /// - GetDistance(T cellA, T cellB)
    /// </remarks>
    public interface IGraph<T> where T : Vertex
    {
        /// <summary>
        /// Returns true, if there is an edge from vertex A to vertex B. 
        /// </summary>
        /// <returns></returns>
        bool IsAdjacent(T vertexA, T vertexB);

        /// <summary>
        /// Returns vertices that neighbor the given vertex. 
        /// </summary>
        /// <returns></returns>
        IEnumerable<T> GetNeighbors(T vertex);

    }
}
