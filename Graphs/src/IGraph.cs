using System.Collections.Generic;

namespace Graph
{
    /// <summary>
    /// An interface for graphs to implement in order to allow common functionality to be used in path finding. 
    /// </summary>
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

        /// <summary>
        /// Returns the cost difference between the given vertices. 
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="vertexA"></param>
        /// <param name="vertexB"></param>
        /// <returns></returns>
        float GetCost(T vertexA, T vertexB);

        /// <summary>
        /// Returns the lowest cost of all the neighboring vertices of the given vertex. 
        /// </summary>
        /// <param name="vertex"></param>
        /// <returns></returns>
        float GetCostLowest(T vertex);

        /// <summary>
        /// Returns the distance between the given vertices. 
        /// </summary>
        /// <param name="vertexA"></param>
        /// <param name="vertexB"></param>
        /// <returns></returns>
        float GetDistance(T vertexA, T vertexB);

        /// <summary>
        /// Returns the heuristic value between the given vertices. 
        /// </summary>
        /// <param name="vertexA"></param>
        /// <param name="vertexB"></param>
        /// <param name=""></param>
        /// <returns></returns>
        float GetHeuristic(T vertexA, T vertexB);
    }
}