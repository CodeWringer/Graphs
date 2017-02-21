using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;

using Tools.Maths.Point3;
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
        // Declarations
        /*****************************************************************/
        #region Declarations

        #endregion Declarations
        /*****************************************************************/
        // Methods
        /*****************************************************************/
        #region Methods

        #region SimpleGrid

        /// <summary>
        /// Finds a path on the given grid and returns the path, beginning with the given start node. 
        /// </summary>
        /// <param name="pntStart">A node to begin the search at. </param>
        /// <param name="pntGoal">A node to end the search at. </param>
        /// <param name="grid">The grid to search on. </param>
        /// <param name="tileImpassable">The number associated with impassable tiles. </param>
        /// <param name="costDiagonal">A cost multiplication factor for diagonal tiles. A value of less than 
        /// or equal to 0 means no diagonal searching is allowed. </param>
        /// <returns></returns>
        public static IEnumerable<Point> GetPath(Point pntStart, Point pntGoal, int[,] grid, int[,] gridCost, int tileImpassable, float costDiagonal = 1.4F)
        {
            SimplePriorityQueue<Point> frontier = new SimplePriorityQueue<Point>();
            Point?[,] cameFrom = new Point?[grid.GetLength(0), grid.GetLength(1)];
            float?[,] costSoFar = new float?[grid.GetLength(0), grid.GetLength(1)];
            costSoFar[pntStart.X, pntStart.Y] = 0;

            frontier.Enqueue(pntStart, 0);
            cameFrom[pntStart.X, pntStart.Y] = null;
            Point nodeCurrent = new Point(); ;

            // Traverse map. 
            while (frontier.Count() != 0)
            {
                nodeCurrent = frontier.Dequeue();

                if (nodeCurrent == pntGoal) // Reached goal destination. 
                    break;

                IEnumerable<Point> neighbors = null;

                if (costDiagonal <= 0)
                    neighbors = Utility.GetNeighbors(nodeCurrent, grid, false);
                else
                    neighbors = Utility.GetNeighbors(nodeCurrent, grid, true);

                for (int next = 0; next < neighbors.Count(); next++)
                {
                    Point nodeNext = neighbors.ElementAt(next);

                    if (grid[nodeNext.X, nodeNext.Y] == tileImpassable) // Looking at impassable tile. 
                        continue;

                    // Get cost. 
                    float newCost = costSoFar[nodeCurrent.X, nodeCurrent.Y].Value + Utility.GetCost(gridCost, nodeCurrent, nodeNext, costDiagonal);

                    if (costSoFar[nodeNext.X, nodeNext.Y] == null || newCost < costSoFar[nodeNext.X, nodeNext.Y].Value)
                    {
                        costSoFar[nodeNext.X, nodeNext.Y] = newCost;

                        float priority = 0;
                        float D = Utility.GetLowestCost(nodeNext, grid, gridCost, costDiagonal);

                        if (costDiagonal > 0)
                        {
                            float D2 = Utility.GetCost(gridCost, nodeCurrent, nodeNext, costDiagonal);
                            priority = newCost + Utility.GetHeuristicDiagonal(pntGoal, nodeNext, D, D2);
                        }
                        else
                        {
                            priority = newCost + Utility.GetHeuristicManhattan(pntGoal, nodeNext, D);
                        }

                        frontier.Enqueue(nodeNext, priority);
                        cameFrom[nodeNext.X, nodeNext.Y] = nodeCurrent;
                    }
                }
            }

            return Utility.ConstructPath(nodeCurrent, pntStart, pntGoal, cameFrom);
        }
        
        #endregion SimpleGrid

        #region SquareGrid

        /// <summary>
        /// Finds a path on the given grid and returns the path, beginning with the given start node. 
        /// </summary>
        /// <param name="pntStart">A node to begin the search at. </param>
        /// <param name="pntGoal">A node to end the search at. </param>
        /// <param name="oGrid">The grid do to the search with. </param>
        /// <param name="costDiagonal">A cost multiplication factor for diagonal tiles. A value of less than 
        /// or equal to 0 means no diagonal searching is allowed. </param>
        /// <returns></returns>
        public static IEnumerable<SquareCell> GetPath(Point pntStart, Point pntGoal, SquareGrid oGrid, float costDiagonal = 1.4F)
        {
            SimplePriorityQueue<SquareCell> frontier = new SimplePriorityQueue<SquareCell>();
            SquareCell[,] cameFrom = new SquareCell[oGrid.Width, oGrid.Height];
            float?[,] costSoFar = new float?[oGrid.Width, oGrid.Height];
            costSoFar[pntStart.X, pntStart.Y] = 0;

            bool allowDiagonal = costDiagonal > 0 ? true : false;

            frontier.Enqueue(oGrid.GetAt(pntStart.X, pntStart.Y), 0);
            cameFrom[pntStart.X, pntStart.Y] = null;
            SquareCell nodeCurrent = null;

            // Traverse map. 
            while (frontier.Count() != 0)
            {
                nodeCurrent = frontier.Dequeue();

                if (nodeCurrent == oGrid.GetAt(pntGoal.X, pntGoal.Y)) // Reached goal destination. 
                    break;

                IEnumerable<SquareCell> neighbors = null;

                if (allowDiagonal)
                    neighbors = oGrid.GetNeighbors(nodeCurrent.Location, true);
                else
                    neighbors = oGrid.GetNeighbors(nodeCurrent.Location, false);

                for (int next = 0; next < neighbors.Count(); next++)
                {
                    SquareCell nodeNext = neighbors.ElementAt(next);

                    if (oGrid.GetAt(nodeNext.X, nodeNext.Y).impassable) // Looking at impassable tile. 
                        continue;

                    // Get cost. 
                    float newCost = costSoFar[nodeCurrent.X, nodeCurrent.Y].Value + Utility.GetCost(nodeCurrent, nodeNext, costDiagonal);

                    if (costSoFar[nodeNext.X, nodeNext.Y] == null || newCost < costSoFar[nodeNext.X, nodeNext.Y].Value)
                    {
                        costSoFar[nodeNext.X, nodeNext.Y] = newCost;

                        float priority = 0;
                        float D = Utility.GetLowestCost(nodeNext, oGrid, costDiagonal);

                        if (costDiagonal > 0)
                        {
                            float D2 = Utility.GetCost(nodeCurrent, nodeNext, costDiagonal) * costDiagonal;
                            priority = newCost + Utility.GetHeuristicDiagonal(pntGoal, nodeNext.Location, D, D2);
                        }
                        else
                        {
                            priority = newCost + Utility.GetHeuristicManhattan(pntGoal, nodeNext.Location, D);
                        }

                        frontier.Enqueue(nodeNext, priority);
                        cameFrom[nodeNext.X, nodeNext.Y] = nodeCurrent;
                    }
                }
            }

            return Utility.ConstructPath(nodeCurrent, pntStart, pntGoal, cameFrom);
        }
        
        #endregion SquareGrid

        #region HexGrid



        #endregion HexGrid

        #region PolygonGrid



        #endregion PolygonGrid

        #endregion Methods
    }
}
