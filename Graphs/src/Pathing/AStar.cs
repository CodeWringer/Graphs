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
        /// <param name="pntEnd">A node to end the search at. </param>
        /// <param name="grid">The grid to search on. </param>
        /// <param name="tileImpassable">The number associated with impassable tiles. </param>
        /// <param name="costDiagonal">A cost multiplication factor for diagonal tiles. A value of less than 
        /// or equal to 0 means no diagonal searching is allowed. </param>
        /// <returns></returns>
        public static IEnumerable<Point> GetPath(Point pntStart, Point pntEnd, int[,] grid, int[,] gridCost, int tileImpassable, float costDiagonal = 1.4F)
        {
            SimplePriorityQueue<Point> frontier = new SimplePriorityQueue<Point>();
            List<Point> lPath = new List<Point>();
            Point?[,] cameFrom = new Point?[grid.GetLength(0), grid.GetLength(1)];
            float?[,] costSoFar = new float?[grid.GetLength(0), grid.GetLength(1)];
            costSoFar[pntStart.X, pntStart.Y] = 0;

            frontier.Enqueue(pntStart, 0);
            cameFrom[pntStart.X, pntStart.Y] = null;
            Point current;

            // Traverse map. 
            while (frontier.Count() != 0)
            {
                current = frontier.Dequeue();

                if (current == pntEnd) // Reached goal destination. 
                    break;

                IEnumerable<Point> neighbors = null;

                if (costDiagonal <= 0)
                    neighbors = Utility.GetNeighbors(current, grid, false);
                else
                    neighbors = Utility.GetNeighbors(current, grid, true);

                for (int next = 0; next < neighbors.Count(); next++)
                {
                    Point pntNext = neighbors.ElementAt(next);

                    if (grid[pntNext.X, pntNext.Y] == tileImpassable) // Looking at impassable tile. 
                        continue;

                    // Get cost. 
                    float newCost = costSoFar[current.X, current.Y].Value + Utility.GetCost(gridCost, current, pntNext, costDiagonal);

                    if (costSoFar[pntNext.X, pntNext.Y] == null || newCost < costSoFar[pntNext.X, pntNext.Y].Value)
                    {
                        costSoFar[pntNext.X, pntNext.Y] = newCost;
                        float priority = newCost + Utility.GetHeuristic(pntEnd, pntNext);
                        frontier.Enqueue(pntNext, priority);
                        cameFrom[pntNext.X, pntNext.Y] = current;
                    }
                }
            }

            if (cameFrom[pntEnd.X, pntEnd.Y] == null) // Could not find path. 
                return null;

            // Construct path. 
            current = pntEnd;
            lPath.Add(current);
            while (current != pntStart)
            {
                current = cameFrom[current.X, current.Y].Value;
                lPath.Add(current);
            }
            lPath.Add(pntStart);
            lPath.Reverse();

            return lPath;
        }
        
        #endregion SimpleGrid

        #region SquareGrid

        /// <summary>
        /// Finds a path on the given grid and returns the path, beginning with the given start node. 
        /// </summary>
        /// <remarks>
        /// Does an early exit. 
        /// </remarks>
        /// <param name="pntStart">A node to begin the search at. </param>
        /// <param name="pntEnd">A node to end the search at. </param>
        /// <param name="oGrid">The grid do to the search with. </param>
        /// <param name="costDiagonal">A cost multiplication factor for diagonal tiles. A value of less than 
        /// or equal to 0 means no diagonal searching is allowed. </param>
        /// <returns></returns>
        public static IEnumerable<SquareCell> GetPath(Point pntStart, Point pntEnd, SquareGrid oGrid, float costDiagonal = 1.4F)
        {
            SimplePriorityQueue<SquareCell> frontier = new SimplePriorityQueue<SquareCell>();
            SquareCell[,] cameFrom = new SquareCell[oGrid.Width, oGrid.Height];
            float?[,] costSoFar = new float?[oGrid.Width, oGrid.Height];
            costSoFar[pntStart.X, pntStart.Y] = 0;

            frontier.Enqueue(oGrid.GetAt(pntStart.X, pntStart.Y), 0);
            cameFrom[pntStart.X, pntStart.Y] = null;
            SquareCell current;

            // Traverse map. 
            while (frontier.Count() != 0)
            {
                current = frontier.Dequeue();

                if (current == oGrid.GetAt(pntEnd.X, pntEnd.Y)) // Reached goal destination. 
                    break;

                IEnumerable<SquareCell> neighbors = null;

                if (costDiagonal <= 0)
                    neighbors = oGrid.GetNeighbors(current.Location, false);
                else
                    neighbors = oGrid.GetNeighbors(current.Location, true);

                for (int next = 0; next < neighbors.Count(); next++)
                {
                    SquareCell pntNext = neighbors.ElementAt(next);

                    if (oGrid.GetAt(pntNext.X, pntNext.Y).impassable) // Looking at impassable tile. 
                        continue;

                    // Get cost. 
                    float newCost = costSoFar[current.X, current.Y].Value + Utility.GetCost(current, pntNext, costDiagonal);

                    if (costSoFar[pntNext.X, pntNext.Y] == null || newCost < costSoFar[pntNext.X, pntNext.Y].Value)
                    {
                        costSoFar[pntNext.X, pntNext.Y] = newCost;
                        float priority = newCost + Utility.GetHeuristic(oGrid.GetAt(pntEnd.X, pntEnd.Y), pntNext);
                        frontier.Enqueue(pntNext, priority);
                        cameFrom[pntNext.X, pntNext.Y] = current;
                    }
                }
            }

            if (cameFrom[pntEnd.X, pntEnd.Y] == null) // Could not find path. 
                return null;

            // Construct path. 
            List<SquareCell> lPath = new List<SquareCell>();
            current = oGrid.GetAt(pntEnd.X, pntEnd.Y);
            lPath.Add(current);
            while (current != oGrid.GetAt(pntStart.X, pntStart.Y))
            {
                current = cameFrom[current.X, current.Y];
                lPath.Add(current);
            }
            lPath.Add(oGrid.GetAt(pntStart.X, pntStart.Y));
            lPath.Reverse();

            return lPath;
        }
        
        #endregion SquareGrid

        #region HexGrid



        #endregion HexGrid

        #region PolygonGrid



        #endregion PolygonGrid

        #endregion Methods
    }
}
