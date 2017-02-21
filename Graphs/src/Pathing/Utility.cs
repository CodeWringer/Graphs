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
    internal abstract class Utility
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

        /// <summary>
        /// Returns the coordinates of all neighboring tiles. 
        /// </summary>
        /// <param name="pnt">A point for which all neighbors are to be returned. </param>
        /// <param name="grid">A gid to search on. </param>
        /// <param name="allowDiagonal">If true, also returns diagonal neighbors. </param>
        /// <returns></returns>
        internal static IEnumerable<Point> GetNeighbors(Point pnt, int[,] grid, bool allowDiagonal)
        {
            List<Point> neighbors = new List<Point>();

            int xAt = -1;
            int yAt = -1;

            // Get axis aligned neighbors. 
            for (int i = 0; i < SquareGrid.Directions.Length; i++)
            {
                xAt = pnt.X + SquareGrid.Directions[i].X;
                yAt = pnt.Y + SquareGrid.Directions[i].Y;

                if (!Utility.IsOutOfBounds(xAt, yAt, grid))
                {
                    neighbors.Add(new Point(xAt, yAt));
                }
            }

            if (allowDiagonal)
            {
                // Get diagonal neighbors. 
                for (int i = 0; i < SquareGrid.DirectionsDiag.Length; i++)
                {
                    xAt = pnt.X + SquareGrid.DirectionsDiag[i].X;
                    yAt = pnt.Y + SquareGrid.DirectionsDiag[i].Y;

                    if (!Utility.IsOutOfBounds(xAt, yAt, grid))
                    {
                        neighbors.Add(new Point(xAt, yAt));
                    }
                }
            }

            return neighbors;
        }

        /// <summary>
        /// Returns true, if the given coordinates represent a diagonal direction. 
        /// </summary>
        /// <param name="x">A x-coordinate in the range of -1 to +1. </param>
        /// <param name="y">A y-coordinate in the range of -1 to +1. </param>
        /// <returns></returns>
        internal static bool IsDiagonal(int x, int y)
        {
            if (x == 0 || y == 0)
                return false;
            else
                return true;
        }

        /// <summary>
        /// Returns true, if the given coordinates are out of bounds. 
        /// </summary>
        /// <param name="x">A x-coordinate. </param>
        /// <param name="y">A y-coordinate. </param>
        /// <returns></returns>
        internal static bool IsOutOfBounds(int x, int y, int[,] grid)
        {
            if (x < 0 || y < 0 || x >= grid.GetLength(0) || y >= grid.GetLength(1))
            {
                return true;
            }

            return false;
        }

        /// <summary>
        /// Returns the cost difference between the given points on the given grid of traversal costs. 
        /// </summary>
        /// <param name="gridCost">A grid of traversal costs. </param>
        /// <param name="pntA">A point. </param>
        /// <param name="pntB">A neighbor point. </param>
        /// <param name="costDiagonal">A cost factor for diagonal tiles. A value of less than 
        /// or equal to 0 won't be used. Can be used to weigh diagonal tiles higher. </param>
        /// <returns></returns>
        internal static float GetCost(int[,] gridCost, Point pntA, Point pntB, float costDiagonal = 1.0F)
        {
            float cost = Math.Max(gridCost[pntB.X, pntB.Y] - gridCost[pntA.X, pntA.Y], 1);

            return costDiagonal > 0 ? (cost * costDiagonal) : cost;
        }

        /// <summary>
        /// Returns the cost difference between the given square cells. 
        /// </summary>
        /// <param name="cellA">A cell. </param>
        /// <param name="cellB">A neighboring cell. </param>
        /// <param name="costDiagonal">A cost factor for diagonal tiles. A value of less than 
        /// or equal to 0 won't be used. Can be used to weigh diagonal tiles higher. </param>
        /// <returns></returns>
        internal static float GetCost(SquareCell cellA, SquareCell cellB, float costDiagonal = 1.0F)
        {
            float cost = Math.Max(cellB.Cost - cellA.Cost, 1.0F);

            return costDiagonal > 0 ? (cost * costDiagonal) : cost;
        }

        /// <summary>
        /// Returns the heuristic value for the given points. 
        /// </summary>
        /// <param name="goal">A point representing the goal position. </param>
        /// <param name="point">A point to get the heuristic for. </param>
        /// <returns></returns>
        internal static float GetHeuristic(Point goal, Point point)
        {
            return Math.Abs(goal.X - point.X) + Math.Abs(goal.Y - point.Y);
        }

        /// <summary>
        /// Returns the heuristic value for the given cells. 
        /// </summary>
        /// <param name="goal">A cell representing the goal position. </param>
        /// <param name="cell">A cell to get the heuristic for. </param>
        /// <returns></returns>
        internal static float GetHeuristic(SquareCell goal, SquareCell cell)
        {
            return Math.Abs(goal.X - cell.X) + Math.Abs(goal.Y - cell.Y);
        }

        #endregion Methods
    }
}
