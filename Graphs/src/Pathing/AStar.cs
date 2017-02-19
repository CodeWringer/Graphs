using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;

using Tools.Maths.Point3;
using Graph.Grid;

namespace Graph.Pathing
{
    /// <summary>
    /// Pathfinding algorithm 'AStar'. 
    /// </summary>
    public class AStar
    {
        /*****************************************************************/
        // Declarations
        /*****************************************************************/
        #region Declarations

        #endregion Declarations
        /*****************************************************************/
        // Constructors
        /*****************************************************************/
        #region Constructors

        #endregion Constructors
        /*****************************************************************/
        // Methods
        /*****************************************************************/
        #region Methods

        #region GenericGrid

        /// <summary>
        /// Finds a path on the given grid and returns the path, beginning with the given start node. 
        /// </summary>
        /// <param name="pntStart">A node to begin the search at. </param>
        /// <param name="pntEnd">A node to end the search at. </param>
        /// <param name="grid">The grid to search on. </param>
        /// <param name="tileImpassable">The number associated with impassable tiles. </param>
        /// <returns></returns>
        public static IEnumerable<Point> GetPath(Point pntStart, Point pntEnd, int[,] grid, int tileImpassable)
        {
            throw new NotImplementedException();
        }

        #endregion GenericGrid

        #region SquareGrid



        #endregion SquareGrid

        #region HexGrid



        #endregion HexGrid

        #region PolygonGrid



        #endregion PolygonGrid

        #endregion Methods
        /*****************************************************************/
        // Events
        /*****************************************************************/
        #region Events

        #endregion Events
    }
}
