using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Graph
{
    /// <summary>
    /// Represents a vertex of a rectangular grid map. 
    /// </summary>
    [DebuggerDisplay("\\{ X = {X} Y = {Y} Z = {Z} cost = {cost} impassable = {impassable} \\}")]
    public class Vertex
    {
        /*****************************************************************/
        // Declarations
        /*****************************************************************/
        #region Declarations

        /// <summary>
        /// X coordinate of this vertex. 
        /// </summary>
        public int X { get; private set; }
        /// <summary>
        /// Y coordinate of this vertex. 
        /// </summary>
        public int Y { get; private set; }
        /// <summary>
        /// Z coordinate of this vertex. 
        /// </summary>
        public int Z { get; private set; }
        /// <summary>
        /// Pathing cost of this vertex. 
        /// </summary>
        public float cost;
        /// <summary>
        /// If true, renders this tile as impassable to path finding. 
        /// </summary>
        public bool impassable;

        #endregion Declarations
        /*****************************************************************/
        // Constructors
        /*****************************************************************/
        #region Constructors

        public Vertex(int x, int y, int z, float cost, bool impassable)
        {
            this.X = x;
            this.Y = y;
            this.Z = z;
            this.cost = cost;
            this.impassable = impassable;
        }

        #endregion Constructors
        /*****************************************************************/
        // Methods
        /*****************************************************************/
        #region Methods

        #endregion Methods
    }
}
