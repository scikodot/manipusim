using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Graphics
{
    // class for communicatin between main thread (Window) and auxiliary threads (Model, Manager, etc.)
    class Dispatcher
    {
        // actions that main thread has to execute
        public static Queue<Action> ActionsQueue = new Queue<Action>();
    }
}
