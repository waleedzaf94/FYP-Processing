using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace UnmanagedWrapTest
{
    class Program
    {
        static void Main(string[] args)
        {
            Processing.CommandWrap test = new Processing.CommandWrap();
            test.Hello();
        }
    }
}
