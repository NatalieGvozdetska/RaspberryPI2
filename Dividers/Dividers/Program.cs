using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Dividers
{
    class Program
    {
        static void Main(string[] args)
        {
        }

        uint[] simpl_arr = //массив заполненный простыми числами

public List<int> GetSimpleDividors(uint number)
        {
            List<int> result = new List<int>();
            uint s = Convert.ToUInt32(Math.Ceiling(Math.Sqrt(number)));

            for (int i = 0; simple_arr[i] < s; i++)
            {
                if (number % simple_arr[i] == 0)
                {
                    result.Add(simple_arr[i]);
                    uint quotient = number / simple_arr[i];

                    var tmp = GetSimpleDividors(quotent);

                    if (tmp.Count > 0)
                    {
                        result.AddRange(tmp);
                    }
                    else
                    {
                        result.Add(quotent);
                    }
                    break;
                }
            }

            return result;
        }
    }
}
