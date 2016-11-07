using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DividersCount
{
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("Insert number, for which it is a need to find the number of dividers: ");
            int a = Convert.ToInt32(Console.ReadLine());
            Console.WriteLine("Dividers of " + a + " are: ");
            List<int> result = SimpleDividersFind(a);
            for (int i = 0; i < result.Count; ++i)
            {
                Console.WriteLine(result[i]);
            }           
            Console.ReadKey();
        }

        public static List <int> SimpleDividersFind(int a) //простой алгоритм (перебор делителей)
        {
            int divider = 1; 
            List<int> Dividers = new List<int>();
            if (a == 1) 
            {
                Dividers.Add(1);
            }
            if (a == 2)
            {
                Dividers.Add(2);
            }
            else
            {
                while (divider <= a / 2)
                {
                    if (a % divider == 0)
                    {
                        Dividers.Add(divider);
                    }
                    divider++;
                }
            }
            return Dividers;
        }

       }
}
