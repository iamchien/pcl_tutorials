#include <cppcolormap.h>

// std::ostream& operator<<(std::ostream& out, const Course& course)
// {
//     out << course.getName(); // for example
//     return out;
// }

int main()
{
    std::cout << cppcolormap::colormap("Reds", 256) << std::endl;

    return 0;
}