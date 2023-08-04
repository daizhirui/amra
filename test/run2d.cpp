// project includes
#include <amra/grid2d.hpp>

// system includes

// standard includes
#include <vector>
#include <chrono>

using namespace AMRA;

int
main(int argc, char** argv) {
    std::string mapfile(argv[1]);

    // // run 2D gridworld planning for fixed starts and goals
    // std::vector<std::vector<int> > starts, goals;
    // starts = { {28, 12} };
    // goals = { {5, 45} };
    // for (int i = 0; i < starts.size(); ++i)
    // {
    // 	Grid2D grid(mapfile);
    // 	grid.CreateSearch();
    // 	// grid.CreateARAStarSearch();
    // 	grid.SetStart(starts[i][0], starts[i][1]);
    // 	grid.SetGoal(goals[i][0], goals[i][1]);
    // 	grid.Plan(true);
    // }

    // run 2D gridworld planning for random starts and goals
    for (int i = 0; i < 1; ++i) {
        Grid2D grid(mapfile);
        if (mapfile.rfind("Boston_0_1024.map") != std::string::npos) {
            grid.SetStart(828, 657);
            grid.SetGoal(1008, 756);
        } else if (mapfile.rfind("Cauldron.map") != std::string::npos) {
            grid.SetStart(342, 450);
            grid.SetGoal(522, 333);
        } else if (mapfile.rfind("Denver_0_1024.map") != std::string::npos) {
            grid.SetStart(306, 171);
            grid.SetGoal(1008, 603);
        } else if (mapfile.rfind("Expedition.map") != std::string::npos) {
            grid.SetStart(720, 423);
            grid.SetGoal(198, 891);
        } else if (mapfile.rfind("NewYork_0_1024.map") != std::string::npos) {
            grid.SetStart(0, 171);
            grid.SetGoal(612, 882);
        } else if (mapfile.rfind("Octopus.map") != std::string::npos) {
            grid.SetStart(549, 837);
            grid.SetGoal(639, 279);
        } else if (mapfile.rfind("TheFrozenSea.map") != std::string::npos) {
            grid.SetStart(288, 414);
            grid.SetGoal(207, 990);
        } else {
            std::cout << "****************************" << std::endl
                      << " USE RANDOM START AND GOAL! "
                      << "****************************" << std::endl;
        }
        grid.CreateSearch();
        auto t0 = std::chrono::system_clock::now();
        grid.Plan(false);
        auto t1 = std::chrono::system_clock::now();
        std::cout << "Planning time: "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                  << " ms" << std::endl;
    }
}
