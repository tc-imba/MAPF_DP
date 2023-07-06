//
// Created by liuyh on 7/7/2023.
//

#include <boost/program_options.hpp>
#include <string>


class Config {
private:
    template<typename T, typename Comp = std::greater_equal<T>>
    void validate(const boost::program_options::variables_map &vm, const std::string &option_name, const T &value) {
        namespace po = boost::program_options;
        if (vm.count(option_name) == 0) return;
        auto comp = Comp();
        auto &v = vm[option_name].as<T>();
        if (v < std::numeric_limits<T>::max() / 2 && comp(v, value)) return;
        throw po::validation_error(
                po::validation_error::kind_t::invalid_option_value,//
                option_name,                                       //
                std::to_string(v)                                  //
        );
    }


public:
    // Generic Options
    std::string configFile;
    bool debug = false;

    // Core Configuration
    std::string mapType;
    std::string objective;
    std::string simulatorType;
    std::string timingType;
    std::string conflictTypes;
    std::string delayType;
    std::string solverType;
    std::string solverBinaryFile;

    // Simulation Configuration
    unsigned long mapSeed = 0;
    unsigned long agentSeed = 0;
    unsigned long simulationSeed = 0;
    unsigned long agentNum = 10;
    unsigned long iteration = 10;
    unsigned long obstacles = 90;
    unsigned long kNeighbor = 2;
    double delayRatio = 0.2;
    double delayInterval = 1;
    double delayStart = 0;
    double suboptimality = 1;

    // Feature Configuration
    bool naiveFeasibilityCheck = false;
    bool naiveCycleCheck = false;
    bool onlyCycleCheck = false;
    bool feasibilityType = false;
    bool prioritizedReplan = false;
    bool prioritizedOpt = false;
    bool noCache = false;

    // I/O Configuration
    std::string mapFile;
    std::string taskFile;
    std::string logFile;
    std::string outputFile;
    std::string simulatorOutputFile;
    std::string timeOutputFile;


    //    std::string mapType, objective, simulatorType, timingType, conflictTypes, outputFileName, simulatorOutputFileName, timeOutputFileName, delayType, solverType, solverBinaryFile, mapFile, taskFile, logFile;
    //    unsigned long window, mapSeed, agentSeed, simulationSeed, agentNum, iteration, obstacles, kNeighbor;
    //    long delayStart, delayInterval;
    //    double minDP, maxDP, delayRatio, suboptimality;
    //    bool debug, allConstraint, useDP, naiveFeasibilityCheck, naiveCycleCheck, onlyCycleCheck, feasibilityType, prioritizedReplan, prioritizedOpt, noCache;

    Config(int argc, const char *argv[]) {
        namespace po = boost::program_options;
        try {
            po::options_description generic("Generic Options");                                                //
            generic.add_options()                                                                              //
                    ("help,h", "Display help message")                                                         //
                    ("debug,d", po::bool_switch(&debug), "Enable debug mode")                                  //
                    ("config,c", po::value<std::string>(&configFile), "Configuration file")                    //
                    ;                                                                                          //
            po::options_description core_config("Core Configuration");                                         //
            core_config.add_options()                                                                          //
                    ("map,m", po::value<std::string>(&mapType)->default_value("random"),                       //
                     "Map type (random / warehouse / graphml)")                                                //
                    ("objective", po::value<std::string>(&objective)->default_value("maximum"),                //
                     "Objective type (maximum / average)")                                                     //
                    ("simulator", po::value<std::string>(&simulatorType),                                      //
                     "Simulator type (default / online / replan / pibt)")                                      //
                    ("timing", po::value<std::string>(&timingType),                                            //
                     "Timing type (discrete / continuous)")                                                    //
                    ("conflicts", po::value<std::string>(&conflictTypes),                                      //
                     "Conflict types")                                                                         //
                    ("delay", po::value<std::string>(&delayType),                                              //
                     "Delay type (agent / node / edge)")                                                       //
                    ("solver", po::value<std::string>(&solverType),                                            //
                     "Solver (default / separate / eecbs / ccbs")                                              //
                    ("solver-binary", po::value<std::string>(&solverBinaryFile),                               //
                     "Solver binary (for eecbs / ccbs)")                                                       //
                    ;                                                                                          //
            po::options_description simulation_config("Simulation Configuration");                             //
            simulation_config.add_options()                                                                    //
                    ("map-seed", po::value<unsigned long>(&mapSeed)->default_value(0),                         //
                     "Map Seed")                                                                               //
                    ("agent-seed", po::value<unsigned long>(&agentSeed)->default_value(0),                     //
                     "Agent Seed")                                                                             //
                    ("simulation-seed", po::value<unsigned long>(&simulationSeed)->default_value(0),           //
                     "Simulation Seed")                                                                        //
                    ("agents,a", po::value<unsigned long>(&agentNum)->default_value(10),                       //
                     "Agent Number")                                                                           //
                    ("iteration,i", po::value<unsigned long>(&iteration)->default_value(10),                   //
                     "Iteration Number")                                                                       //
                    ("obstacles", po::value<unsigned long>(&obstacles)->default_value(90),                     //
                     "Obstacles")                                                                              //
                    ("k-neighbor", po::value<unsigned long>(&kNeighbor)->default_value(2),                     //
                     "K-th neighborhood in path planning")                                                     //
                    ("delay-ratio", po::value<double>(&delayRatio)->default_value(0.2),                        //
                     "Delay Ratio")                                                                            //
                    ("delay-interval", po::value<double>(&delayInterval)->default_value(1),                    //
                     "Delay Interval")                                                                         //
                    ("delay-start", po::value<double>(&delayStart)->default_value(0),                          //
                     "Delay Start")                                                                            //
                    ("suboptimality", po::value<double>(&suboptimality)->default_value(1),                     //
                     "Suboptimality of CBS")                                                                   //
                    ;                                                                                          //
            po::options_description feature_config("Feature Configuration");                                   //
            feature_config.add_options()                                                                       //
                    ("naive-feasibility", po::bool_switch(&naiveFeasibilityCheck),                             //
                     "Use naive feasibility check")                                                            //
                    ("naive-cycle", po::bool_switch(&naiveCycleCheck),                                         //
                     "Use naive cycle check")                                                                  //
                    ("only-cycle", po::bool_switch(&onlyCycleCheck),                                           //
                     "Use only cycle check")                                                                   //
                    ("feasibility-type", po::bool_switch(&feasibilityType),                                    //
                     "Classify feasibility types")                                                             //
                    ("prioritized-replan", po::bool_switch(&prioritizedReplan),                                //
                     "Use prioritized replan")                                                                 //
                    ("prioritized-opt", po::bool_switch(&prioritizedOpt),                                      //
                     "Use prioritized replan with optimization")                                               //
                    ("no-cache", po::bool_switch(&noCache),                                                    //
                     "Disable cache for map generator and solver")                                             //
                    ;                                                                                          //
            po::options_description io_config("I/O Configuration");                                            //
            io_config.add_options()                                                                            //
                    ("map-file", po::value<std::string>(&mapFile), "Map file")                                 //
                    ("task-file", po::value<std::string>(&taskFile), "Task file")                              //
                    ("log-file", po::value<std::string>(&logFile), "Log file")                                 //
                    ("output,o", po::value<std::string>(&outputFile), "Statistics output file")                //
                    ("simulator-output", po::value<std::string>(&simulatorOutputFile), "Simulator output file")//
                    ("time-output", po::value<std::string>(&timeOutputFile), "Time output File")               //
                    ;                                                                                          //
            po::options_description cmdline_options;
            cmdline_options.add(generic).add(core_config).add(simulation_config).add(feature_config).add(io_config);
            po::options_description config_file_options;
            config_file_options.add(core_config).add(simulation_config).add(feature_config).add(io_config);

            po::variables_map vm;
            po::store(po::command_line_parser(argc, argv).options(generic).allow_unregistered().run(), vm);
            po::notify(vm);

            if (vm.count("help")) {
                std::cerr << "Multi Agent Path Finding with Delay Probability" << std::endl << std::endl;
                std::cerr << "Usage: " << argv[0] << " [OPTIONS]" << std::endl;
                std::cerr << cmdline_options << std::endl;
                exit(0);
            }

            if (!configFile.empty()) {
                std::ifstream fin(configFile);
                if (!fin.is_open()) {
                    std::cerr << "error: can not open config file: " << configFile << std::endl;
                    exit(0);
                }
                po::store(po::parse_config_file(fin, config_file_options), vm);
            }
            po::store(po::parse_command_line(argc, argv, cmdline_options), vm);
            po::notify(vm);

            validate<unsigned long>(vm, "map-seed", 0);
            validate<unsigned long>(vm, "agent-seed", 0);
            validate<unsigned long>(vm, "simulation-seed", 0);
            validate<unsigned long>(vm, "agents", 1);
            validate<unsigned long>(vm, "iteration", 0);
            validate<unsigned long>(vm, "obstacles", 0);
            validate<unsigned long>(vm, "k-neighbor", 2);
            validate<double>(vm, "delay-ratio", 0);
            validate<double>(vm, "delay-interval", 0);

            /*for (const auto &it: vm) {
                std::cout << it.first.c_str() << " ";
                auto &value = it.second.value();
                if (auto v = boost::any_cast<bool>(&value))
                    std::cout << std::boolalpha << *v;
                else if (auto v = boost::any_cast<uint32_t>(&value))
                    std::cout << *v;
                else if (auto v = boost::any_cast<std::string>(&value))
                    std::cout << *v;
                else
                    std::cout << "error";
                std::cout << "\n";
            }*/

        } catch (std::exception &e) {
            std::cerr << e.what() << std::endl;
            exit(0);
        }
    }
};
