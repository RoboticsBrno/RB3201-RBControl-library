#pragma once

#include <vector>
#include <mutex>
#include <functional>

namespace rb {

class Motor;

class Regulator {
public:
    typedef float Num;
    typedef std::function<Num()> InputReader;
    typedef std::function<void(Num)> OutputWriter;

    Regulator(Regulator const&) = delete;
    void operator=(Regulator const&) = delete;

    Regulator();

    ~Regulator();
    
    void install(InputReader inr, OutputWriter outw);

    void process();

    static void add_preprocessor(std::function<void()> fcn);
    static void add_postprocessor(std::function<void()> fcn);
private:
    InputReader m_reader;
    OutputWriter m_writer;

    static void process_trampoline();

    static std::mutex s_mutex;
    static std::vector<Regulator*> s_instances;
    static std::vector<std::function<void()> > s_preprocessors;
    static std::vector<std::function<void()> > s_postprocessors;
};

} // namespace rb
