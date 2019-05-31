#pragma once

#include <mutex>
#include <vector>
#include <string>
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

    Regulator(std::string&& name);

    ~Regulator();
    
    void install(InputReader inr, OutputWriter outw);

    void process();

    void set(Num w);

    void set_params(Num p, Num s, Num d);

    void set_max_output(Num max);
    void set_zero_threshold(Num ths);
    void set_sum_zero_coef(Num coef);

    static void add_preprocessor(std::function<void()> fcn);
    static void add_postprocessor(std::function<void()> fcn);
private:
    bool clamp_output(Num& x);

    std::mutex m_mutex;

    const std::string m_name;

    InputReader m_reader;
    OutputWriter m_writer;

    Num m_p;
    Num m_s;
    Num m_d;
    
    Num m_e_zero_ths;
    Num m_s_zero_coef;

    int64_t m_t_last;

    Num m_sum;
    Num m_e_last;
    Num m_w;
    Num m_x_max;

    static void process_trampoline(void*);
    static void process_loop(void*);
    static Num abs(Num n);

    static std::mutex s_mutex;
    static std::vector<Regulator*> s_instances;
    static std::vector<std::function<void()> > s_preprocessors;
    static std::vector<std::function<void()> > s_postprocessors;
};

} // namespace rb
