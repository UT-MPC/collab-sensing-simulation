/*
 * SensingExchange.h
 *
 */

#ifndef UTMPC_UTILITY_SENSINGEXCHANGE_H_
#define UTMPC_UTILITY_SENSINGEXCHANGE_H_

#include <string>
#include <vector>

#include "Constants.h"

namespace scentssim {

class SensingExchange {
public:
    SensingExchange() {}
    SensingExchange(int local, SignalType s_type, int source, int dest, int q_id, ContextType c_type,
            double cnt1, double cnt2) :
            local_id_(local), signal_type_(s_type), source_id_(source), dest_id_(dest), query_id_(q_id), context_type_(
                    c_type), content1_(cnt1), content2_(cnt2) {
    }
    SensingExchange(const std::string& str);
    SensingExchange(const SensingExchange& rhs);

    ~SensingExchange() {}

    bool operator==(const SensingExchange& other) const;
    bool operator!=(const SensingExchange& other) const;

    std::string str() const;

    int get_local_id() const;
    SignalType get_signal_type() const;
    int get_source_id() const;
    int get_dest_id() const;
    int get_query_id() const;
    ContextType get_context_type() const;
    double get_content1() const;
    double get_content2() const;
    AnswerType get_answer_type() const;

    void set_local_id(int l_id);
    void set_signal_type(SignalType s_type);
    void set_source_id(int s_id);
    void set_dest_id(int d_id);
    void set_query_id(int q_id);
    void set_context_type(ContextType c_type);
    void set_content1(double c1);
    void set_content2(double c2);
    void set_answer_type(AnswerType a_type);
    
 private:
    int local_id_;
    SignalType signal_type_;
    int source_id_;
    int dest_id_;
    int query_id_;
    ContextType context_type_;
    double content1_;
    double content2_;
    // Members for response only
    AnswerType answer_type_;
};

} // end of scentssim

#endif /* UTMPC_UTILITY_SENSINGEXCHANGE_H_ */
