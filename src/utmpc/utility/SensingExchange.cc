#include "SensingExchange.h"

#include <omnetpp.h>

using std::string;
using std::vector;

namespace scentssim {

SensingExchange::SensingExchange(const string& str) {
    vector<string> tokens;
    std::size_t start = 0, end = 0;
    while ((end = str.find(kDelimiter, start)) != string::npos) {
        if (end != start) {
            tokens.push_back(str.substr(start, end - start));
        }
        start = end + 1;
    }
    if (end != start) {
        tokens.push_back(str.substr(start));
    }
    if (tokens.size() < 9) {
        throw omnetpp::cRuntimeError(
                "Error in length when constructing sensing exchange from string.");
    }
    local_id_ = std::stoi(tokens[0]);
    signal_type_ = (SignalType) std::stoi(tokens[1]);
    source_id_ = std::stoi(tokens[2]);
    dest_id_ = std::stoi(tokens[3]);
    query_id_ = std::stoi(tokens[4]);
    context_type_ = (ContextType) std::stoi(tokens[5]);
    content1_ = std::stod(tokens[6]);
    content2_ = std::stod(tokens[7]);
    answer_type_ = (AnswerType) std::stoi(tokens[8]);

}
SensingExchange::SensingExchange(const SensingExchange& a_record) {
    local_id_ = a_record.local_id_;
    signal_type_ = a_record.signal_type_;
    source_id_ = a_record.source_id_;
    dest_id_ = a_record.dest_id_;
    query_id_ = a_record.query_id_;
    context_type_ = a_record.context_type_;
    content1_ = a_record.content1_;
    content2_ = a_record.content2_;
    answer_type_ = a_record.answer_type_;
}

bool SensingExchange::operator==(const SensingExchange& other) const {
    return local_id_ == other.local_id_ && signal_type_ == other.signal_type_
            && source_id_ == other.source_id_ && dest_id_ == other.dest_id_
            && query_id_ == other.query_id_
            && context_type_ == other.context_type_
            && content1_ == other.content1_ && content2_ == other.content2_
            && answer_type_ == other.answer_type_;
}

bool SensingExchange::operator!=(const SensingExchange& other) const {
    return !(operator==(other));
}

string SensingExchange::str() const {
    char str[150];
    sprintf(str, "%d;%d;%d;%d;%d;%d;%f;%f;%d", local_id_, signal_type_,
            source_id_, dest_id_, query_id_, context_type_, content1_,
            content2_, answer_type_);
    return string(str);
}

int SensingExchange::get_local_id() const {
    return local_id_;
}
SignalType SensingExchange::get_signal_type() const {
    return signal_type_;
}
int SensingExchange::get_source_id() const {
    return source_id_;
}
int SensingExchange::get_dest_id() const {
    return dest_id_;
}
int SensingExchange::get_query_id() const {
    return query_id_;
}
ContextType SensingExchange::get_context_type() const {
    return context_type_;
}
double SensingExchange::get_content1() const {
    return content1_;
}
double SensingExchange::get_content2() const {
    return content2_;
}
AnswerType SensingExchange::get_answer_type() const {
    return answer_type_;
}

void SensingExchange::set_local_id(int l_id) {
    local_id_ = l_id;
}
void SensingExchange::set_signal_type(SignalType s_type) {
    signal_type_ = s_type;
}
void SensingExchange::set_source_id(int s_id) {
    source_id_ = s_id;
}
void SensingExchange::set_dest_id(int d_id) {
    dest_id_ = d_id;
}
void SensingExchange::set_query_id(int q_id) {
    query_id_ = q_id;
}
void SensingExchange::set_context_type(ContextType c_type) {
    context_type_ = c_type;
}
void SensingExchange::set_content1(double c1) {
    content1_ = c1;
}
void SensingExchange::set_content2(double c2) {
    content2_ = c2;
}
void SensingExchange::set_answer_type(AnswerType a_type) {
    answer_type_ = a_type;
}

} /* end of scentssim */
