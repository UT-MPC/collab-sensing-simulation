/*
 * ContextMsg.h
 *
 */

#ifndef UTMPC_UTILITY_CONTEXTMSG_H_
#define UTMPC_UTILITY_CONTEXTMSG_H_

#include <omnetpp.h>

#include "SensingExchange.h"

namespace scentssim {

class ContextMsg {
public:
    ContextMsg() {}
    ContextMsg(int sourceId, int cType, double cnt1, double cnt2,
            omnetpp::SimTime createTime) :
            source_id_(sourceId), context_type_(cType), content1_(cnt1), content2_(
                    cnt2), timestamp_(createTime) {
    }
    ~ContextMsg() {}

    double get_content1() const;
    double get_content2() const;
    int get_context_type() const;
    int get_source_id() const;
    const omnetpp::SimTime& get_timestamp() const;

    void set_content1(int c1);
    void set_content2(int c2);
    void set_context_type(int c_type);
    void set_source_id(int s_id);
    void set_timestamp(const omnetpp::SimTime& ts);

private:
    int source_id_;
    int context_type_;
    double content1_;
    double content2_;
    omnetpp::SimTime timestamp_;
};

} // end of scentssim

#endif /* UTMPC_UTILITY_CONTEXTMSG_H_ */
