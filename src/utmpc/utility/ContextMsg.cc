#include "ContextMsg.h"

using omnetpp::SimTime;

namespace scentssim {

double ContextMsg::get_content1() const {
    return content1_;
}
double ContextMsg::get_content2() const {
    return content2_;
}
int ContextMsg::get_context_type() const {
    return context_type_;
}
int ContextMsg::get_source_id() const {
    return source_id_;
}
const SimTime& ContextMsg::get_timestamp() const {
    return timestamp_;
}

void ContextMsg::set_content1(int c1) {
    content1_ = c1;
}
void ContextMsg::set_content2(int c2) {
    content2_ = c2;
}
void ContextMsg::set_context_type(int c_type) {
    context_type_ = c_type;
}
void ContextMsg::set_source_id(int s_id) {
    source_id_ = s_id;
}
void ContextMsg::set_timestamp(const SimTime& ts) {
    timestamp_ = ts;
}

} /* end of scentssim */
