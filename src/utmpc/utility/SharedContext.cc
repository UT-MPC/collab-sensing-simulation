/*
 * SharedContext.cc
 *
 */

#include "SharedContext.h"

#include <iostream>
#include <omnetpp.h>
#include <vector>

using std::string;
using std::vector;

namespace scentssim {

SharedContext::SharedContext() {
}

SharedContext::SharedContext(uint8_t node_id, ContextType context_type,
        uint8_t lambda_left, uint32_t sensing_capabilities, uint32_t app_needs,
        double val1, double val2) {
    node_id_ = node_id;
    context_type_ = context_type;
    lambda_left_ = lambda_left;
    sensing_capabilities_ = sensing_capabilities;
    app_needs_ = app_needs;
    val1_ = val1;
    val2_ = val2;
    valid_ = true;
}

SharedContext::SharedContext(const SharedContext& rhs) {
    node_id_ = rhs.node_id_;
    context_type_ = rhs.context_type_;
    lambda_left_ = rhs.lambda_left_;
    sensing_capabilities_ = rhs.sensing_capabilities_;
    app_needs_ = rhs.app_needs_;
    val1_ = rhs.val1_;
    val2_ = rhs.val2_;
    valid_ = rhs.valid_;
}

SharedContext::SharedContext(const std::string& encoded) {
    vector<string> tokens;
    std::size_t start = 0, end = 0;
    while ((end = encoded.find(kDelimiter, start)) != string::npos) {
        if (end != start) {
            tokens.push_back(encoded.substr(start, end - start));
        }
        start = end + 1;
    }
    if (end != start) {
        tokens.push_back(encoded.substr(start));
    }
    if (tokens.size() < 7) {
        valid_ = false;
        return;
    }
    node_id_ = (uint8_t) std::stoi(tokens[0]);
    context_type_ = (ContextType) std::stoi(tokens[1]);
    lambda_left_ = (uint8_t) std::stoi(tokens[2]);
    sensing_capabilities_ = (uint32_t) std::stoi(tokens[3]);
    app_needs_ = (uint32_t) std::stoi(tokens[4]);
    val1_ = std::stod(tokens[5]);
    val2_ = std::stod(tokens[6]);
    valid_ = true;

}

SharedContext::~SharedContext() {
}

bool SharedContext::operator==(const SharedContext& rhs) const {
    return node_id_ == rhs.node_id_ && context_type_ == rhs.context_type_
            && lambda_left_ == rhs.lambda_left_
            && sensing_capabilities_ == rhs.sensing_capabilities_
            && app_needs_ == rhs.app_needs_ && val1_ == rhs.val1_
            && val2_ == rhs.val2_;
}

bool SharedContext::operator!=(const SharedContext& other) const {
    return !(operator==(other));
}

string SharedContext::encode() const {
    char str[50];
    sprintf(str, "%d;%d;%d;%d;%d;%f;%f", (int)node_id_, (int) context_type_,
            (int)lambda_left_, sensing_capabilities_, app_needs_, val1_, val2_);
    return string(str);
}

void SharedContext::countdown() {
    --lambda_left_;
}

bool SharedContext::is_valid() const {
    return valid_;
}

uint8_t SharedContext::get_node_id() const {
    return node_id_;
}

ContextType SharedContext::get_context_type() const {
    return context_type_;
}

uint8_t SharedContext::get_lambda_left() const {
    return lambda_left_;
}

uint32_t SharedContext::get_sensing_capabilities() const {
    return sensing_capabilities_;
}

uint32_t SharedContext::get_app_needs() const {
    return app_needs_;
}

double SharedContext::get_val1() const {
    return val1_;
}

double SharedContext::get_val2() const {
    return val2_;
}

void SharedContext::set_valid(bool valid) {
    valid_ = valid;
}

void SharedContext::set_node_id(uint8_t node_id) {
    node_id_ = node_id;
}

void SharedContext::set_context_type(ContextType context_type) {
    context_type_ = context_type;
}

void SharedContext::set_lambda_left(uint8_t lambda_left) {
    lambda_left_ = lambda_left;
}

void SharedContext::set_sensing_capabilities(uint32_t sensing_capabilities) {
    sensing_capabilities_ = sensing_capabilities;
}

void SharedContext::set_app_needs(uint32_t app_needs) {
    app_needs_ = app_needs;
}

void SharedContext::set_val1(double val1) {
    val1_ = val1;
}

void SharedContext::set_val2(double val2) {
    val2_ = val2;
}

} /* namespace scentssim */
