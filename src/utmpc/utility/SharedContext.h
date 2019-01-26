/*
 * SharedContext.h
 *
 */

#ifndef UTMPC_UTILITY_SHAREDCONTEXT_H_
#define UTMPC_UTILITY_SHAREDCONTEXT_H_

#include <stdint.h>
#include <string>

#include "Constants.h"

namespace scentssim {

class SharedContext {
public:
    SharedContext();
    SharedContext(uint8_t node_id, ContextType context_type, uint8_t epoch_left,
            uint32_t sensing_capabilities, uint32_t app_needs, double val1,
            double val2);
    SharedContext(const SharedContext& rhs);
    SharedContext(const std::string& encoded);
    ~SharedContext();

    bool operator==(const SharedContext& other) const;
    bool operator!=(const SharedContext& other) const;

    std::string encode() const;
    void countdown();

    bool is_valid() const;
    uint8_t get_node_id() const;
    ContextType get_context_type() const;
    uint8_t get_lambda_left() const;
    uint32_t get_sensing_capabilities() const;
    uint32_t get_app_needs() const;
    double get_val1() const;
    double get_val2() const;

    void set_valid(bool valid);
    void set_node_id(uint8_t node_id);
    void set_context_type(ContextType context_type);
    void set_lambda_left(uint8_t lambda_left);
    void set_sensing_capabilities(uint32_t sensing_capabilities);
    void set_app_needs(uint32_t app_needs);
    void set_val1(double val1);
    void set_val2(double val2);

private:
    bool valid_;
    uint8_t node_id_;
    ContextType context_type_;
    uint8_t lambda_left_;
    uint32_t sensing_capabilities_;
    uint32_t app_needs_;
    double val1_;
    double val2_;

};

} /* namespace scentssim */

#endif /* UTMPC_UTILITY_SHAREDCONTEXT_H_ */
