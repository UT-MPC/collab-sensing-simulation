/*
 * SensingApplication.h
 *
 *      Author: liuchg1011
 */

#ifndef UTMPC_APPLAYER_SENSINGAPPLICATION_H_
#define UTMPC_APPLAYER_SENSINGAPPLICATION_H_

#include <omnetpp.h>

#include <bitset>
#include <deque>
#include <eigen3/Eigen/Dense>
#include <map>
#include <vector>

#include "inet/mobility/contract/IMobility.h"
#include "../mobility/HumanWPMobility.h"
#include "../utility/Constants.h"
#include "../utility/ContextMsg.h"
#include "../utility/SensingExchange.h"

namespace scentssim {

typedef std::bitset<kMaxContextTypes> Bitmap;

class SensingApplication: public omnetpp::cSimpleModule {
public:
    SensingApplication();
    virtual ~SensingApplication();

    virtual void initialize();
    virtual void handleMessage(omnetpp::cMessage *msg);
    virtual void processTimer(omnetpp::cMessage *timer);

    virtual void processMsgFromNDModule(omnetpp::cMessage *msg);
    virtual bool sendMsgToNDModule(const SensingExchange& msg);

    SensingExchange generateSensingQuery(ContextType queryType);
    void handleSensingQuery(SensingExchange* query);
    void handleSensingRequest(SensingExchange* req);
    void processSensingResponse(SensingExchange* res);

    ContextMsg getSensorReading(ContextType contextType);
    bool localCheck(SensingExchange* query);
    int selectCandidate(ContextType contextType);
    void updateLocalContext(const SensingExchange& context);

    void emitRecord(const SensingExchange& rec);

    static omnetpp::simsignal_t sensing_signal;

    std::vector<ContextType> query_types;
    std::vector<ContextType> sensing_capabilities;

    int context_type_size;
    int last_query_idx;
    int node_id;
    omnetpp::SimTime nd_lambda;
    omnetpp::SimTime query_interval;
    FulfillerStrategy fulfiller_option;

    long num_cache_hit;
    static int num_query_generated;
    long num_request_rcvd;
    long num_request_sent;
    long num_response_rcvd;
    long num_response_sent;

private:
    class SensingPeer {
    public:
      int node_id;
      std::vector<ContextType> caps;
      float known_cost;
      float stability;
      omnetpp::SimTime last_update_t;
    };
    bool processed(const SensingExchange& obj);
    bool existsStationary();
    omnetpp::cMessage *query_msg_;
    omnetpp::cMessage *query_timer_;
    omnetpp::cMessage *start_timer_;
    static std::map<int, int> ttl_map_;
    static std::map<int, float> cost_map_;

    std::map<int, ContextMsg> cache_map_;
    std::map<int, int> request_map_;  /* Keep track of the queries awaiting for response. */
    std::deque<SensingExchange> processed_queue_;
    inet::IMobility *mob_;
    std::vector<SensingPeer> peers_;
    SensingPeer dummy_peer_;
    Eigen::MatrixXf C_;
    std::vector<std::pair<SensingExchange, omnetpp::SimTime>> packet_vector_;
};

} /* namespace scentssim */

#endif /* UTMPC_APPLAYER_SENSINGAPPLICATION_H_ */
