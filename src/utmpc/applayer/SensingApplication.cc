/*
 * SensingApplication.cc
 *
 *      Author: liuchg1011
 */

#include "SensingApplication.h"

#include <algorithm>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <string>
#include <vector>

#include "inet/common/geometry/common/GeographicCoordinateSystem.h"
#include "inet/common/ModuleAccess.h"

using inet::Coord;
using inet::IMobility;
using inet::GeoCoord;
using inet::IGeographicCoordinateSystem;
using omnetpp::cMessage;
using omnetpp::cModule;
using omnetpp::cRuntimeError;
using omnetpp::cSimpleModule;
using omnetpp::simTime;
using omnetpp::SimTime;
using omnetpp::SIMTIME_S;
using omnetpp::SIMTIME_MS;
using omnetpp::simsignal_t;
using std::endl;
using std::vector;
using std::string;
using Eigen::MatrixXf;
using Eigen::VectorXf;

namespace scentssim {

Define_Module(SensingApplication);

// register signals for data collection
simsignal_t SensingApplication::sensing_signal = registerSignal(
        "sensingSignal");

int SensingApplication::num_query_generated = 0;

// Helper function for splitting strings
vector<string> split(const string &text, char sep) {
    vector<string> tokens;
    std::size_t start = 0, end = 0;
    while ((end = text.find(sep, start)) != string::npos) {
        if (end != start) {
            tokens.push_back(text.substr(start, end - start));
        }
        start = end + 1;
    }
    if (end != start) {
        tokens.push_back(text.substr(start));
    }
    return tokens;
}

uint32_t convertContextTypesToUint(const vector<ContextType>& type_vec,
        int type_size) {
    Bitmap bitmap { };
    for (const auto type : type_vec) {
        bitmap.set(type, true);
    }
    // EV_DEBUG << "Converted to bitmap:" << bitmap << endl;
    return bitmap.to_ulong();
}

vector<ContextType> convertUintToContextTypes(uint32_t int_val, int type_size) {
    Bitmap bitmap(int_val);
    vector<ContextType> res;
    for (int type = kTypeAcceleration; type < type_size; ++type) {
        if (bitmap.test(type))
            res.push_back((ContextType) type);
    }
    return res;
}

SensingApplication::SensingApplication() {
}

SensingApplication::~SensingApplication() {
    cancelAndDelete(query_msg_);
    cancelAndDelete(query_timer_);
    cancelAndDelete(start_timer_);
}

void SensingApplication::initialize() {
    cSimpleModule::initialize();

    node_id = par("nodeId");
    query_interval = SimTime(par("queryInterval").doubleValue(), SIMTIME_S);
    nd_lambda = SimTime(par("lambda").intValue(), SIMTIME_MS);

    query_msg_ = new cMessage("query-message");
    query_timer_ = new cMessage("timer-sensing-query");
    start_timer_ = new cMessage("start-timer");

    last_query_idx = 0;

    num_cache_hit = 0;
    num_request_rcvd = 0;
    num_request_sent = 0;
    num_response_sent = 0;
    num_response_rcvd = 0;
    WATCH(num_cache_hit);
    WATCH(num_request_rcvd);
    WATCH(num_request_sent);
    WATCH(num_response_rcvd);
    WATCH(num_response_sent);

    context_type_size = par("contextTypeSize");
    if (context_type_size > 8) {
        EV_INFO << "Extended (dummy) context types are used. (type size:"
                       << context_type_size << ")\n";
    }

    int cap_mode = par("capabilityMode");
    int numOmniNodes = par("numOmniNodes");
    if (numOmniNodes > 0 && node_id < (numOmniNodes / 10)) {
        cap_mode = kCapabilityOmni;
    }
    switch (cap_mode) {
    case kCapabilityNone: {
        break;
    }
    case kCapabilityOmni: {
        for (int type = kTypeAcceleration; type < context_type_size; ++type) {
            sensing_capabilities.push_back((ContextType) type);
        }
        break;
    }
    case kCapabilityRandomSubset: {
        int random_ratio = par("randomRatio");
        if (random_ratio == 0)
            break;
        else if (random_ratio < 0 || random_ratio > 100) {
            throw cRuntimeError(
                    "[App] Random ratio must be set when using this mode!");
        }
        // Randomly set RR*size bits to 1
        Bitmap bitmap { };
        while (bitmap.count() < ((float) random_ratio / 100) * context_type_size) {
            bitmap.set(intrand(context_type_size), true);
        }
        sensing_capabilities = convertUintToContextTypes(bitmap.to_ulong(),
                context_type_size);
        break;
    }
    case kCapabilitySubset: {
        vector<string> tokens = split(par("capSubset").stdstringValue(),
                kDelimiter);
        if (tokens.size() > 0) {
            EV_INFO
                           << "[App] Initializing sensing capabilities according to specified subset.\n";
            for (auto type_str : tokens) {
                sensing_capabilities.push_back(
                        (ContextType) std::stoi(type_str));
            }
        } else {
            throw cRuntimeError(
                    "[App] You must specify the subset(cap) when using this mode!");
        }
        break;
    }
    default:
        throw cRuntimeError("Unknown capability mode!");
    }
    dummy_peer_.node_id = node_id;
    dummy_peer_.caps = sensing_capabilities;
    dummy_peer_.stability = kDefaultStability;

    if (par("queryCapMatch").boolValue()) {
        query_types = sensing_capabilities;
    } else {
        int query_mode = par("queryMode");
        switch (query_mode) {
        case kQueryModeComplete: {
            for (int type = kTypeAcceleration; type < context_type_size;
                    ++type) {
                query_types.push_back((ContextType) type);
            }
            break;
        }
        case kQueryModeNone: {
            break;
        }
        case kQueryModeRandomSubset: {
            int random_ratio = par("randomRatio");
            if (random_ratio == 0)
                break;
            else if (random_ratio < 0 || random_ratio > 100) {
                throw cRuntimeError(
                        "[App] Random ratio must be set when using this mode!");
            }
            // Randomly set RR*size bits to 1
            Bitmap bitmap { };
            while (bitmap.count()
                    < ((float) random_ratio / 100) * context_type_size) {
                bitmap.set(intrand(context_type_size), true);
            }
            query_types = convertUintToContextTypes(bitmap.to_ulong(),
                    context_type_size);
            break;
        }
        case kQuerySubset: {
            vector<string> tokens = split(par("querySubset").stdstringValue(),
                    kDelimiter);
            if (tokens.size() > 0) {
                EV_INFO
                               << "[App] Initializing query types according to specified subset.\n";
                for (auto type_str : tokens) {
                    query_types.push_back((ContextType) std::stoi(type_str));
                }
            } else {
                throw cRuntimeError(
                        "[App] You must specify the subset(query) when using this mode!");
            }
            break;
        }
        default:
            throw cRuntimeError("[App] Unknown query mode!");
        }
    }

    string strategy = par("fulfillerStrategy").stdstringValue();
    if (strategy == "independent") {
        fulfiller_option = kOptionIndependent;
    } else if (strategy == "random_select") {
        fulfiller_option = kOptionRandomSelect;
    } else if (strategy == "optimized") {
        fulfiller_option = kOptionOptimized;
    } else {
        error("[App] Invalid fulfiller strategy: %s.", strategy.c_str());
    }

    // schedule start timer
    SimTime start_time = SimTime(par("startTime").doubleValue(), SIMTIME_S);
    scheduleAt(start_time, start_timer_);

    cModule* host = inet::getContainingNode(this);
    mob_ = omnetpp::check_and_cast<IMobility*>(host->getSubmodule("mobility"));

    EV_INFO << "[App] App initialized, scheduling the start timer as "
                   << start_time << endl;
    if (query_types.size() > 0) {
        EV_INFO << "[App] Query types: ";
        for (auto query_type : query_types) {
            EV_INFO << query_type << kDelimiter;
        }
        EV_INFO << endl;
    }
    if (sensing_capabilities.size() > 0) {
        EV_INFO << "[App] Sensing capabilities: ";
        for (auto s_cap : sensing_capabilities) {
            EV_INFO << s_cap << kDelimiter;
        }
        EV_INFO << endl;
    }
    EV_INFO << "[App] Fulfiller option: " << fulfiller_option << endl;
}

void SensingApplication::handleMessage(cMessage *msg) {
    if (msg->isSelfMessage()) {
        processTimer(msg);
    } else if (msg->arrivedOn("NDLayerIn")) {
        processMsgFromNDModule(msg);
        delete msg;
    }
}

void SensingApplication::processTimer(cMessage* timer) {
    SimTime next_query_time = simTime() + query_interval;
    if (timer == query_timer_) {
        if (query_types.size() > 0) {
            int query_type = query_types[intrand(query_types.size())];
            SensingExchange query = generateSensingQuery(
                    (ContextType) query_type);
            EV_INFO << "[App] Next query (type " << query_type
                           << ") is scheduled to " << next_query_time << "s.\n";
            emitRecord(query);
            handleSensingQuery(&query);
        } else {
            EV_DEBUG << "[App] Empty query types.\n";
        }
    } else if (timer == start_timer_ && query_types.size() > 0) {
        EV_INFO << "[App] Start to generate sensing queries. First query at "
                       << next_query_time << "s.\n";
    }
    scheduleAt(next_query_time, query_timer_);
}

void SensingApplication::processMsgFromNDModule(cMessage *msg) {
    if (msg->getKind() == kKindNewEpochBeacons) {
        //   refreshEpochBeacons(); //TODO(liuchg) this is from SASO
        return;
    }
    if (fulfiller_option == kOptionIndependent) {
        return;
    }
    string msg_str = msg->getName();
    if (msg_str.empty()) {
        EV_DEBUG << "[App] Got empty msg from ND. Updating candidate list...\n";
    } else {
        EV_INFO << "[App] Got msg from ND: " << msg_str << endl;
        // Reconstruct the exchange packets
        vector<string> exchange_str_vector = split(msg_str, kMsgDelimiter);
        for (const auto& ex_it : exchange_str_vector) {
            SensingExchange ex(ex_it);
            if (processed(ex)) {
                continue;
            } else {
                processed_queue_.push_back(ex);
                if (processed_queue_.size() > kProcessedQueueLength) {
                    processed_queue_.pop_front();
                }
                // process req. or res.
                switch (ex.get_signal_type()) {
                case kSignalTypeRequest: {
                    if (ex.get_dest_id() == node_id) {
                        ++num_request_rcvd;
                        handleSensingRequest((SensingExchange*) &ex);
                    } else if (ex.get_source_id() == node_id) {
                        continue;
                    } else {
                        sendMsgToNDModule(ex);
                    }
                    break;
                }
                case kSignalTypeResponse: {
                    ++num_response_rcvd;
                    processSensingResponse((SensingExchange*) &ex);
                    break;
                }
                default:
                    throw cRuntimeError(
                            "[App] Wrong type of message received from ND module!");
                }
            }
        }
    }

    int neighbor_id = kUninitializedInt;
    if (msg->hasPar("neighbor_id")) {
        neighbor_id = (int) msg->par("neighbor_id").longValue();
    }
    if (neighbor_id != kUninitializedInt) {
        auto it =
                std::find_if(peers_.begin(), peers_.end(),
                        [&](const SensingPeer& peer) -> bool {return peer.node_id == neighbor_id;});
        if (it == peers_.end()) {
            SensingPeer p;
            p.node_id = neighbor_id;
            vector<string> tokens = split(msg->par("cap").stringValue(),
                    kDelimiter);
            for (auto& token : tokens) {
                if (!token.empty())
                    p.caps.push_back((ContextType) std::stoi(token));
            }
            p.stability =
                    msg->hasPar("stability") ?
                            msg->par("stability").doubleValue() : 0;
            p.last_update_t = simTime();
            peers_.push_back(p);

        } else {
            (*it).stability =
                    msg->hasPar("stability") ?
                            msg->par("stability").doubleValue() : 0;
            (*it).last_update_t = simTime();
        }
    }
}

/**
 * Update the packet vector (capacity = 3) with priorities and construct
 *  the message byte array.
 * */
bool SensingApplication::sendMsgToNDModule(const SensingExchange& msg) {
    bool duplicated = false;
    /* Erase the expired message if there is any. */
    for (auto it = packet_vector_.begin(); it != packet_vector_.end();) {
        if ((*it).first == msg) {
            duplicated = true;
        }
        if (((*it).second + kPacketValidLambda * nd_lambda) <= simTime()) {
            it = packet_vector_.erase(it);
        } else {
            ++it;
        }
    }
    if (duplicated) {
        return true;
    }

    if (packet_vector_.size() < kMaxPacketCount) {
        packet_vector_.push_back(std::make_pair(msg, simTime()));
    } else {
        /* Find replaceable packets in the vector. */
        int rep_pkt_idx = kUninitializedInt;
        /* Look for relayed packet. */
        for (int i = 0; i < kMaxPacketCount; ++i) {
            if (packet_vector_.at(i).first.get_local_id() != node_id
                    || packet_vector_.at(i).first.get_source_id() != node_id) {
                rep_pkt_idx = i;
                break;
            }
        }
        /* Drop the packet if there is no room for the new response. */
        if (rep_pkt_idx == kUninitializedInt) {
            if (msg.get_source_id() == node_id
                    && msg.get_signal_type() == kSignalTypeResponse) {
                SensingExchange fail_record = msg;
                fail_record.set_signal_type(kSignalTypeFailedBeaconFull);
                emitRecord(fail_record);
            }
            // Will be deleted later.
            return false;
        }
        packet_vector_[rep_pkt_idx] = std::make_pair(msg, simTime());
    }
    // Pass down the byte array.
    string packet_str;
    for (const auto& it : packet_vector_) {
        packet_str += (it.first.str() + kMsgDelimiter);
    }
    if (!packet_str.empty()) {
        packet_str = packet_str.substr(0, packet_str.length() - 1);
    }
    cMessage *msg_to_nd = new cMessage(packet_str.c_str());
    send(msg_to_nd, "NDLayerOut");
    return true;
}

SensingExchange SensingApplication::generateSensingQuery(
        ContextType queryType) {
    SensingExchange query;
    query.set_local_id(node_id);
    query.set_signal_type(kSignalTypeQuery);
    query.set_context_type(queryType);
    query.set_query_id(num_query_generated++);

    query.set_source_id(node_id);
    query.set_dest_id(kUninitializedInt);
    query.set_content1(kUninitializedDouble);
    query.set_content2(kUninitializedDouble);
    query.set_answer_type(kAnswerTypeUninitialized);
    return query;
}

void SensingApplication::handleSensingQuery(SensingExchange* query) {
    ContextType sensor_type = query->get_context_type();
    if (localCheck(query)) {
        ++num_cache_hit;
        // record localCheck
        query->set_signal_type(kSignalTypeLocalCheck);
        emitRecord(*query);
        return;
    }

    // select sensing candidate
    int cand_id = selectCandidate(sensor_type);

    if (cand_id == kCandidateNotFound) {
        // no capable node in the neighborhood
        query->set_signal_type(kSignalTypeFailedNoCandidate);
        emitRecord(*query);
        return;
    } else {
        // pack a request and call handler
        query->set_signal_type(kSignalTypeRequest);
        query->set_source_id(node_id);
        query->set_dest_id(cand_id);
        // record request
        emitRecord(*query);
        if (cand_id == node_id) {
            handleSensingRequest(query);
        } else {
            bool load_success = sendMsgToNDModule(*query);
            EV_INFO << "Sending out sensing request to " << cand_id;
            if (load_success) {
                EV_INFO << " (success)\n";
                ++num_request_sent;
                /* Keep a record of the request. */
                if (request_map_.find(sensor_type) == request_map_.end()) {
                    request_map_[sensor_type] = query->get_query_id();
                }
            } else {
                /* Payload is full. Try to fulfill it locally (best effort). */
                EV_INFO << " (PAYLOAD IS FULL)!\n";
                if (std::find(sensing_capabilities.begin(),
                        sensing_capabilities.end(), sensor_type)
                        != sensing_capabilities.end()) {
                    EV_INFO << "Retry: going to check local sensors.\n";
                    handleSensingRequest(query);
                } else {
                    EV_INFO << "Retry failed.\n";
                }
            }

        }
    }
}

void SensingApplication::handleSensingRequest(SensingExchange* req) {
    if (localCheck(req)) {
        // answer with cached value
        ++num_cache_hit;
    } else {
        const ContextMsg sensor_reading = getSensorReading(
                req->get_context_type());
        req->set_content1(sensor_reading.get_content1());
        req->set_content2(sensor_reading.get_content2());
        req->set_answer_type(kOriginalAnswer);
        updateLocalContext(*req);
    }
    req->set_signal_type(kSignalTypeResponse);
    req->set_local_id(node_id);
    req->set_dest_id(req->get_source_id());
    req->set_source_id(node_id);

    sendMsgToNDModule(*req); // response
    EV_DEBUG << "[App] Request processed. (response is " << req->str()
                    << ").\n";
    emitRecord(*req);
}

void SensingApplication::processSensingResponse(SensingExchange* res) {
    int sender_id = res->get_source_id();
    updateLocalContext(*res);

    if (res->get_answer_type() == kOriginalAnswer) {
        auto beg = peers_.begin();
        auto end = peers_.end();
        auto it = std::find_if(beg, end, [sender_id](decltype(*beg) & vt) {
            return vt.node_id == sender_id;});
        if (end != it) {
            (*it).known_cost += cost_map_[res->get_context_type()];
        }
    }
    if (res->get_dest_id() == node_id) {
        res->set_local_id(node_id);
        EV_INFO << "[App] Received response " << " from " << sender_id << ":"
                       << res->str() << endl;
        emitRecord(*res);
        return;
    }
    /* Emit record if the response is what this node needs. */
    if (request_map_.find(res->get_context_type()) != request_map_.end()) {
        res->set_local_id(node_id);
        res->set_dest_id(node_id);
        res->set_query_id(request_map_[res->get_context_type()]);
        request_map_.erase(res->get_context_type());
        emitRecord(*res);
    }
}

ContextMsg SensingApplication::getSensorReading(ContextType contextType) {
    double content1 = kUninitializedDouble;
    double content2 = kUninitializedDouble;
    if (std::find(sensing_capabilities.begin(), sensing_capabilities.end(),
            contextType) == sensing_capabilities.end()) {
        throw cRuntimeError("[App] Matching capability not found.");
    }
    switch (contextType) {
    case kTypePosition: {
        content1 = kInitializedDouble;
        content2 = kInitializedDouble;

        if (existsStationary())
            break;

        HumanWPMobility* human_mob = nullptr;
        try {
            human_mob = omnetpp::check_and_cast<HumanWPMobility*>(mob_);
        } catch (cRuntimeError err) {
            EV_DEBUG
                            << "[App] getSensorReading() catch error: mobility is no HumanWP. Cannot use Geo coordinate. Return canvas coordinate instead.\n";
        }
        if (human_mob) {
            // Geo coordinate on map
            const GeoCoord& geoCoord =
                    human_mob->get_geographic_coordinate_system()->computeGeographicCoordinate(
                            human_mob->getCurrentPosition());
            content1 = geoCoord.latitude.get();
            content2 = geoCoord.longitude.get();
        } else {
            // Coordinate on canvas
            const Coord& pos = mob_->getCurrentPosition();
            content1 = pos.x;
            content2 = pos.y;
        }

        EV_DEBUG << "[App-GPS] checking position at node " << node_id << ": <"
                        << content1 << ", " << content2 << ">\n";

        break;
    }
    default: {
        if (contextType >= kContextTypeSize) {
            throw cRuntimeError(
                    "[App] Unknown context type when reading sensor.");
        }
        content1 = kInitializedDouble;
        content2 = kInitializedDouble;
    }
    }
    dummy_peer_.known_cost += cost_map_[contextType];

    return ContextMsg(node_id, contextType, content1, content2, simTime());
}

// Check local context pool for valid records.
// Argument is updated to a local check when returns true.
bool SensingApplication::localCheck(SensingExchange* query) {
    int sensor_type = query->get_context_type();
    if (cache_map_.find(sensor_type) != cache_map_.end()) {
        const ContextMsg& cached_record = cache_map_[sensor_type];
        if (simTime()
                <= cached_record.get_timestamp()
                        + SimTime(ttl_map_[sensor_type], SIMTIME_S)) {
            // mark the query finished (localCheck) and record
            query->set_signal_type(kSignalTypeLocalCheck);
            query->set_content1(cached_record.get_content1());
            query->set_content2(cached_record.get_content2());
            query->set_answer_type(kReusedAnswer);
            return true;
        }
    }
    return false;
}

int SensingApplication::selectCandidate(ContextType contextType) {
    if (fulfiller_option == kOptionIndependent) {
        auto it = std::find(sensing_capabilities.begin(),
                sensing_capabilities.end(), contextType);
        return (it != sensing_capabilities.end()) ? node_id : kCandidateNotFound;
    }
    int cand = kCandidateNotFound;
    vector<SensingPeer> capable_peers;
    if (std::find(sensing_capabilities.begin(), sensing_capabilities.end(),
            contextType) != sensing_capabilities.end()) {
        capable_peers.push_back(dummy_peer_);
    }
    for (const auto& it : peers_) {
        if (par("stationary").boolValue() == true && it.node_id == 0)
            continue;
        if (std::find(it.caps.begin(), it.caps.end(), contextType)
                != it.caps.end()) {
            if (it.last_update_t + kNumLambdaLost * nd_lambda >= simTime()) {
                // Filter out lost neighbors (different from decay).
                capable_peers.push_back(it);
            }
        }
    }
    if (capable_peers.empty()) {
        return kCandidateNotFound;
    }
    if (fulfiller_option == kOptionRandomSelect) {
        cand = capable_peers[intrand(capable_peers.size())].node_id;
        EV_DEBUG << "[App] Random select (capable) candidate: " << cand << endl;
    } else if (fulfiller_option == kOptionOptimized) {
        int m = capable_peers.size();
        assert(m > 0);
        int n = kContextTypeSize;
        float alpha = par("alpha").doubleValue();
        MatrixXf D = MatrixXf::Zero(n, n);
        VectorXf E = VectorXf::Zero(m);
        VectorXf L = VectorXf::Zero(m);
        C_ = MatrixXf::Zero(m, n);
        float e_len_s = (float) par("epochLength").intValue() / 1000;
        for (int i = 0; i < m; ++i) {
            E(i) = capable_peers[i].known_cost;
            L(i) = capable_peers[i].stability;
            float time_decay =
                    (simTime() - capable_peers[i].last_update_t).dbl()
                            / e_len_s;
            if (time_decay > 5) {
                float exp_factor = std::exp(-(time_decay - 5));
                L(i) = L(i) * exp_factor;
            }
        }

        for (int i = 0; i < m; ++i) {
            for (ContextType c : capable_peers[i].caps) {
                //debug
                //C_(i, c) = L(i);
                C_(i, c) = 1;
            }
        }

        VectorXf W = VectorXf::Zero(n);
        for (int j = 0; j < n; ++j) {
            W(j) = (float) C_.col(j).sum() / m;
            D(j, j) = W(j);
        }

        C_ = C_ * D;

        for (int i = 0; i < m; ++i) {
            float row_sum = (float) C_.row(i).sum();
            for (int j = 0; j < n; ++j) {
                C_(i, j) = C_(i, j) == 0 ? 0 : (C_(i, j) / row_sum);
                C_(i, j) = C_(i, j) == 0 ? 0 : (1 - C_(i, j));
            }
            row_sum = (float) C_.row(i).sum();
            for (int j = 0; j < n; ++j) {
                C_(i, j) = C_(i, j) == 0 ? 0 : (C_(i, j) / row_sum) * (L(i));
            }
        }

        VectorXf Aj = C_.col(contextType);

        // E
        float e_sum = E.colwise().sum().value();
        for (int i = 0; i < m; ++i) {
            E(i) = 1 - E(i) / e_sum;
        }

        VectorXf K = VectorXf::Zero(m);
        for (int i = 0; i < m; ++i) {
            K(i) = alpha * Aj(i) + (1 - alpha) * (1 - E(i));
        }

        float max_val = 0;
        int max_idx = 0;
        for (int i = 0; i < m; ++i) {
            if (K(i) >= max_val) {
                max_idx = i;
                max_val = K(i);
            }
        }

        cand = capable_peers[max_idx].node_id;
        EV_DEBUG << "[App " << node_id << "]Optimal candidate: " << cand
                        << ". [" << max_val << "]\n";
        EV_DEBUG << "[App] K: ";
        for (int i = 0; i < m; ++i)
            EV_DEBUG << K(i) << ", ";
        EV_DEBUG << endl;
    } else {
        throw cRuntimeError("[App] Unrecognized fulfiller option.");
    }
    return cand;
}

void SensingApplication::updateLocalContext(const SensingExchange& res) {
    ContextMsg context(res.get_source_id(), res.get_context_type(),
            res.get_content1(), res.get_content2(), simTime());
    cache_map_[res.get_context_type()] = context;
}

// VectorRecorder workaround for data collection
void SensingApplication::emitRecord(const SensingExchange& rec) {
    if (par("stationary").boolValue() == true)
        return;
    emit(sensing_signal, (double) rec.get_local_id());
    emit(sensing_signal, (double) rec.get_signal_type());
    emit(sensing_signal, (double) rec.get_source_id());
    emit(sensing_signal, (double) rec.get_dest_id());
    emit(sensing_signal, (double) rec.get_query_id());
    emit(sensing_signal, (double) rec.get_context_type());
    emit(sensing_signal, rec.get_content1());
    emit(sensing_signal, rec.get_content2());
    // Emit additional records for responses
    if (rec.get_signal_type() == kSignalTypeResponse) {
        if (existsStationary())
            return;
        ASSERT(rec.get_answer_type() != kAnswerTypeUninitialized);
        emit(sensing_signal, (double) rec.get_answer_type());
        double lat, lon;
        HumanWPMobility* human_mob = nullptr;
        try {
            human_mob = omnetpp::check_and_cast<HumanWPMobility*>(mob_);
        } catch (cRuntimeError err) {
            EV_DEBUG
                            << "[App] emitRecord() catch error: mobility is no HumanWP."
                            << " Cannot use Geo coordinate. Return canvas coordinate instead.\n";
        }
        if (human_mob) {
            const GeoCoord& geoCoord =
                    human_mob->get_geographic_coordinate_system()->computeGeographicCoordinate(
                            human_mob->getCurrentPosition());
            lat = geoCoord.latitude.get();
            lon = geoCoord.longitude.get();
        } else {
            const Coord& pos = mob_->getCurrentPosition();
            lat = pos.x;
            lon = pos.y;
        }
        emit(sensing_signal, lat);
        emit(sensing_signal, lon);
    }
}

bool SensingApplication::processed(const SensingExchange& obj) {
    for (const auto& it : processed_queue_) {
        if (it == obj)
            return true;
    }
    return false;
}

bool SensingApplication::existsStationary() {
    char mod_str[20];
    sprintf(mod_str, "stationary[0]");
    cModule* n_host = omnetpp::getSimulation()->getModuleByPath(mod_str);
    return n_host != nullptr;
}

// Initialize static member. Time unit: S.
std::map<int, int> SensingApplication::ttl_map_ = { { kTypeAcceleration,
        kTTLAcceleration }, { kTypeAirQuality, kTTLAirQuality }, {
        kTypeAtmosphericPressure, kTTLAtmosphericPressure }, { kTypeOrientation,
        kTTLOrientation }, { kTypeHumidity, kTTLHumidity }, { kTypeAmbientNoise,
        kTTLAmbientNoise }, { kTypeMotion, kTTLMotion }, { kTypePosition,
        kTTLPosition } };

std::map<int, float> SensingApplication::cost_map_ = { { kTypeAcceleration,
        kCostAcceleration }, { kTypeAirQuality, kCostAirQuality }, {
        kTypeAtmosphericPressure, kCostAtmosphericPressure }, {
        kTypeOrientation, kCostOrientation }, { kTypeHumidity, kCostHumidity },
        { kTypeAmbientNoise, kCostAmbientNoise }, { kTypeMotion, kCostMotion },
        { kTypePosition, kCostPosition } };
}

/* namespace scentssim */
